/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tabletop_object_detector/marker_generator.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tabletop_object_detector/TabletopDetectionResult.h>
#include <tabletop_object_detector/TabletopDetection.h>

#include "augmented_object_detector/GC3DApplication.hpp"
#include "augmented_object_detector/DatabaseModelComplete.h"
#include "augmented_object_detector/DetectObject.h"

using namespace std;
using namespace cv;
using namespace tabletop_object_detector;

class GrabCutNode
{
public:
  std::string node_name_;
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! A tf transform listener
  tf::TransformListener listener_;
  // for database fitting
  ros::ServiceClient detect_object_client;
  image_transport::CameraPublisher mask_pub;
  //! Service server for object detection
  ros::ServiceServer object_detection_srv_;

  // converts OpenCV to ROS images
  sensor_msgs::CvBridge img_bridge_;
  // converts OpenCV to ROS images
  sensor_msgs::CvBridge depth_img_bridge_;

  // image transport
  image_transport::ImageTransport it;

  // Create image and info message objects for segmentation mask topic
  sensor_msgs::Image mask_img_msg;

  //! Publisher for markers
  ros::Publisher marker_pub_;
  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;
  //! Subscriber for rgbd images
  image_transport::Subscriber image_sub_;
  Mat image_;

  float quality_threshold;
  bool use_database;
  std::string detection_frame;


  GrabCutNode(const std::string& node_name,ros::NodeHandle& nh)
  : node_name_(node_name), nh_(nh), priv_nh_("~"), listener_(ros::Duration(120.0)), it(nh)
  {
    ROS_INFO("grabcut_node: starting");

    // marker
    num_markers_published_ = 1;
    current_marker_id_ = 1;
    marker_pub_ = nh.advertise<visualization_msgs::Marker>(nh.resolveName("markers_out"), 10);

    quality_threshold=0.005;

    // Check whether database use is requested
    priv_nh_.param("use_database",use_database,false);
    priv_nh_.param("detection_frame",detection_frame,string("/base_link"));

    priv_nh_.param("use_database",use_database,false);
    if (use_database) {
      ROS_INFO("grabcut_node: subscribing to object detection service");
      string detect_object_service_name;
      priv_nh_.param<string>("detect_object_service_name", detect_object_service_name, string("detect_object"));
      detect_object_client = nh.serviceClient<augmented_object_detector::DetectObject>(detect_object_service_name, true);
      if (!detect_object_client.exists()) detect_object_client.waitForExistence();
      quality_threshold=0.005;
    }

    // Create image publisher for segmentation mask
    mask_pub = it.advertiseCamera( nh.getNamespace() + "/" + node_name_ + "/mask_image/image_raw", 3 );

    // object detection server
    ROS_INFO("grabcut_node: advertising object_detection service");
    object_detection_srv_ = nh_.advertiseService(nh_.resolveName("object_detection_srv"),
                                                  &GrabCutNode::serviceCallback, this);

    std::string image_topic = nh_.resolveName("image_in");
    image_sub_ = it.subscribe(image_topic, 1 , &GrabCutNode::imageCallback, this);
  }

  //! Empty stub
  ~GrabCutNode() {}

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    sensor_msgs::CvBridge bridge;
   /* try
    {
      image_ = bridge.imgMsgToCv(msg, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }*/
  }

  /* Scales camera extrinsics by a factor to allow for conversion of camera
   * parameters between pyramid levels. */
  void scaleCameraInfo(sensor_msgs::CameraInfo& info, float scale)
  {
    info.height = info.height / scale;
    info.width  = info.width / scale;
    info.roi.height = info.roi.height / scale;
    info.roi.width  = info.roi.width / scale;
    info.K = info.K;
    for (int i=0; i<8; i++) info.K[i] /= scale;
    info.P = info.P;
    for (int i=0; i<10; i++) info.P[i] /= scale;
  }

  /* Publishes image and camera info */
  void
  publishImage(
      const image_transport::CameraPublisher& pub,
      sensor_msgs::CameraInfo& info_msg,
      sensor_msgs::Image& img_msg,
      const cv::Mat& image,
      const std::string& encoding)
  {
    fillImage(img_msg, encoding, image.rows, image.cols, image.step, const_cast<uint8_t*>(image.data));
    pub.publish(img_msg, info_msg);
  }

  /* Filter received point cloud based on binary mask */
  void
  filterPointCloud(const sensor_msgs::CameraInfo& info_msg,
          const cv::Mat& mask,
          pcl::PointCloud<pcl::PointXYZRGB>& cloud)
  {
    image_geometry::PinholeCameraModel model_;
    model_.fromCameraInfo(info_msg);
    int width = cloud.width;
    int height = cloud.height;

    ROS_INFO("grab_view: cloud size before filtering is %d",
            int(cloud.points.size()));
    pcl::PointCloud<pcl::PointXYZRGB>::VectorType::iterator iter;
    pcl::PointCloud<pcl::PointXYZRGB>::VectorType new_points;
    new_points.reserve(cloud.points.size());
    int point_id = 0;
    for (iter = cloud.points.begin(); iter != cloud.points.end(); ++iter)
    {
      point_id++;
      pcl::PointXYZRGB point = *iter;
      if (isnan(point.x) || isnan(point.y) || isnan(point.z))
        continue;
      cv::Point3d p3d(point.x, point.y, point.z);
      cv::Point2d p2d;
      model_.project3dToPixel(p3d, p2d);
      int x = round(p2d.x);
      int y = round(p2d.y);
      int mask_val = int(mask.at<unsigned char>(y, x));
      if (mask_val > 0)
        new_points.push_back(point);
    }
    ROS_INFO("grab_view: cloud size after filtering is %d",
            int(new_points.size()));

    cloud.width = 0;
    cloud.height = 0;
    cloud.is_dense = 0;
    cloud.points = new_points;

    return;
  }


  bool getPointCloud(sensor_msgs::PointCloud2& cloud_msg, pcl::PointCloud<pcl::PointXYZRGB>& cloud)
  {
    // get camera info
    std::string topic = nh_.resolveName("cloud_in");
    ROS_INFO("Waiting for a point_cloud2 on topic %s", topic.c_str());
    sensor_msgs::PointCloud2::ConstPtr cloud_msg_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(10.0));
    if(!cloud_msg_ptr) {
      ROS_ERROR("Could not receive a point cloud!");
    }
    else {
      cloud_msg = *cloud_msg_ptr;
      pcl::fromROSMsg<pcl::PointXYZRGB>(cloud_msg, cloud);
      ROS_INFO("Got point cloud!");
    }
    return true;
  }

  bool getCameraInfo(sensor_msgs::CameraInfo& camera_info_msg)
  {
    // get camera info
    std::string camera_info_topic = nh_.resolveName("camera_info_in");
    ROS_INFO("Waiting for camera_info on topic %s", camera_info_topic.c_str());
    sensor_msgs::CameraInfoConstPtr camera_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, nh_, ros::Duration(10.0));
    if(!camera_info_ptr) {
      ROS_ERROR("Could not receive a camera info!");
    }
    else {
      camera_info_msg = *camera_info_ptr;
      // Scale the camera info message to match the pyramid scaling
      scaleCameraInfo(camera_info_msg, 4);
    }
    return true;
  }

  bool getImage(sensor_msgs::Image& image_msg, Mat& image)
  {
    // get image
    std::string image_topic = nh_.resolveName("image_in");
    ROS_INFO("Waiting for image on topic %s", image_topic.c_str());
    sensor_msgs::ImageConstPtr image_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic, nh_, ros::Duration(10.0));
    if(!image_ptr) {
      ROS_ERROR("Could not receive an image!");
      return false;
    }
    image_msg = *image_ptr;
    IplImage* ipl_image;
    try {
      ipl_image = img_bridge_.imgMsgToCv(image_ptr, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException e) {
      ROS_ERROR("%s: Unable to convert %s image to bgr8",
          node_name_.c_str(),
          image_ptr->encoding.c_str());
      return false;
    }
    image = ipl_image;

    return true;
  }

  bool getDepthImage(sensor_msgs::Image& image_msg, Mat& image)
  {
    // get image
    std::string image_topic = nh_.resolveName("depth_image_in");
    ROS_INFO("Waiting for image on topic %s", image_topic.c_str());
    sensor_msgs::ImageConstPtr image_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic, nh_, ros::Duration(10.0));
    if(!image_ptr) {
      ROS_ERROR("Could not receive an image!");
      return false;
    }
    image_msg = *image_ptr;
    IplImage* ipl_image;
    try {
      ipl_image = depth_img_bridge_.imgMsgToCv(image_ptr, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException e) {
      ROS_ERROR("%s: Unable to convert %s image",
          node_name_.c_str(),
          image_ptr->encoding.c_str());
      return false;
    }
    image = ipl_image;

    return true;
  }

  bool serviceCallback(TabletopDetection::Request &request, TabletopDetection::Response &response) {

    // get point cloud
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    getPointCloud(cloud_msg,cloud);

    // get image
    sensor_msgs::Image image_msg;
    Mat image;
    getImage(image_msg, image);

    // get depth image
    sensor_msgs::Image depth_image_msg;
    Mat depth_image;
    getDepthImage(depth_image_msg, depth_image);

    // get camera info
    sensor_msgs::CameraInfo camera_info_msg;
    getCameraInfo(camera_info_msg);

    // Initialize GC3DApplication object
    GC3DApplication gcapp("grab cut 3D", image, depth_image);

    // Wait for instructions
    while (ros::ok())
    {
      int c = cvWaitKey(0);
      switch( (char) c )
      {
        // r: reset segmentation
        case 'r':
          cout << endl;
          gcapp.initializedIs(false);
          break;

        // n: run next iteration
        case 'n':
          {
            cout << "<" << gcapp.iterCount() << "... ";
            if( gcapp.rectState() == GC3DApplication::SET)
            {
              gcapp.iterCountInc();
              cout << gcapp.iterCount() << ">" << endl;
            }
            else
                cout << "rect must be determined>" << endl;
            break;
          }

        // d: set default rectangle
        case 'd':
          gcapp.rectIs(GC3DApplication::DEFAULT_RECT);
          break;

        // u: get new image from camera
        case 'u':
          // get point cloud
          getPointCloud(cloud_msg,cloud);
          // get image
          getImage(image_msg, image);
          // get camera info
          getCameraInfo(camera_info_msg);
          break;

        // w, k, y, g, b: change background color
        case 'w':
          cout << "Setting background color to white" << endl;
          gcapp.winColorIs(GC3DApplication::WHITE);
          break;
        case 'k':
          cout << "Setting background color to black" << endl;
          gcapp.winColorIs(GC3DApplication::BLACK);
          break;
        case 'y':
          cout << "Setting background color to gray" << endl;
          gcapp.winColorIs(GC3DApplication::GRAY);
          break;
        case 'g':
          cout << "Setting background color to green" << endl;
          gcapp.winColorIs(GC3DApplication::GREEN);
          break;
        case 'b':
          cout << "Setting background color to blue" << endl;
          gcapp.winColorIs(GC3DApplication::BLUE);
          break;

        // 0, 1, 2, 3, 4, 5: change pyramid level
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
          {
//            int plevel = (int)((char)c - '0');
//            pcam.pyrLevelIs(plevel);
//            cout << "Changing pyramid level to " << plevel << endl;
//            gcapp.imageIs(pcam.image());
            break;
          }

        // p: publish segmentation mask on outgoing topic
        case 'p':
          {
            // Fetch binary mask from GCApp object
            cv::Mat binary_mask = gcapp.binaryMask();

            // process point cloud
            // fill service response
            TabletopDetectionResult& detection_message = response.detection;
            processPointCloud(cloud_msg, image_msg, camera_info_msg, binary_mask, detection_message);

            cout << "Publishing mask image" << endl;
            // Get image header from incoming image message
            mask_img_msg.header.stamp = image_msg.header.stamp;
            mask_img_msg.header.frame_id = image_msg.header.frame_id;
            // NB: publishing mask*255 so that it can be viewed as a monochrome
            // image to check the mask.
            publishImage(mask_pub, camera_info_msg, mask_img_msg, binary_mask*255, sensor_msgs::image_encodings::TYPE_8UC1);
            return true;
            break;
          }
      }

    }

    return true;
  }

  bool transformPointCloud(const std::string &target_frame, const  pcl::PointCloud<pcl::PointXYZRGB>& cloud_in,  pcl::PointCloud<pcl::PointXYZRGB>& cloud_out)
  {
    //convert cloud to processing frame
    string err_msg;
    try
    {
      pcl_ros::transformPointCloud(target_frame, cloud_in,  cloud_out, listener_);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("Failed to transform cloud from frame %s into frame %s: %s", cloud_in.header.frame_id.c_str(),target_frame.c_str(),ex.what());
      return false;
    }
    ROS_INFO("Input cloud converted to %s frame", target_frame.c_str());
    return true;
  }

  bool transformPointCloud(const std::string &target_frame, const sensor_msgs::PointCloud2& cloud_in, sensor_msgs::PointCloud2& cloud_out)
  {
    //convert cloud to processing frame
    sensor_msgs::PointCloud old_cloud,old_cloud_transformed;
    sensor_msgs::convertPointCloud2ToPointCloud (cloud_in, old_cloud);
    string err_msg;
    if (listener_.canTransform(target_frame.c_str(),cloud_in.header.frame_id.c_str(), ros::Time(0), &err_msg))
    {
      try
      {
        listener_.transformPointCloud(target_frame, old_cloud, old_cloud_transformed);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("Failed to transform cloud from frame %s into frame %s: %s", old_cloud.header.frame_id.c_str(),target_frame.c_str(),ex.what());
        return false;
      }
    }
    else
    {
      ROS_ERROR("Could not transform %s to %s: %s", cloud_in.header.frame_id.c_str(), target_frame.c_str(), err_msg.c_str());
      return false;
    }
    sensor_msgs::convertPointCloudToPointCloud2 (old_cloud_transformed, cloud_out);
    ROS_INFO("Input cloud converted to %s frame", target_frame.c_str());
    return true;
  }

  bool processPointCloud(const sensor_msgs::PointCloud2& cloud_msg, const sensor_msgs::Image& image_msg, sensor_msgs::CameraInfo& camera_info_msg, cv::Mat& binary_mask, TabletopDetectionResult &detection_message)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud, converted_cloud, detection_cloud;
    pcl::fromROSMsg<pcl::PointXYZRGB>(cloud_msg, cloud);

    // convert cloud to optical frame
    ROS_INFO("Transforming point cloud to %s frame", image_msg.header.frame_id.c_str());
    transformPointCloud(image_msg.header.frame_id, cloud, converted_cloud);

    // Filter point cloud based on scaled camera info and mask
    ROS_INFO("Filtering point cloud with grab cut mask");
    static const char WINDOW[] = "processPointCloud Mask";
    cv::namedWindow(WINDOW);
    cv::imshow(WINDOW, binary_mask*50);
    cv::waitKey();

    filterPointCloud(camera_info_msg, binary_mask, converted_cloud);
    if (converted_cloud.points.empty()) {
      ROS_ERROR("grabcut_node: No points left after filtering");
      return false;
    }

    // transform cloud to detection frame
    ROS_INFO("Transforming point cloud to %s frame", detection_frame.c_str());
    transformPointCloud(detection_frame, converted_cloud, detection_cloud);

    sensor_msgs::PointCloud2 detection_cloud_msg;
    pcl::toROSMsg<pcl::PointXYZRGB>(detection_cloud, detection_cloud_msg);


    // If requested, query database for best fit model(s)
    if (use_database && !detection_cloud.points.empty())
    {
      ROS_INFO("Query database for object fit");
      int num_results = 1;
      augmented_object_detector::DetectObject detect_object;
      detect_object.request.cloud = detection_cloud_msg;
      detect_object.request.num_results = num_results;
      if (!detect_object_client.call(detect_object) ||
          detect_object.response.status.code
            != detect_object.response.status.SUCCESS )
      {
        ROS_ERROR("grab_view: DetectObject service call failed (error %i)", detect_object.response.status.code);
        return false;
      }

      vector<augmented_object_detector::DatabaseModelComplete> model_list = detect_object.response.model_list;
      augmented_object_detector::DatabaseModelComplete& model = model_list[0];

      ROS_INFO("grab_view: best fit object is %i (%s)", model.model_id, model.name.c_str());

      // Generate marker
      visualization_msgs::Marker object_marker = MarkerGenerator::getFitMarker(model.mesh, model.score);
      object_marker.header = detection_cloud_msg.header;
      object_marker.color.r = 1.0;
      object_marker.color.g = 0.0;
      object_marker.color.b = 0.0;

      // Get model pose
      geometry_msgs::PoseStamped marker_pose;
      listener_.transformPose("/base_link", model.pose, marker_pose);
      object_marker.pose = marker_pose.pose;
      marker_pub_.publish(object_marker);

      household_objects_database_msgs::DatabaseModelPose pose_message;
      pose_message.model_id = model.model_id;
      //model is published in incoming cloud frame
      pose_message.pose.header = detection_cloud_msg.header;
      pose_message.pose = model.pose;
      household_objects_database_msgs::DatabaseModelPoseList pose_list;
      pose_list.model_list.push_back(pose_message);
      //transform is identity since now objects have their own reference frame
      detection_message.models.push_back(pose_list);
    }

    // fill service response

    detection_message.result = detection_message.SUCCESS;
    detection_message.cluster_model_indices = std::vector<int>(1, -1);
    detection_message.cluster_model_indices[0] = 0;
    sensor_msgs::PointCloud cluster;
    sensor_msgs::convertPointCloud2ToPointCloud(detection_cloud_msg,cluster);
    detection_message.clusters.push_back(cluster);

    // publish markers
    publishClusterMarkers<sensor_msgs::PointCloud>(detection_message.clusters, detection_cloud_msg.header);
    clearOldMarkers(cloud_msg.header.frame_id);

    return true;
  }

  /* Helper functions */
  void usage()
  {
    ROS_WARN("./grabcut_node [img fmt] [transport] camera:=<camera namespace> ");
    exit(1);
  }

  //! Publishes rviz markers for the given tabletop clusters
  template <class PointCloudType>
  void publishClusterMarkers(const std::vector<PointCloudType> &clusters,
          roslib::Header cloud_header)
  {
   geometry_msgs::Pose table_pose;
   //tf::poseTFToMsg(table_plane_trans, table_pose);
   for (size_t i=0; i<clusters.size()
   ; i++)
   {
     ROS_INFO("Cloud size: %d",(int)clusters[i].points.size());
     visualization_msgs::Marker cloud_marker =  MarkerGenerator::getCloudMarker(clusters[i]);
     cloud_marker.header = cloud_header;
     //set the marker in the pose of the table
     //cloud_marker.pose = table_pose;
     cloud_marker.ns = "tabletop_node";
     cloud_marker.id = current_marker_id_++;
     marker_pub_.publish(cloud_marker);
   }
  }

  void clearOldMarkers(std::string frame_id)
  {
   for (int id=current_marker_id_; id < num_markers_published_; id++)
   {
     visualization_msgs::Marker delete_marker;
     delete_marker.header.stamp = ros::Time::now();
     delete_marker.header.frame_id = frame_id;
     delete_marker.id = id;
     delete_marker.action = visualization_msgs::Marker::DELETE;
     delete_marker.ns = "tabletop_node";
     marker_pub_.publish(delete_marker);
   }
   num_markers_published_ = current_marker_id_;
   current_marker_id_ = 0;
  }

};

void print_directions()
{
  cout << "First, select the rectangular area\n" <<
      "Hot keys: \n"
      "\tESC - quit the program\n"
      "\tr - restore the original image\n"
      "\tu - update image (get new frame from camera)\n"
      "\tn - next iteration\n"
      "\t0-5 - switch pyramid level for current image\n"
      "\n"
      "Background colors: \n"
      "\tWhite (w), Black (k), Gray (y), Green (g), Blue (b)\n"
      "\n"
      "Segmentation hint controls: \n"
      "\tleft mouse button - set rectangle\n"
      "\n"
      "\tCTRL+left mouse button - set GC_BGD pixels\n"
      "\tSHIFT+left mouse button - set CG_FGD pixels\n"
      "\n"
      "\tCTRL+right mouse button - set GC_PR_BGD pixels\n"
      "\tSHIFT+right mouse button - set CG_PR_FGD pixels\n";
}



int main( int argc, char** argv )
{
  string node_name = "grabcut_tabletop_node";
  ros::init(argc, argv, node_name);

  // Run grab cut node
  ros::NodeHandle nh;
  GrabCutNode node(node_name,nh);

  // Tell user how this works
  //print_directions();

  // ...and spin
  ros::spin();

  return 0;
}
