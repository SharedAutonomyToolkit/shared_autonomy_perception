#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <shared_autonomy_msgs/KinectAssembly.h>

using namespace sensor_msgs;

typedef message_filters::sync_policies::ApproximateTime<Image, Image, CameraInfo, PointCloud2> sync_policy;

void approxCB(const ImageConstPtr& image, const ImageConstPtr& depth, const CameraInfoConstPtr& cam_info, const PointCloud2ConstPtr& points) {
  //ROS_INFO("assemble_kinect callback called!");

}

void exactCB(const ImageConstPtr& depth, const CameraInfoConstPtr& cam_info, const PointCloud2ConstPtr& points) {

  //ROS_INFO("assemble_kinect callback called, w/o rgb image!");
}

bool serviceCB(shared_autonomy_msgs::KinectAssembly::Request &req,
	       shared_autonomy_msgs::KinectAssembly::Response &res) {

  ROS_INFO("service callback called!");
  return true;
}
	       


int main(int argc, char** argv) {
  ros::init(argc, argv, "assemble_kinect_node");
  ros::NodeHandle nh;

  message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_color", 1);
  message_filters::Subscriber<Image> depth_sub(nh, "/camera/depth_registered/image", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "/camera/depth_registered/camera_info", 1);
  message_filters::Subscriber<PointCloud2> points_sub(nh, "/camera/depth_registered/points", 1);

  message_filters::Synchronizer<sync_policy> sync(sync_policy(10), image_sub, depth_sub, info_sub, points_sub);
  sync.registerCallback(boost::bind(&approxCB, _1, _2, _3, _4));

  message_filters::TimeSynchronizer<Image, CameraInfo, PointCloud2> sync2(depth_sub, info_sub, points_sub, 10);
  sync2.registerCallback(boost::bind(&exactCB, _1, _2, _3));

  ros::ServiceServer service = nh.advertiseService("assemble_kinect", serviceCB);

  ros::spin();
  return 0;

}
