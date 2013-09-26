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

class KinectAssembler {

private:
  // root_nh is so we can advertise a service / listen to topics NOT in remapped namespace
  // priv_nh is for topics/parameters that we DO want pushed down
  ros::NodeHandle root_nh_;
  ros::NodeHandle priv_nh_;
  
  ros::ServiceServer kinect_srv_;

  // ------------- Callbacks --------------
  void approxCB(const ImageConstPtr& image, const ImageConstPtr& depth, 
		const CameraInfoConstPtr& cam_info, const PointCloud2ConstPtr& points);
  void exactCB(const ImageConstPtr& depth, const CameraInfoConstPtr& cam_info, 
	       const PointCloud2ConstPtr& points);
  bool serviceCB(shared_autonomy_msgs::KinectAssembly::Request &req,
		 shared_autonomy_msgs::KinectAssembly::Response &res);
  
  // -------------- Data --------------
  Image image_;
  Image depth_;
  CameraInfo info_;
  PointCloud2 points_;
  // TODO: Do I need any mutexes here s.t. I can't accidentally respond 
  // to a service request halfway through updating the data?

public:
  KinectAssembler();
  ~KinectAssembler();


};

KinectAssembler::KinectAssembler() : root_nh_(""), priv_nh_("~") {

  message_filters::Subscriber<Image> image_sub(root_nh_, "/camera/rgb/image_color", 1);
  message_filters::Subscriber<Image> depth_sub(root_nh_, "/camera/depth_registered/image", 1);
  message_filters::Subscriber<CameraInfo> info_sub(root_nh_, "/camera/depth_registered/camera_info", 1);
  message_filters::Subscriber<PointCloud2> points_sub(root_nh_, "/camera/depth_registered/points", 1);

  message_filters::Synchronizer<sync_policy> sync(sync_policy(10), image_sub, 
						  depth_sub, info_sub, points_sub);
  //sync.registerCallback(boost::bind(&approxCB, _1, _2, _3, _4));

  message_filters::TimeSynchronizer<Image, CameraInfo, PointCloud2> sync2(depth_sub, 
									  info_sub, points_sub, 10);
  //sync2.registerCallback(boost::bind(&exactCB, _1, _2, _3));



  // TODO: rgbd_assembler had something like "revolveName" here ...
  kinect_srv_ = root_nh_.advertiseService("assemble_kinect", &KinectAssembler::serviceCB, this);
  ROS_INFO("KinectAssembler started");

}

KinectAssembler::~KinectAssembler() {
}

void KinectAssembler::approxCB(const ImageConstPtr& image, const ImageConstPtr& depth, const CameraInfoConstPtr& cam_info, const PointCloud2ConstPtr& points) {
  //ROS_INFO("assemble_kinect callback called!");

}

void KinectAssembler::exactCB(const ImageConstPtr& depth, const CameraInfoConstPtr& cam_info, const PointCloud2ConstPtr& points) {

  //ROS_INFO("assemble_kinect callback called, w/o rgb image!");
}

bool KinectAssembler::serviceCB(shared_autonomy_msgs::KinectAssembly::Request &req,
	       shared_autonomy_msgs::KinectAssembly::Response &res) {

  ROS_INFO("service callback called!");
  return true;
}
	       


int main(int argc, char** argv) {
  ros::init(argc, argv, "assemble_kinect_node");
  KinectAssembler ka_node;

  ros::spin();
  return 0;

}
