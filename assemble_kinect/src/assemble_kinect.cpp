#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <shared_autonomy_msgs/KinectAssembly.h>

using namespace sensor_msgs;

typedef message_filters::sync_policies::ApproximateTime<Image, Image, CameraInfo, PointCloud2> KinectSyncPolicy;

class KinectAssembler {

private:
  // root_nh is so we can advertise a service / listen to topics NOT in remapped namespace
  // priv_nh is for topics/parameters that we DO want pushed down
  ros::NodeHandle root_nh_;
  ros::NodeHandle priv_nh_;
  
  ros::ServiceServer kinect_srv_;

  message_filters::Subscriber<Image> image_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Subscriber<CameraInfo> info_sub_;
  message_filters::Subscriber<PointCloud2> points_sub_;

  message_filters::Synchronizer<KinectSyncPolicy> sync_;

  // ------------- Callbacks --------------
  void approxCB(const ImageConstPtr& image, const ImageConstPtr& depth, 
		const CameraInfoConstPtr& cam_info, const PointCloud2ConstPtr& points);
  bool serviceCB(shared_autonomy_msgs::KinectAssembly::Request &req,
		 shared_autonomy_msgs::KinectAssembly::Response &res);
  
  // -------------- Data --------------
  shared_autonomy_msgs::KinectAssembly::Response resp_;
  bool has_data_;

public:
  KinectAssembler();
  ~KinectAssembler();

};

// TODO: Is this good practice to init everything like this?

KinectAssembler::KinectAssembler() : 
  root_nh_(""), priv_nh_("~"),
  // We expect these to be remapped in the launch file, using the same arg as openni_launch. 
  image_sub_(root_nh_, "camera/rgb/image_color", 1),
  depth_sub_(root_nh_, "camera/depth_registered/image", 1),
  info_sub_(root_nh_, "camera/depth_registered/camera_info", 1),
  points_sub_(root_nh_, "camera/depth_registered/points", 1),
  // 10 is queue size for the synching approach
  sync_(KinectSyncPolicy(100), image_sub_, depth_sub_, info_sub_, points_sub_) {

  sync_.registerCallback(boost::bind(&KinectAssembler::approxCB, this, _1, _2, _3, _4));

  // TODO: rgbd_assembler had something like "resolveName" here ...
  kinect_srv_ = root_nh_.advertiseService("camera/assemble_kinect", &KinectAssembler::serviceCB, this);
  has_data_ = false;
  ROS_INFO("KinectAssembler started");

}

KinectAssembler::~KinectAssembler() {
}

// TODO: Do I want fancier logic here where we can only send a given set of data once?
void KinectAssembler::approxCB(const ImageConstPtr& image, const ImageConstPtr& depth, const CameraInfoConstPtr& info, const PointCloud2ConstPtr& points) {
  resp_.image = *image;
  resp_.depth = *depth;
  resp_.info = *info;
  resp_.points = *points;
  has_data_ = true;
}

bool KinectAssembler::serviceCB(shared_autonomy_msgs::KinectAssembly::Request &req,
	       shared_autonomy_msgs::KinectAssembly::Response &res) {
  res = resp_;
  ROS_INFO("service callback called!");

  // check that data was initialized
  if(has_data_){
    return true;
  }
  else {
    return false;
  }
}
	       

int main(int argc, char** argv) {
  ros::init(argc, argv, "assemble_kinect_node");
  KinectAssembler ka_node;

  ros::spin();
  return 0;

}
