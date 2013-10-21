#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <grabcut_3d/grabcut_3d.h>

//#include <pcl/point_cloud.h> // from tutorial
#include <pcl/point_types.h> // from tutorial AND ben's code
#include <pcl/kdtree/kdtree_flann.h> // guessing, from kdtree http://docs.pointclouds.org/trunk/classpcl_1_1_kd_tree.html
//#include <pcl/features/feature.h> // from ben's code
//#include <pcl/registration/transforms.h> // from ben's code

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <shared_autonomy_msgs/BoundingBoxAction.h>
#include <shared_autonomy_msgs/EditPixelAction.h>
#include <shared_autonomy_msgs/Pixel.h>
#include <shared_autonomy_msgs/SegmentAction.h>

class Grabcut3dSegmentation {

protected:
  ros::NodeHandle root_nh_;
  ros::NodeHandle priv_nh_;
  ros::Publisher depth_pub_;
  ros::Publisher interp_pub_;
  actionlib::SimpleActionServer<shared_autonomy_msgs::SegmentAction> segment_server_;
  actionlib::SimpleActionClient<shared_autonomy_msgs::BoundingBoxAction> bb_client_;
  actionlib::SimpleActionClient<shared_autonomy_msgs::EditPixelAction> label_client_;

  bool getPixelLabels(const cv_bridge::CvImage &image_bridge, const cv_bridge::CvImage &mask_bridge, 
                      std::vector<shared_autonomy_msgs::Pixel> &foreground_pixels,
                      std::vector<shared_autonomy_msgs::Pixel> &background_pixels);
  bool getBoundingBox(const cv_bridge::CvImage &image,
                      int &min_col, int &max_col, int &min_row, int &max_row);
  void maskFromBB(cv_bridge::CvImage &mask_bridge, int min_col, int max_col, int min_row, int max_row);
  void grabcutMaskFromBB(const cv_bridge::CvImage &rgb_bridge, const cv_bridge::CvImage &depth_bridge, 
                         cv_bridge::CvImage &mask_bridge, int min_col, int max_col, int min_row, int max_row);
  void grabcutMaskFromPixels(const cv_bridge::CvImage &rgb_bridge, const cv_bridge::CvImage &depth_bridge, 
                             cv_bridge::CvImage &mask_bridge,
                             const std::vector<shared_autonomy_msgs::Pixel>& foreground_pixels,
                             const std::vector<shared_autonomy_msgs::Pixel>& background_pixels);
  bool matFromRGBMessage(const sensor_msgs::Image& image, cv::Mat& mat);
  bool matFromDepthMessage(const sensor_msgs::Image& image, cv::Mat& mat);
  bool checkHMIConnected();
  void interpolateDepthImageNearestNeighbor(cv::Mat& depth_image);

  // -------------------- parameters ------------------
  double loop_rate_;
  double preempt_wait_; // how long to give servers to cancel the goal
  double connect_wait_; // how long to wait when trying to initially connect
  int grabcut_iters_; // how many iterations to allow grabcut3D
  double min_range_; // parameters for scaling depth image from 32FC1 -> 8UC1
  double max_range_;
  // TODO: This suggests to me that maybe we should have the radius on the GUI
  // side of this, and take in an exact list of pixels, or a mask? 
  int click_radius_; // how many pixels around a mouse click are set to fg/bg

public:
  Grabcut3dSegmentation(std::string segment_name, std::string bb_name, std::string label_name);
  ~Grabcut3dSegmentation(void);
  void segmentExecuteCB(const shared_autonomy_msgs::SegmentGoalConstPtr &goal);

};

Grabcut3dSegmentation::Grabcut3dSegmentation(std::string segment_name, std::string bb_name, std::string label_name)  : 
  root_nh_(""), priv_nh_("~"),
  segment_server_(root_nh_, segment_name, boost::bind(&Grabcut3dSegmentation::segmentExecuteCB, this, _1), false), 
  bb_client_(bb_name, true), 
  label_client_(label_name, true)
{
  // initialize all parameters
  priv_nh_.param<double>(std::string("loop_rate"), loop_rate_, 10.0);
  priv_nh_.param<double>(std::string("preempt_wait"), preempt_wait_, 15.0);
  priv_nh_.param<double>(std::string("connect_wait"), connect_wait_, 20.0);
  priv_nh_.param<int>(std::string("grabcut_iters"), grabcut_iters_, 5);
  priv_nh_.param<double>(std::string("min_range"), min_range_, 0.0);
  priv_nh_.param<double>(std::string("max_range"), max_range_, 2.5);
  priv_nh_.param<int>(std::string("click_radius"), click_radius_, 5);

  ROS_INFO("grabcut3D segmentation initialized");

  segment_server_.start();

  depth_pub_ = root_nh_.advertise<sensor_msgs::Image>("original_depth", 5);
  interp_pub_ = root_nh_.advertise<sensor_msgs::Image>("interpolated_depth", 5);

}

Grabcut3dSegmentation::~Grabcut3dSegmentation(void) {

}

bool Grabcut3dSegmentation::getPixelLabels(const cv_bridge::CvImage &image_bridge, const cv_bridge::CvImage &mask_bridge,
                                           std::vector<shared_autonomy_msgs::Pixel> &foreground_pixels,
                                           std::vector<shared_autonomy_msgs::Pixel> &background_pixels) {

  ROS_INFO("grabcut3d_segmentation in getPixelLabels");
  foreground_pixels.clear();
  background_pixels.clear();

  bool segment_preempted = false;

  // Package up segmentation goal for the HMI, and send it out
  shared_autonomy_msgs::EditPixelGoal label_goal;
  shared_autonomy_msgs::EditPixelResult label_result;
  image_bridge.toImageMsg(label_goal.image);
  mask_bridge.toImageMsg(label_goal.mask);
  label_client_.sendGoal(label_goal);

  ROS_INFO("grabcut3d_segmentation sent edit pixel goal");

  // wait for bounding box result OR preemption
  ros::Rate rr(loop_rate_);
  while (!label_client_.getState().isDone() and ros::ok() and !segment_preempted) {
    segment_preempted = segment_server_.isPreemptRequested();
    rr.sleep();
  } 

  if(label_client_.getState().isDone()) {
    if(label_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      shared_autonomy_msgs::EditPixelResultConstPtr myresult = label_client_.getResult();
      std::vector<shared_autonomy_msgs::Pixel>::const_iterator it;
      for(it = myresult->fg.begin(); it != myresult->fg.end(); it++) {
	foreground_pixels.push_back(*it);
      }
      for(it = myresult->bg.begin(); it != myresult->bg.end(); it++) {
	background_pixels.push_back(*it);
      }
      return true;
    }
    else { // preempted, aborted, rejected, etc. ...
      ROS_INFO("grabcut3d_segmentation edit pixel client returned UNsuccessfully");
      segment_server_.setAborted();
      return false;
    }
  }
  else if (segment_preempted) {
    ROS_INFO("grabcut3d_segmentation - preempt requested");
    label_client_.cancelGoal();
    label_client_.waitForResult(ros::Duration(preempt_wait_));
    segment_server_.setPreempted();
    return false;
  }
  else {
    // Only way to get here should be if !ros::ok(), in which case we also quit
    ROS_INFO("quitting b/c ros::ok() false.");
    segment_server_.setAborted();
    return false;
  }
}

bool Grabcut3dSegmentation::getBoundingBox(const cv_bridge::CvImage &image_bridge,
                                           int &min_col, int &max_col, int &min_row, int &max_row) {

  ROS_INFO("grabcut3d_segmentation in getBoundingBox");
  bool segment_preempted = false;

  shared_autonomy_msgs::BoundingBoxGoal bb_goal;
  shared_autonomy_msgs::BoundingBoxResult bb_result;
  image_bridge.toImageMsg(bb_goal.image);
  bb_client_.sendGoal(bb_goal);

  ROS_INFO("grabcut3d_segmentation sent goal");

  // wait for bounding box result OR preemption
  ros::Rate rr(loop_rate_);
  while (!bb_client_.getState().isDone() and ros::ok() and !segment_preempted) {
    segment_preempted = segment_server_.isPreemptRequested();
    rr.sleep();
  } 

  ROS_INFO("grabcut3d_segmentation broke out of loop waiting for bbox result");

  if(bb_client_.getState().isDone()) {
    if(bb_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("grabcut3d_segmentation's bbox client returned successfully");
      bb_result = *bb_client_.getResult();
      min_col = bb_result.min_col.data;
      max_col = bb_result.max_col.data;
      min_row = bb_result.min_row.data;
      max_row = bb_result.max_row.data;
      return true;
    }
    else { // preempted, aborted, rejected, etc. ...
      ROS_INFO("grabcut3d_segmentation bbox client returned UNsuccessfully");
      segment_server_.setAborted();
      return false;
    }
  }
  else if (segment_preempted) {
    ROS_INFO("grabcut3d_segmentation - preempt requested");
    //request preempt 
    bb_client_.cancelGoal();
    bb_client_.waitForResult(ros::Duration(preempt_wait_));
    segment_server_.setPreempted();
    return false;
  }
  else {
    // Only way to get here should be if !ros::ok(), in which case we also quit
    ROS_INFO("quitting b/c ros::ok() false.");
    segment_server_.setAborted();
    return false;
  }
}

// fill mask in w/ bounds from bbox call
// TODO: try using this, and varying the rule for what it's filled as (probable/definite)
void Grabcut3dSegmentation::maskFromBB(cv_bridge::CvImage &mask_bridge, int min_col, int max_col, int min_row, int max_row) {
  cv::Point p1 = cv::Point(min_col, min_row);
  cv::Point p2 = cv::Point(max_col, max_row);
  cv::rectangle(mask_bridge.image, p1, p2, 1, CV_FILLED);
}

/*
 * Helper function to convert ROS sensor images to openCV mat images
 * (Originally from object_segmentation_ui)
 * @param image_msg - ROS sensor image
 * @param image - opencv Mat image
 *
 */
bool Grabcut3dSegmentation::matFromRGBMessage(const sensor_msgs::Image& image_msg, cv::Mat& image) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg);
    ROS_INFO("DEPTH encoding: %s", cv_ptr->encoding.c_str());
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  image = cv_ptr->image;
  return true;
}

bool Grabcut3dSegmentation::matFromDepthMessage(const sensor_msgs::Image& depth_msg, cv::Mat& depth) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(depth_msg);
    ROS_INFO("DEPTH encoding: %s", cv_ptr->encoding.c_str());
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  // handling depth images, and manually converting them, as described in:
  // http://answers.ros.org/question/10222/openni_camera-depth-image-opencv/
  // TODO: test that this image makes sense (publish it and listen on image_view?)
  double scaled_depth;
  if(cv_ptr->encoding != "32FC1") {
    ROS_ERROR("Grabcut3dSegmentation::matFromDepthMessage expects depth to be 32FC1");
    return false;
  } 
  else {
    depth = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    for(int i = 0; i < cv_ptr->image.rows; i++) {
      float* Di = cv_ptr->image.ptr<float>(i);
      char* Ii = depth.ptr<char>(i);
      for(int j = 0; j < cv_ptr->image.cols; j++) {
        // need to preserve the NaN depths...
        if(Di[j] > 0) {
          scaled_depth = std::min(1.0, std::max(0.0, (Di[j]-min_range_)/(max_range_-min_range_)));
          Ii[j] = (char) (254*scaled_depth);
          Ii[j] = (char) 0;
        } 
        else {
          Ii[j] = (char) 255; // white!
        }
      }   
    }
  }
  return true;
}

// fill mask in w/ result from calling grabcut_3d w/ bounding box
void Grabcut3dSegmentation::grabcutMaskFromBB(const cv_bridge::CvImage &rgb_bridge, const cv_bridge::CvImage &depth_bridge, 
                                              cv_bridge::CvImage &mask_bridge, int min_col, int max_col, int min_row, int max_row) {
  
  // we're initializing from rect, so fill it in w/ input bounds
  cv::Rect rect = cv::Rect(min_col, min_row, max_col-min_col, max_row-min_row); 

  //grabcut3d will build its own models
  cv::Mat bgd_model;
  cv::Mat fgd_model;

  //TODO: does this init as probable or forced for things outside of the rectangle?
  grabCut3D(rgb_bridge.image, depth_bridge.image, mask_bridge.image, 
            rect, bgd_model, fgd_model, grabcut_iters_, cv::GC_INIT_WITH_RECT);
}

// updates the input mask with new classification based on input pixel labels
void Grabcut3dSegmentation::grabcutMaskFromPixels(const cv_bridge::CvImage &rgb_bridge, const cv_bridge::CvImage &depth_bridge, 
                                                  cv_bridge::CvImage &mask_bridge,
                                                  const std::vector<shared_autonomy_msgs::Pixel>& foreground_pixels,
                                                  const std::vector<shared_autonomy_msgs::Pixel>& background_pixels) {
  // TODO: make this a parameter
  std::vector<shared_autonomy_msgs::Pixel>::const_iterator it;
  for(it = foreground_pixels.begin(); it != foreground_pixels.end(); it++) {
    cv::Point pp = cv::Point((*it).u, (*it).v);
    cv::circle(mask_bridge.image, pp, click_radius_, cv::GC_FGD, CV_FILLED);
  }
  for(it = background_pixels.begin(); it != background_pixels.end(); it++) {
    cv::Point pp = cv::Point((*it).u, (*it).v);
    cv::circle(mask_bridge.image, pp, click_radius_, cv::GC_BGD, CV_FILLED);
  }

  cv::Rect rect;
  cv::Mat bgd_model;
  cv::Mat fgd_model;

  grabCut3D(rgb_bridge.image, depth_bridge.image, mask_bridge.image, 
            rect, bgd_model, fgd_model, grabcut_iters_, cv::GC_INIT_WITH_MASK);

}

bool Grabcut3dSegmentation::checkHMIConnected() {
  if(!bb_client_.isServerConnected()) {
    bool bb_ready = bb_client_.waitForServer(ros::Duration(connect_wait_));
    if(!bb_ready) {
      segment_server_.setAborted();
      ROS_WARN("grabcut3dsegmentation could not execute segmentation - HMI bounding box not connected!");
      return false;
    }
  }
  if(!label_client_.isServerConnected()) {
    bool label_ready = label_client_.waitForServer(ros::Duration(connect_wait_));
    if(!label_ready) {
      segment_server_.setAborted();
      ROS_WARN("grabcut3dsegmentation could not execute segmentation - HMI pixel labeller not connected!");
      return false;
    }
  }
  ROS_INFO("grabcut3d_segmentation got HMI server!");
  return true;
}

// This function copied from bosch-internal/stacks/shared_autonomy/grabcut_3d_and_2d/src/grabcut_3d_app.cpp
void Grabcut3dSegmentation::interpolateDepthImageNearestNeighbor(cv::Mat& depth_image) {
  // TODo: need cleaner way of communicating that this is nan depth =(
  // it's currently also hard-coded in mat from depth message, and depends on the driver's conventions
  char nan_val = 255;
  // construct cloud used for NN calculations
  pcl::PointCloud<pcl::PointXY> cloud;
  cv::Size size = depth_image.size();
  int point_id = 0;
  for(int row = 0; row< size.height; row++) {
    char* lineptr = depth_image.ptr<char>(row);
    for(int col = 0; col < size.width; col++) {
      if(lineptr[col] != nan_val) { // TODO: are all NANs represented by 0's in the code? make variable!
        pcl::PointXY pt;
        pt.x = (float) col;
        pt.y = (float) row;
        cloud.points.push_back(pt);
      }
    }
  }
  
  // setting up the kdtree for interpolation
  pcl::KdTreeFLANN<pcl::PointXY> kdtree; 
  int kk = 1; // how many neighbors we'll ask nearestKSearch for
  std::vector<int> k_indices(kk);
  std::vector<float> k_distances(kk);

  pcl::PointCloud<pcl::PointXY>::ConstPtr cloud_ptr = boost::make_shared< const pcl::PointCloud<pcl::PointXY> > (cloud);
  kdtree.setInputCloud(cloud_ptr);

  // actually do interpolation
  for(int row = 0; row < size.height; row++) {
    char* lineptr = depth_image.ptr<char>(row);
    for(int col = 0; col < size.width; col++) {
      if(lineptr[col] == nan_val) {
        pcl::PointXY pt;
        pt.x = (float) col;
        pt.y = (float) row;
        kdtree.nearestKSearch(pt, kk, k_indices, k_distances);
        // TODO: make this a parameter
        if(k_distances[0] < 20.0) { // only consider points w/in sqrt(20) dist
          const pcl::PointXY& closest_point = cloud.points[k_indices[0]];
          lineptr[col] = depth_image.at<char>(closest_point.y, closest_point.x);
        }
      }
    }
  }
}

  // TODO: This needs better sub-functions... have them return bool for success, and return if not?
  // * check HMI server up
  // * obtain bounding_box
  // * bbox bounds to mask
  // * obtain labels/acceptance
  // * update mask w/ labels

//TODO: I kind of dislike how all the helper functions are the ones that actually handle
// setting the actionlib server status, but it got too cumbersome to have it at the top level
// TODO: checking every time if we foo_succeeded before continuing is ugly. better alternative?

void Grabcut3dSegmentation::segmentExecuteCB(const shared_autonomy_msgs::SegmentGoalConstPtr &segment_goal) {
  ROS_INFO("grabcut3d_segmentation::segmentExecuteCB called");

  // local storage of data is in cv_bridge::CvImage format. 
  // Conversions to ROS happen as close as possible to the callback/publish function
  cv::Mat rgb_mat;
  matFromRGBMessage(segment_goal->image, rgb_mat);
  cv_bridge::CvImage rgb_bridge(segment_goal->image.header, segment_goal->image.encoding, rgb_mat);

  // depth images get an additional interpolation step
  cv::Mat depth_mat;
  matFromDepthMessage(segment_goal->depth, depth_mat);
  cv_bridge::CvImage tmp_bridge(segment_goal->depth.header, "mono8", depth_mat);
  //testing the interpolation!
  sensor_msgs::Image tmp_image;
  tmp_bridge.toImageMsg(tmp_image);
  depth_pub_.publish(tmp_image);
  ROS_INFO("published depth image!");
  
  interpolateDepthImageNearestNeighbor(depth_mat);  
  cv_bridge::CvImage depth_bridge(segment_goal->depth.header, "mono8", depth_mat);
  //testing the interpolation!
  sensor_msgs::Image depth_image;
  depth_bridge.toImageMsg(depth_image);
  interp_pub_.publish(depth_image);
  ROS_INFO("published depth image!");

  cv::Mat mask_mat = cv::Mat::zeros(segment_goal->image.height, segment_goal->image.width, CV_8UC1);
  cv_bridge::CvImage mask_bridge(segment_goal->image.header, "mono8", mask_mat);

  // Check if HMI server is up; if not, we can't do segmentation
  bool hmi_server_connected = checkHMIConnected();
  if(!hmi_server_connected) {
    return;
  }

  // Get initial bounding box; this SETS col/row values ...
  int min_col, max_col, min_row, max_row;
  bool bb_succeeded = getBoundingBox(rgb_bridge, min_col, max_col, min_row, max_row);
  if(!bb_succeeded) {
    return;
  }

  // initialize mask from rectangle or initial segmentation
  grabcutMaskFromBB(rgb_bridge, depth_bridge, mask_bridge, min_col, max_col, min_row, max_row);
  ROS_INFO("rectangle added to cv::Mat");

  std::vector<shared_autonomy_msgs::Pixel> foreground_pixels;
  std::vector<shared_autonomy_msgs::Pixel> background_pixels;
  shared_autonomy_msgs::Pixel mypixel;

  bool label_succeeded = getPixelLabels(rgb_bridge, mask_bridge, foreground_pixels, background_pixels);
  if(!label_succeeded) {
    return;
  }
  while (!(foreground_pixels.empty() and background_pixels.empty())) {
    grabcutMaskFromPixels(rgb_bridge, depth_bridge, mask_bridge, foreground_pixels, background_pixels);
    label_succeeded = getPixelLabels(rgb_bridge, mask_bridge, foreground_pixels, background_pixels);
    if (!label_succeeded) {
      return;
    }
  }

  // return segmentation result
  shared_autonomy_msgs::SegmentResult segmentation_result;
  mask_bridge.toImageMsg(segmentation_result.mask);
  segment_server_.setSucceeded(segmentation_result);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "grabcut3d_segmentation_node");

  Grabcut3dSegmentation segmenter("segment_service", "bbox_service", "pixel_service");

  ros::spin();
  return 0;
}
