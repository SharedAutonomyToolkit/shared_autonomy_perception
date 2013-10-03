#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <grabcut_3d/grabcut_3d.h>
#include <sensor_msgs/image_encodings.h>
#include <shared_autonomy_msgs/BoundingBoxAction.h>
#include <shared_autonomy_msgs/EditPixelAction.h>
#include <shared_autonomy_msgs/Pixel.h>
#include <shared_autonomy_msgs/SegmentAction.h>

class Grabcut3dSegmentation {

protected:
  ros::NodeHandle root_nh_;
  ros::NodeHandle priv_nh_;
  actionlib::SimpleActionServer<shared_autonomy_msgs::SegmentAction> segment_server_;
  actionlib::SimpleActionClient<shared_autonomy_msgs::BoundingBoxAction> bb_client_;
  actionlib::SimpleActionClient<shared_autonomy_msgs::EditPixelAction> label_client_;
  std::string action_name_;
  shared_autonomy_msgs::SegmentResult segmentation_result_;

  bool getPixelLabels(const sensor_msgs::Image& image, const sensor_msgs::Image& mask,
		      std::vector<shared_autonomy_msgs::Pixel> &foreground_pixels,
		      std::vector<shared_autonomy_msgs::Pixel> &background_pixels);
  bool getBoundingBox(const sensor_msgs::Image& image, 
		      int *min_col, int *max_col, int *min_row, int *max_row);
  void maskFromBB(cv::Mat &mask, int min_col, int max_col, int min_row, int max_row);
  void grabcutMaskFromBB(const sensor_msgs::Image& image, cv::Mat &mask, int min_col, int max_col, int min_row, int max_row);
  void grabcutMaskFromPixels(const sensor_msgs::Image& image, cv_bridge::CvImage &mask_bridge,
			     std::vector<shared_autonomy_msgs::Pixel> foreground_pixels,
			     std::vector<shared_autonomy_msgs::Pixel> background_pixels);
  bool matFromImageMessage(const sensor_msgs::Image& image, cv::Mat& mat);
  //bool imageMessageFromMat(sensor_msgs::Image& image, const cv::Mat& mat);
  
  bool checkHMIConnected();

  // -------------------- parameters ------------------
  double loop_rate_;
  double preempt_wait_; // how long to give servers to cancel the goal
  double connect_wait_; // how long to wait when trying to initially connect
  int grabcut_iters_; // how many iterations to allow grabcut3D

public:
  Grabcut3dSegmentation(std::string segment_name, std::string bb_name, std::string label_name);
  ~Grabcut3dSegmentation(void);
  void segmentExecuteCB(const shared_autonomy_msgs::SegmentGoalConstPtr &goal);

};

Grabcut3dSegmentation::Grabcut3dSegmentation(std::string segment_name, std::string bb_name, std::string label_name)  : 
  root_nh_(""), priv_nh_("~"),
  segment_server_(root_nh_, segment_name, boost::bind(&Grabcut3dSegmentation::segmentExecuteCB, this, _1), false), 
  bb_client_(bb_name, true), 
  label_client_(label_name, true),
  action_name_(segment_name) 
{
  // initialize all parameters
  priv_nh_.param<double>(std::string("loop_rate"), loop_rate_, 10.0);
  priv_nh_.param<double>(std::string("preempt_wait"), preempt_wait_, 15.0);
  priv_nh_.param<double>(std::string("connect_wait"), connect_wait_, 20.0);
  priv_nh_.param<int>(std::string("grabcut_iters"), grabcut_iters_, 5);

  ROS_INFO("grabcut3D segmentation initialized");
  segment_server_.start();

}

Grabcut3dSegmentation::~Grabcut3dSegmentation(void) {

}

bool Grabcut3dSegmentation::getPixelLabels(const sensor_msgs::Image& image, const sensor_msgs::Image& mask,
					   std::vector<shared_autonomy_msgs::Pixel> &foreground_pixels,
					   std::vector<shared_autonomy_msgs::Pixel> &background_pixels) {

  ROS_INFO("grabcut3d_segmentation in getPixelLabels");
  foreground_pixels.clear();
  background_pixels.clear();

  bool segment_preempted = false;

  // Package up segmentation goal for the HMI, and send it out
  shared_autonomy_msgs::EditPixelGoal label_goal;
  shared_autonomy_msgs::EditPixelResult label_result;
  label_goal.image = image;
  label_goal.mask = mask;
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
    ROS_INFO("%s quitting b/c ros::ok() false.", action_name_.c_str());
    segment_server_.setAborted();
    return false;
  }
}


bool Grabcut3dSegmentation::getBoundingBox(const sensor_msgs::Image& image, 
					   int *min_col, int *max_col, int *min_row, int *max_row) {

  ROS_INFO("grabcut3d_segmentation in getBoundingBox");
  bool segment_preempted = false;

  shared_autonomy_msgs::BoundingBoxGoal bb_goal;
  shared_autonomy_msgs::BoundingBoxResult bb_result;
  bb_goal.image = image;
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
      *min_col = bb_result.min_col.data;
      *max_col = bb_result.max_col.data;
      *min_row = bb_result.min_row.data;
      *max_row = bb_result.max_row.data;
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
    ROS_INFO("%s quitting b/c ros::ok() false.", action_name_.c_str());
    segment_server_.setAborted();
    return false;
  }
}

// fill mask in w/ bounds from bbox call
// TODO: try using this, and varying the rule for what it's filled as (probable/definite)
void Grabcut3dSegmentation::maskFromBB(cv::Mat &mask, int min_col, int max_col, int min_row, int max_row) {
  cv::Point p1 = cv::Point(min_col, min_row);
  cv::Point p2 = cv::Point(max_col, max_row);
  cv::rectangle(mask, p1, p2, 1, CV_FILLED);
}

/*
 * Helper function to convert ROS sensor images to openCV mat images
 * (Originally from object_segmentation_ui)
 * @param image_msg - ROS sensor image
 * @param image - opencv Mat image
 *
 */
bool Grabcut3dSegmentation::matFromImageMessage(const sensor_msgs::Image& image_msg, cv::Mat& image) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
  image = cv_ptr->image;
  return true;
}
/**
bool Grabcut3dSegmentation::imageMessageFromMat(sensor_msgs::Image& ros_image, const cv::Mat& mat_image) {
  cv_bridge::CvImage bridge_image;
  bridge_image.header.stamp = ros::Time::now();
  bridge_image.header.frame_id = segment_goal->image.header.frame_id;
  bridge_image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  bridge_image.image = mat_image;
  try {
    bridge_image.toImageMsg(ros_image);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
  return true;
}
**/

// fill mask in w/ result from calling grabcut_3d w/ bounding box
void Grabcut3dSegmentation::grabcutMaskFromBB(const sensor_msgs::Image& ros_image, 
					      cv::Mat &mask, int min_col, int max_col, int min_row, int max_row) {

  cv::Mat rgb_image;
  matFromImageMessage(ros_image, rgb_image);
  cv::Mat depth_image;
  // TODO: this causes the error message:
  // [ERROR] [1380575220.208802077]: cv_bridge exception: [32FC1] is not a color format. but [bgr8] is. The conversion does not make sense
  //matFromImageMessage(segment_goal->depth, depth_image);

  // we're initializing from rect, so fill it in w/ input bounds
  cv::Rect rect = cv::Rect(min_col, min_row, max_col-min_col, max_row-min_row); 

  //grabcut3d will build its own models
  cv::Mat bgd_model;
  cv::Mat fgd_model;
  //TODO: does this init as probable or forced for things outside of the rectangle?

  grabCut3D(rgb_image, rgb_image, mask, rect, bgd_model, fgd_model, grabcut_iters_, cv::GC_INIT_WITH_RECT);
}

// updates the input mask with new classification based on input pixel labels
// sets 
void Grabcut3dSegmentation::grabcutMaskFromPixels(const sensor_msgs::Image& ros_image, cv_bridge::CvImage &mask_bridge,
						  std::vector<shared_autonomy_msgs::Pixel> foreground_pixels,
						  std::vector<shared_autonomy_msgs::Pixel> background_pixels) {
  // TODO: make this a parameter
  int radius = 20;
  std::vector<shared_autonomy_msgs::Pixel>::iterator it;
  for(it = foreground_pixels.begin(); it != foreground_pixels.end(); it++) {
    cv::Point pp = cv::Point((*it).u, (*it).v);
    cv::circle(mask_bridge.image, pp, radius, cv::GC_FGD, CV_FILLED);
  }
  for(it = background_pixels.begin(); it != background_pixels.end(); it++) {
    cv::Point pp = cv::Point((*it).u, (*it).v);
    cv::circle(mask_bridge.image, pp, radius, cv::GC_BGD, CV_FILLED);
  }

  // we're initializing from rect, so fill it in w/ input bounds
  cv::Rect rect;

  //grabcut3d will build its own models
  cv::Mat bgd_model;
  cv::Mat fgd_model;

  cv::Mat rgb_image;
  matFromImageMessage(ros_image, rgb_image);
  grabCut3D(rgb_image, rgb_image, mask_bridge.image, rect, bgd_model, fgd_model, grabcut_iters_, cv::GC_INIT_WITH_MASK);

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

  // TODO: This needs better sub-functions... have them return bool for success, and return if not?
  // * check HMI server up
  // * obtain bounding_box
  // * bbox bounds to mask
  // * obtain labels/acceptance
  // * update mask w/ labels


  /** This compiles, not sure it's the right way to do it ..
  // we use bridge_image and bridge_mask locally; convert back to ROS when necessary
  // TODO: I'm struggling with when to use the Ptr or not...
  cv_bridge::CvImagePtr bridge_image_ptr;
  bridge_image_ptr = cv_bridge::toCvCopy(segment_goal->image);
  cv_bridge::CvImage bridge_image(bridge_image_ptr->header, bridge_image_ptr->encoding, bridge_image_ptr->image);

  // Set up mask that'll be used for the rest of the program to iterate on bg/fg
  cv::Mat mask = cv::Mat::zeros(segment_goal->image.height, segment_goal->image.width, CV_8UC1);
  ROS_INFO("created cv::Mat mask");
  cv_bridge::CvImage bridge_mask(bridge_image_ptr->header, "mono8", mask);
  **/


//TODO: I kind of dislike how all the helper functions are the ones that actually handle
// setting the actionlib server status, but it got too cumbersome to have it at the top level
void Grabcut3dSegmentation::segmentExecuteCB(const shared_autonomy_msgs::SegmentGoalConstPtr &segment_goal) {
  ROS_INFO("grabcut3d_segmentation::segmentExecuteCB called");

  // Check if HMI server is up; if not, we can't do segmentation
  bool hmi_server_connected = checkHMIConnected();
  if(!hmi_server_connected) {
    return;
  }

  // Get initial bounding box
  int min_col, max_col, min_row, max_row;
  bool bb_succeeded = getBoundingBox(segment_goal->image, &min_col, &max_col, &min_row, &max_row);
  if(!bb_succeeded) {
    return;
  }

  // initialize mask from rectangle or initial segmentation
  cv::Mat mask_mat = cv::Mat::zeros(segment_goal->image.height, segment_goal->image.width, CV_8UC1);
  cv_bridge::CvImage mask_bridge(segment_goal->image.header, "mono8", mask_mat);
  //maskFromBB(mask, min_col, max_col, min_row, max_row);
  grabcutMaskFromBB(segment_goal->image, mask_bridge.image, min_col, max_col, min_row, max_row);
  ROS_INFO("rectangle added to cv::Mat");

  // TODO: this is a mess. where do we use the ROS message types, and where do we use the cv?
  // I'm just keeping both versions around for the mask ... 
  sensor_msgs::Image mask_img;
  mask_bridge.toImageMsg(mask_img);

  std::vector<shared_autonomy_msgs::Pixel> foreground_pixels;
  std::vector<shared_autonomy_msgs::Pixel> background_pixels;
  shared_autonomy_msgs::Pixel mypixel;

  // TODO: checking every time if we have valid info before continuing is ugly. 
  // better alternative?
  // TODO: I hate how these functions take different data format and I have to 
  // convert every time; best to convert inside function and only have to do it once
  bool label_succeeded = getPixelLabels(segment_goal->image, mask_img, foreground_pixels, background_pixels);
  if(!label_succeeded) {
    return;
  }
  while (!(foreground_pixels.empty() and background_pixels.empty())) {
    ROS_INFO("looping in pixel feedback. sizes: %d (fgd), %d (bdg)", foreground_pixels.size(), background_pixels.size());

    grabcutMaskFromPixels(segment_goal->image, mask_bridge, foreground_pixels, background_pixels);
    mask_bridge.toImageMsg(mask_img);

    label_succeeded = getPixelLabels(segment_goal->image, mask_img, foreground_pixels, background_pixels);
    if (!label_succeeded) {
      return;
    }
  }

  // update image message mask
  mask_bridge.toImageMsg(mask_img);

  // return mask (work w/ openCV functions, then convert at the last minute?)
  segmentation_result_.mask = mask_img;
  segment_server_.setSucceeded(segmentation_result_);
  ROS_INFO("%s succeeded.", action_name_.c_str());  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "grabcut3d_segmentation_node");

  Grabcut3dSegmentation segmenter("segment_service", "bbox_service", "pixel_service");

  ros::spin();
  return 0;
}
