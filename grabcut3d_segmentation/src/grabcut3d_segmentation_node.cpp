#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <grabcut_3d/grabcut_3d.h>
#include <sensor_msgs/image_encodings.h>
#include <shared_autonomy_msgs/BoundingBoxAction.h>
#include <shared_autonomy_msgs/EditPixelAction.h>
#include <shared_autonomy_msgs/SegmentAction.h>

enum BBoxFinalState {
  SUCCEEDED,
  PREEMPTED,
  FAILED
};

/**


 **/

class Grabcut3dSegmentation {

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<shared_autonomy_msgs::SegmentAction> segment_server_;
  actionlib::SimpleActionClient<shared_autonomy_msgs::BoundingBoxAction> bb_client_;
  actionlib::SimpleActionClient<shared_autonomy_msgs::EditPixelAction> label_client_;
  std::string action_name_;
  shared_autonomy_msgs::SegmentResult segmentation_result_;

  BBoxFinalState getPixelLabels(const sensor_msgs::Image& image, const sensor_msgs::Image& mask);
  BBoxFinalState getBoundingBox(const shared_autonomy_msgs::SegmentGoalConstPtr &segment_goal,
				int &min_col, int &max_col, int &min_row, int &max_row);

  void maskFromBB(cv::Mat &mask, int min_col, int max_col, int min_row, int max_row);
  void grabcutMaskFromBB(const shared_autonomy_msgs::SegmentGoalConstPtr &segment_goal, cv::Mat &mask, int min_col, int max_col, int min_row, int max_row);
  bool matFromImageMessage(const sensor_msgs::Image& image_msg, cv::Mat& image);

public:
  Grabcut3dSegmentation(std::string segment_name, std::string bb_name, std::string label_name);
  ~Grabcut3dSegmentation(void);
  void segmentExecuteCB(const shared_autonomy_msgs::SegmentGoalConstPtr &goal);

};

Grabcut3dSegmentation::Grabcut3dSegmentation(std::string segment_name, std::string bb_name, std::string label_name)  : 
  segment_server_(nh_, segment_name, boost::bind(&Grabcut3dSegmentation::segmentExecuteCB, this, _1), false), 
  bb_client_(bb_name, true), 
  label_client_(label_name, true),
  action_name_(segment_name) 
{

  ROS_INFO("grabcut3d_segmentation initialized");
  segment_server_.start();

}

Grabcut3dSegmentation::~Grabcut3dSegmentation(void) {

}

BBoxFinalState Grabcut3dSegmentation::getPixelLabels(const sensor_msgs::Image& image, const sensor_msgs::Image& mask) {

  ROS_INFO("grabcut3d_segmentation in getPixeslLabels");
  // whether a preempt has been requested
  bool segment_preempted = false;
  BBoxFinalState label_state = FAILED;
  // Package up segmentation goal for the HMI, and send it out
  shared_autonomy_msgs::EditPixelGoal label_goal;
  // TODO: pass this in as parameter
  shared_autonomy_msgs::EditPixelResult label_result;
  label_goal.image = image;
  label_goal.mask = mask;
  label_client_.sendGoal(label_goal);

  ROS_INFO("grabcut3d_segmentation sent edit pixel goal");

  // wait for bounding box result OR preemption
  ros::Rate rr(10);
  while (!label_client_.getState().isDone() and ros::ok() and !segment_preempted) {
    segment_preempted = segment_server_.isPreemptRequested();
    rr.sleep();
  } 

  ROS_INFO("grabcut3d_segmentation broke out of loop waiting for pixel label result");

  if(label_client_.getState().isDone()) {
    if(label_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      // TODO: in future, this would be the only state that continues to the next
      // segmentation step; the rest would return a preempted/aborted state
      ROS_INFO("grabcut3d_segmentation's label client returned successfully");
      label_state = SUCCEEDED;
    }
    else { // preempted, aborted, rejected, etc. ...
      ROS_INFO("grabcut3d_segmentation edit pixel client returned UNsuccessfully");
      label_state = FAILED;
    }
  }
  else if (segment_preempted) {
    ROS_INFO("grabcut3d_segmentation - preempt requested");
    //request preempt 
    label_client_.cancelGoal();
    label_client_.waitForResult(ros::Duration(15.0));
    label_state = PREEMPTED;
  }
  else {
    // Only way to get here should be if !ros::ok(), in which case we also quit
    ROS_INFO("%s quitting b/c ros::ok() false.", action_name_.c_str());
    label_state = FAILED;
  }
  return label_state;
}


BBoxFinalState Grabcut3dSegmentation::getBoundingBox(const shared_autonomy_msgs::SegmentGoalConstPtr &segment_goal, 
					       int &min_col, int &max_col, int &min_row, int &max_row) {

  ROS_INFO("grabcut3d_segmentation in getBoundingBox");
  // whether a preempt has been requested
  bool segment_preempted = false;
  BBoxFinalState bb_state = FAILED;
  // Package up segmentation goal for the HMI, and send it out
  shared_autonomy_msgs::BoundingBoxGoal bb_goal;
  // TODO: pass this in as parameter
  shared_autonomy_msgs::BoundingBoxResult bb_result;
  bb_goal.image = segment_goal->image;
  bb_client_.sendGoal(bb_goal);

  ROS_INFO("grabcut3d_segmentation sent goal");

  // wait for bounding box result OR preemption
  ros::Rate rr(10);
  while (!bb_client_.getState().isDone() and ros::ok() and !segment_preempted) {
    segment_preempted = segment_server_.isPreemptRequested();
    rr.sleep();
  } 

  ROS_INFO("grabcut3d_segmentation broke out of loop waiting for bbox result");

  if(bb_client_.getState().isDone()) {
    if(bb_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      // TODO: in future, this would be the only state that continues to the next
      // segmentation step; the rest would return a preempted/aborted state
      ROS_INFO("grabcut3d_segmentation's bbox client returned successfully");
      bb_result = *bb_client_.getResult();
      min_col = bb_result.min_col.data;
      max_col = bb_result.max_col.data;
      min_row = bb_result.min_row.data;
      max_row = bb_result.max_row.data;
      ROS_INFO("coords are: %d, %d, %d, %d", min_col, max_col, min_row, max_row);
      bb_state = SUCCEEDED;
    }
    else { // preempted, aborted, rejected, etc. ...
      ROS_INFO("grabcut3d_segmentation bbox client returned UNsuccessfully");
      bb_state = FAILED;
    }
  }
  else if (segment_preempted) {
    ROS_INFO("grabcut3d_segmentation - preempt requested");
    //request preempt 
    bb_client_.cancelGoal();
    bb_client_.waitForResult(ros::Duration(15.0));
    bb_state = PREEMPTED;
  }
  else {
    // Only way to get here should be if !ros::ok(), in which case we also quit
    ROS_INFO("%s quitting b/c ros::ok() false.", action_name_.c_str());
    bb_state = FAILED;
  }
  return bb_state;
}

// fill mask in w/ bounds from bbox call
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
  // converts ROS images to OpenCV
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
  
  image = cv_ptr->image;
  return true;
}


// fill mask in w/ result from calling grabcut_3d w/ bounding box
void Grabcut3dSegmentation::grabcutMaskFromBB(const shared_autonomy_msgs::SegmentGoalConstPtr &segment_goal, cv::Mat &mask, int min_col, int max_col, int min_row, int max_row) {

  cv::Mat rgb_image;
  matFromImageMessage(segment_goal->image, rgb_image);
  cv::Mat depth_image;
  // TODO: this causes the error message:
  // [ERROR] [1380575220.208802077]: cv_bridge exception: [32FC1] is not a color format. but [bgr8] is. The conversion does not make sense
  //matFromImageMessage(segment_goal->depth, depth_image);

  // we're initializing from rect, so fill it in w/ input bounds
  cv::Rect rect = cv::Rect(min_col, min_row, max_col-min_col, max_row-min_row); 

  //grabcut3d will build its own models
  cv::Mat bgd_model;
  cv::Mat fgd_model;

  grabCut3D(rgb_image, rgb_image, mask, rect, bgd_model, fgd_model, 5, cv::GC_INIT_WITH_RECT);

  //cv::Point p1 = cv::Point(min_col, min_row);
  //cv::Point p2 = cv::Point(max_col, max_row);
  //cv::rectangle(mask, p1, p2, 1, CV_FILLED);
}


  // TODO: This needs some sub-functions...
  // * obtain bounding_box
  // * bbox bounds to mask
  // * obtain labels/acceptance
  // * update mask w/ labels
void Grabcut3dSegmentation::segmentExecuteCB(const shared_autonomy_msgs::SegmentGoalConstPtr &segment_goal) {
  ROS_INFO("grabcut3d_segmentation::segmentExecuteCB called");
  // Check if HMI server is up; if not, we can't do segmentation
  if(!bb_client_.isServerConnected()) {
    bool bb_ready = bb_client_.waitForServer(ros::Duration(15.0));
    if(!bb_ready) {
      segment_server_.setAborted();
      ROS_WARN("grabcut3dsegmentation could not execute segmentation - HMI bounding box not connected!");
      return;
    }
  }
  if(!label_client_.isServerConnected()) {
    bool label_ready = label_client_.waitForServer(ros::Duration(15.0));
    if(!label_ready) {
      segment_server_.setAborted();
      ROS_WARN("grabcut3dsegmentation could not execute segmentation - HMI pixel labeller not connected!");
      return;
    }
  }
  ROS_INFO("grabcut3d_segmentation got HMI server!");

  // TODO: I dislike how this function also uses segment_server_, but I think 
  // that it has to in order to handle the preemption stuff
  int min_col, max_col, min_row, max_row;
  BBoxFinalState bb_state = getBoundingBox(segment_goal, min_col, max_col, min_row, max_row);
  if(bb_state == FAILED) {
    segment_server_.setAborted();
    return;
  }
  else if (bb_state == PREEMPTED) {
    segment_server_.setPreempted();
    return;
  }
  else if (bb_state != SUCCEEDED) {
    ROS_INFO("invalid state returned from getBoundingBox!");
    return;
  }

  // Set up mask that'll be used for the rest of the program to iterate on bg/fg
  cv::Mat mask = cv::Mat::zeros(segment_goal->image.height, segment_goal->image.width, CV_8UC1);
  ROS_INFO("created cv::Mat mask");

  // initialize mask from rectangle or initial segmentation
  //maskFromBB(mask, min_col, max_col, min_row, max_row);
  grabcutMaskFromBB(segment_goal, mask, min_col, max_col, min_row, max_row);
  ROS_INFO("rectangle added to cv::Mat");

  // Display resulting mask 
  cv_bridge::CvImage mask_msg;
  mask_msg.header.stamp = ros::Time::now();
  mask_msg.header.frame_id = "base_link";
  mask_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  mask_msg.image = mask;
  sensor_msgs::Image mask_img;// = sensor_msgs::Image();
  mask_msg.toImageMsg(mask_img);

  BBoxFinalState label_state = getPixelLabels(segment_goal->image, mask_img);
  if(label_state == FAILED) {
    segment_server_.setAborted();
    return;
  }
  else if (label_state == PREEMPTED) {
    segment_server_.setPreempted();
    return;
  }
  else if (label_state != SUCCEEDED) {
    ROS_INFO("invalid state returned from getPixelLabels!");
    return;
  }

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ros::Time::now();
  out_msg.header.frame_id = "base_link";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  out_msg.image = mask;
  sensor_msgs::Image tmp_img;// = sensor_msgs::Image();
  out_msg.toImageMsg(tmp_img);

  ROS_INFO("cv::Mat converted to sensor_msgs::Image");

  // return mask (work w/ openCV functions, then convert at the last minute?)
  segmentation_result_.mask = tmp_img;
  segment_server_.setSucceeded(segmentation_result_);
  ROS_INFO("%s succeeded.", action_name_.c_str());  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "grabcut3d_segmentation_node");

  Grabcut3dSegmentation segmenter("segment_service", "bbox_service", "pixel_service");

  ros::spin();
  return 0;
}
