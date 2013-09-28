#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <shared_autonomy_msgs/BoundingBoxAction.h>
#include <shared_autonomy_msgs/SegmentAction.h>

enum BBoxFinalState {
  SUCCEEDED,
  PREEMPTED,
  FAILED
};

class BenSegmentation {

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<shared_autonomy_msgs::SegmentAction> segment_server_;
  actionlib::SimpleActionClient<shared_autonomy_msgs::BoundingBoxAction> bb_client_;
  std::string action_name_;
  shared_autonomy_msgs::SegmentFeedback feedback_;
  shared_autonomy_msgs::SegmentResult segmentation_result_;

  BBoxFinalState getBoundingBox(const shared_autonomy_msgs::SegmentGoalConstPtr &segment_goal,
				int &min_col, int &max_col, int &min_row, int &max_row);
public:
  BenSegmentation(std::string segment_name, std::string bb_name);
  ~BenSegmentation(void);
  void segmentExecuteCB(const shared_autonomy_msgs::SegmentGoalConstPtr &goal);

};

BenSegmentation::BenSegmentation(std::string segment_name, std::string bb_name)  : 
  segment_server_(nh_, segment_name, boost::bind(&BenSegmentation::segmentExecuteCB, this, _1), false), 
  bb_client_(bb_name, true), 
  action_name_(segment_name) 
{
  segment_server_.start();

}

BenSegmentation::~BenSegmentation(void) {

}

BBoxFinalState BenSegmentation::getBoundingBox(const shared_autonomy_msgs::SegmentGoalConstPtr &segment_goal, 
					       int &min_col, int &max_col, int &min_row, int &max_row) {

  // whether a preempt has been requested
  bool segment_preempted = false;
  BBoxFinalState bb_state = FAILED;
  // Package up segmentation goal for the HMI, and send it out
  shared_autonomy_msgs::BoundingBoxGoal bb_goal;
  // TODO: pass this in as parameter
  shared_autonomy_msgs::BoundingBoxResult bb_result;
  bb_goal.image = segment_goal->image;
  bb_client_.sendGoal(bb_goal);

  ROS_INFO("ben_segmentation sent goal");

  //feedback_.count = std_msgs::Int32();
  //feedback_.count.data = 0;

  // wait for bounding box result OR preemption
  ros::Rate rr(10);
  // it looks like there IS an .isDone():
  // http://docs.ros.org/hydro/api/actionlib/html/classactionlib_1_1SimpleClientGoalState.html
  while (!bb_client_.getState().isDone() and ros::ok() and !segment_preempted) {
    // TODO: do I want to publish any feedback here?
    //feedback_.count.data = feedback_.count.data + 1;
    //segment_server_.publishFeedback(feedback_);

    segment_preempted = segment_server_.isPreemptRequested();
    rr.sleep();
  } 

  ROS_INFO("ben_segmentation broke out of loop waiting for bbox result");

  if(bb_client_.getState().isDone()) {
    if(bb_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      // TODO: in future, this would be the only state that continues to the next
      // segmentation step; the rest would return a preempted/aborted state
      ROS_INFO("ben_segmentation's bbox client returned successfully");
      bb_result = *bb_client_.getResult();
      min_col = bb_result.min_col.data;
      max_col = bb_result.max_col.data;
      min_row = bb_result.min_row.data;
      max_row = bb_result.max_row.data;
      ROS_INFO("coords are: %d, %d, %d, %d", min_col, max_col, min_row, max_row);
      bb_state = SUCCEEDED;
    }
    else { // preempted, aborted, rejected, etc. ...
      ROS_INFO("ben_segmentation bbox client returned UNsuccessfully");
      bb_state = FAILED;
    }
  }
  else if (segment_preempted) {
    ROS_INFO("ben_segmentation - preempt requested");
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

  // TODO: This needs some sub-functions...
  // * obtain bounding_box
  // * bbox bounds to mask
  // * obtain labels/acceptance
  // * update mask w/ labels
void BenSegmentation::segmentExecuteCB(const shared_autonomy_msgs::SegmentGoalConstPtr &segment_goal) {

  // Check if HMI server is up; if not, we can't do segmentation
  bool hmi_ready = bb_client_.waitForServer(ros::Duration(15.0));
  if(!hmi_ready) {
    segment_server_.setAborted();
    return;
  }
  ROS_INFO("ben_segmentation got HMI server!");

  // Get the bounding box
  // I dislike how this function also uses segment_server_, but I think 
  // that it has to in order to handle the preemption stuff
  int min_col, max_col, min_row, max_row;
  BBoxFinalState bb_state = getBoundingBox(segment_goal, min_col, max_col, min_row, max_row);
  
  ROS_INFO("getBoundingBox returned");

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

  // Convert bounding box to mask
  cv::Mat mask = cv::Mat::zeros(segment_goal->image.height, segment_goal->image.width, CV_8UC1);
  
  ROS_INFO("created cv::Mat mask");

  // fill mask in w/ bounds from bbox call
  cv::Point p1 = cv::Point(min_col, min_row);
  cv::Point p2 = cv::Point(max_col, max_row);
  cv::rectangle(mask, p1, p2, 1, CV_FILLED);

  ROS_INFO("rectangle added to cv::Mat");

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
  ros::init(argc, argv, "ben_segmentation_node");
  BenSegmentation segmenter(ros::this_node::getName(), "/get_bounding_box");
  ros::spin();
  return 0;
}
