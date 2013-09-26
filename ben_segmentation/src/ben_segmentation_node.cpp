#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <shared_autonomy_msgs/BoundingBoxAction.h>
#include <shared_autonomy_msgs/SegmentAction.h>

class BenSegmentation {

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<shared_autonomy_msgs::SegmentAction> segment_server_;
  actionlib::SimpleActionClient<shared_autonomy_msgs::BoundingBoxAction> bb_client_;
  std::string action_name_;
  shared_autonomy_msgs::SegmentFeedback feedback_;
  shared_autonomy_msgs::SegmentResult segmentation_result_;
  bool bb_done_; 

public:
  BenSegmentation(std::string segment_name, std::string bb_name);
  ~BenSegmentation(void);
  void segmentExecuteCB(const shared_autonomy_msgs::SegmentGoalConstPtr &goal);
  void bbDoneCB(const actionlib::SimpleClientGoalState& state, const shared_autonomy_msgs::BoundingBoxResultConstPtr& result);
  //  void bbDoneCB(const actionlib::SimpleClientGoalState& state, const shared_autonomy_msgs::BoundingBoxResult& result);

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

void BenSegmentation::bbDoneCB(const actionlib::SimpleClientGoalState& state, const shared_autonomy_msgs::BoundingBoxResultConstPtr& result) {
  ROS_INFO("bbDoneCB called!");
  bb_done_ = true;
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

  //At this point, goal is NOT done; will be reset by done_cb
  bb_done_ = false;
  // whether a preempt has been requested
  bool segment_preempted = false;

  // Package up segmentation goal for the HMI, and send it out
  shared_autonomy_msgs::BoundingBoxGoal bb_goal;
  shared_autonomy_msgs::BoundingBoxResult bb_result;
  bb_goal.image = segment_goal->image;
  bb_client_.sendGoal(bb_goal, boost::bind(&BenSegmentation::bbDoneCB, this, _1, _2));
  //bb_client_.sendGoal(bb_goal, boost::bind(&BenSegmentation::bbDoneCB, this, _1, _2));

  ROS_INFO("ben_segmentation sent goal");

  //feedback_.count = std_msgs::Int32();
  //feedback_.count.data = 0;

  // wait for bounding box result OR preemption
  ros::Rate rr(0.1);
  while (!bb_done_ and ros::ok() and !segment_preempted) {
    // TODO: do I want to publish any feedback here?
    //feedback_.count.data = feedback_.count.data + 1;
    //segment_server_.publishFeedback(feedback_);

    segment_preempted = segment_server_.isPreemptRequested();
    rr.sleep();
  } 

  ROS_INFO("ben_segmentation broke out of loop waiting for bbox result");

  if(bb_done_) {
    if(bb_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      // TODO: in future, this would be the only state that continues to the next
      // segmentation step; the rest would return a preempted/aborted state
      ROS_INFO("ben_segmentation's bbox client returned successfully");
      bb_result = *bb_client_.getResult();
      ROS_INFO("coords are: %d, %d, %d, %d", 
	       bb_result.min_col.data, bb_result.max_col.data,
	       bb_result.min_row.data, bb_result.max_row.data);
    }
    else { // preempted, aborted, rejected, etc. ...
      ROS_INFO("ben_segmentation bbox client returned UNsuccessfully");
      segment_server_.setAborted();
      return;
    }
  }
  else if (segment_preempted) {
    ROS_INFO("ben_segmentation - preempt requested");
    //request preempt 
    bb_client_.cancelGoal();
    bb_client_.waitForResult(ros::Duration(15.0));
    segment_server_.setPreempted();
    return;
  }
  else {
    // Only way to get here should be if !ros::ok(), in which case we also quit
    ROS_INFO("%s quitting b/c ros::ok() false.", action_name_.c_str());
    return;
  }

  // Only way to be here is if the bbox succeeded, and the segmentation result was
  // published; in future, will continue work w/ the mask before returning / setting result
  segmentation_result_.mask = sensor_msgs::Image();
  ROS_INFO("%s succeeded.", action_name_.c_str());
  segment_server_.setSucceeded(segmentation_result_);
  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ben_segmentation_node");
  BenSegmentation segmenter(ros::this_node::getName(), "/get_bounding_box");
  ros::spin();
  return 0;
}
