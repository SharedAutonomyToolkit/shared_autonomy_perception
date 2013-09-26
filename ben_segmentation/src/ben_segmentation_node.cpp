#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <shared_autonomy_msgs/SegmentAction.h>

class BenSegmentation {

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<shared_autonomy_msgs::SegmentAction> as_;
  std::string action_name_;
  shared_autonomy_msgs::SegmentFeedback feedback_;
  shared_autonomy_msgs::SegmentResult result_;

public:
  BenSegmentation(std::string name);
  ~BenSegmentation(void);
  void executeCB(const shared_autonomy_msgs::SegmentGoalConstPtr &goal);

};

BenSegmentation::BenSegmentation(std::string name)  : 
  as_(nh_, name, boost::bind(&BenSegmentation::executeCB, this, _1), false), 
  action_name_(name) 
{
  as_.start();
}

BenSegmentation::~BenSegmentation(void) {

}

void BenSegmentation::executeCB(const shared_autonomy_msgs::SegmentGoalConstPtr &goal) {
  // flag for whether we broke out of one of the loops thanks to preemption
  bool preempted = false;
  // how long to sleep while waiting for feedback / checking for preemption
  ros::Rate rr(1.0);

  feedback_.count = std_msgs::Int32();
  feedback_.count.data = 0;
  // TODO: this is where I'd make the first call to the HMI actionserver

  for(int ii = 1; ii < 10; ii++) {
    // check for/handle preemption
    if(as_.isPreemptRequested() or !ros::ok()) {
      ROS_INFO("%s preempted", action_name_.c_str());
      // TODO: pass preempt along to the HMI actionserver, wait for that ...
      as_.setPreempted();
      preempted = true;
      break;
    }

    // update feedback
    feedback_.count.data = feedback_.count.data + 1;
    as_.publishFeedback(feedback_);

    rr.sleep();
  }

  if(preempted) {
    return;
  }

  /**
  // TODO: do first segmentation based on that mask
  
  // TODO: make 2nd call to HMI actionserver!
  bool segmentation_accepted = false;
  while (!segmentation_accpted) {
    // TODO: actionlib service call to HMI server for feedback/acceptance
    for (int ii = 1; ii < 10; ii++) {

    }
    // TODO: update segmentation_accepted based on the feedback received
  }

  if(preempted) {
    return;
  }
  **/

  // finally, return the segmentation!
  result_.mask = sensor_msgs::Image();
  ROS_INFO("%s succeeded.", action_name_.c_str());
  as_.setSucceeded(result_); 

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ben_segmentation_node");
  BenSegmentation segmenter(ros::this_node::getName());
  ros::spin();
  return 0;
}
