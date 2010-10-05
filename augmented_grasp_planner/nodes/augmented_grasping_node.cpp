/*
 * augmented_grasping_node.cpp
 *
 *  Created on: Oct 4, 2010
 *      Author: duhadway
 */

#include <vector>

#include <ros/ros.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <object_manipulator/tools/grasp_marker_publisher.h>

#include "augmented_grasp_planner/user_grasp_selection.h"
#include "augmented_grasp_planner/grasp_selection_marker_publisher.h"

using namespace object_manipulation_msgs;
using namespace std;
using namespace ros;

geometry_msgs::PoseStamped getTargetPose(const object_manipulation_msgs::GraspableObject &target)
{
  switch(target.type)
  {
  case object_manipulation_msgs::GraspableObject::DATABASE_MODEL:
    return target.model_pose.pose;
  case object_manipulation_msgs::GraspableObject::POINT_CLUSTER:
  {
    geometry_msgs::PoseStamped pose;
    pose.header = target.cluster.header;
    pose.pose.position.x = pose.pose.position.y = pose.pose.position.z = 0.0;
    pose.pose.orientation.x = pose.pose.orientation.y = pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose;
  }
  default:
    ROS_ERROR("Unknown type of graspable object in getTargetPose");
    return target.model_pose.pose;
  }
}


class AugmentedGraspPlanner {
private:
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  ros::NodeHandle public_nh_;

  //! Server for the get grasps service
  ros::ServiceServer grasp_planning_srv_;

  //! client for the underlying automated grasp planner
  ros::ServiceClient automated_planning_srv_;

  std::string grasp_planning_srv_name_;
  std::string automated_planning_srv_name_;

  object_manipulator::GraspMarkerPublisher marker_pub_;
public:

  AugmentedGraspPlanner() : priv_nh_("~"), public_nh_("") {
    priv_nh_.param<std::string>("planner_service_name", grasp_planning_srv_name_, "augmented_database_grasp_planning");
    priv_nh_.param<std::string>("automated_planner_service_name", automated_planning_srv_name_, "database_grasp_planning");

    grasp_planning_srv_ = priv_nh_.advertiseService(grasp_planning_srv_name_, &AugmentedGraspPlanner::graspPlanningCB, this);
  }

  bool graspPlanningCB(GraspPlanning::Request &request, GraspPlanning::Response &response)
  {
    ros::ServiceClient client = public_nh_.serviceClient<object_manipulation_msgs::GraspPlanning>(automated_planning_srv_name_);

    object_manipulation_msgs::GraspPlanning srv;
    srv.request = request;
    if (!client.call(srv))
    {
      ROS_ERROR("Augmented grasp planner failed to call automated planner at %s", automated_planning_srv_name_.c_str());
      response = srv.response;
      return false;
    }

    if (srv.response.grasps.size() > 1)
      rankGrasps(srv.response.grasps, getTargetPose(request.target));

    response = srv.response;
    return true;
  }

  void rankGrasps(std::vector<object_manipulation_msgs::Grasp>& grasps, geometry_msgs::PoseStamped target_pose)
  {
    //marker publisher
    object_manipulator::GraspMarkerPublisher marker_pub_;

    std::vector<geometry_msgs::Pose> poses;
    size_t count = grasps.size();
    for(size_t i = 0; i < count; ++i)
    {
      poses.push_back(grasps[i].grasp_pose);
    }

    user_selection_interface::UserGraspSelection marker_selector;
    marker_selector.init(target_pose, poses);
    int selected = marker_selector.select();

    if (selected > 0) {
      object_manipulation_msgs::Grasp tmp = grasps[0];
      grasps[0] = grasps[selected];
      grasps[selected] = tmp;
    }
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "augmented_grasp_planner_node");
  AugmentedGraspPlanner planner;
  ros::spin();
  return 0;
}
