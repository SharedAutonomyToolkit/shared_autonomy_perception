/*
 * user_grasp_pose_selection.h
 *
 *  Created on: 2010-08-03
 *      Author: Hao Dang
 */

#ifndef USER_GRASP_POSE_SELECTION_H_
#define USER_GRASP_POSE_SELECTION_H_

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <interactive_rviz_msgs/SelectionMessage.h>
#include <interactive_rviz_msgs/ControlMessage.h>
#include <tf/transform_broadcaster.h>

namespace user_selection_interface
{
class UserGraspPoseSelection
{
private:
	ros::NodeHandle nh_;
	bool is_ready_;
	bool selected_;
	/* This variable is tricky, publish TF once at the very beginning of the initialization
	 * does not make the virtual hand visualize well, so we need to keep publishing the
	 * TF, but still we need to know whether user has moved the gripper or not, once the
	 * user has moved the user, we need to handle the control to the user not in here
	 */
	bool user_moved_;
	geometry_msgs::TransformStamped pose_selected_;
	tf::TransformBroadcaster tf_broadcaster_;
	ros::Subscriber pose_control_sub_;
	ros::Subscriber pose_broadcast_sub_;

	// publishURDF is deprecated
	void publishURDF(std::string urdf_path);
	bool checkURDF();
	void clean();
	void publishDefaultTF();

	void selectionCallback(const rviz_communicator_msgs::SelectionMessageConstPtr& msg);
	void controlCallback(const rviz_communicator_msgs::ControlMessageConstPtr& msg);

public:
	UserGraspPoseSelection();

	void init();
	geometry_msgs::TransformStamped select();
};

} //user_selection_interface
#endif /* USER_GRASP_POSE_SELECTION_H_ */
