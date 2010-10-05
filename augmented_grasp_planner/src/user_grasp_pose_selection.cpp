/*
 * user_grasp_pose_selection.cpp
 *
 *  Created on: 2010-08-03
 *      Author: Hao Dang
 */

#include "user_selection_interface/user_grasp_pose_selection.h"
#include <ros/callback_queue.h>
#include <fstream>

//the name for the urdf, it is a private member of this node
#define URDF_NAME "virtual_gripper_urdf"
//#define URDF_PATH "/home/dah1pal/ros/overlays/bosch-ros-pkg/hao/pr2-gripper/gripper_without_collision.urdf"

namespace user_selection_interface
{

UserGraspPoseSelection::UserGraspPoseSelection() :
		nh_(""),
		is_ready_(false),
		selected_(false)
{
	//wait for user feedback
	pose_broadcast_sub_ = nh_.subscribe("sTopic", 10, &UserGraspPoseSelection::selectionCallback, this);
	pose_control_sub_ = nh_.subscribe("cTopic", 10, &UserGraspPoseSelection::controlCallback, this);
}

void
UserGraspPoseSelection::init()
{
	//publishURDF(URDF_PATH);
	if(checkURDF())
		ROS_INFO("virtual gripper urdf loaded successfully");
	else
	{
		ROS_ERROR("virtual gripper urdf not found");
		return;
	}

	publishDefaultTF();
	is_ready_ = true;
	user_moved_ = false;
}

geometry_msgs::TransformStamped
UserGraspPoseSelection::select()
{
	if(!is_ready_)
	{
		ROS_ERROR("Not ready for selection, load gripper urdf first");
		return pose_selected_;
	}

	ROS_INFO("waiting for user selection");
	while(!selected_ && ros::ok())
	{
		publishDefaultTF();
		//ROS_INFO("waiting for user selection");
		ros::getGlobalCallbackQueue()->callAvailable();
		ros::Duration(0.1).sleep();
	}

	clean();
	return pose_selected_;
}

void
UserGraspPoseSelection::selectionCallback(const rviz_communicator_msgs::SelectionMessageConstPtr& msg)
{
	//	ROS_ERROR("IN SELECTION CALLBACK");
	pose_selected_ = msg->transform;
	selected_ = true;
}

void
UserGraspPoseSelection::controlCallback(const rviz_communicator_msgs::ControlMessageConstPtr& msg)
{
	//	ROS_ERROR("IN CONTROL CALLBACK");
	//	tf::StampedTransform st;
	//	transformStampedMsgToTF(msg->transform, st);
	//	tf_broadcaster_.sendTransform(st);
	user_moved_ = true;
}

void
UserGraspPoseSelection::clean()
{
	//make the virtual hand disappear
	//	nh_.deleteParam(URDF_NAME);

	//this is another hack, which i hate so much
	//move the gripper out of our view
	tf::Transform transform = tf::Transform::getIdentity();
	transform.setOrigin(tf::Vector3(99, 99, 99));
	tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "virtual_gripper_origin", "virtual_gripper_palm_link"));
}

void
UserGraspPoseSelection::publishDefaultTF()
{
	tf::Transform transform = tf::Transform::getIdentity();
//		ROS_ERROR("SENT");
	//between robot and virtual_gripper_origin
	tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "virtual_gripper_origin"));

	//this is a hack, not an elegant way, sorry for that
	if(!user_moved_)
		tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "virtual_gripper_origin", "virtual_gripper_palm_link"));

	transform.setOrigin(tf::Vector3(0.077, 0.010, 0.000));
	tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "virtual_gripper_palm_link", "virtual_gripper_l_finger_link"));
	transform.setOrigin(tf::Vector3(0.091, 0.005, 0.000));
	tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "virtual_gripper_l_finger_link", "virtual_gripper_l_finger_tip_link"));
	transform.setOrigin(tf::Vector3(0.077, -0.010, 0.000));
	tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "virtual_gripper_palm_link", "virtual_gripper_r_finger_link"));
	transform.setOrigin(tf::Vector3(0.091, -0.005, 0.000));
	tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "virtual_gripper_r_finger_link", "virtual_gripper_r_finger_tip_link"));

}

bool
UserGraspPoseSelection::checkURDF()
{
	return nh_.hasParam(URDF_NAME);
}

void
UserGraspPoseSelection::publishURDF(std::string urdf_path)
{
	int length;
	char * buffer;
	std::ifstream is;

	is.open (urdf_path.c_str(), std::ios::binary );

	if(!is.is_open())
	{
		ROS_ERROR("URDF NOT OPENED");
		return;
	}
	ROS_INFO("READY TO PUBLISH URDF");

	// get length of file:
	is.seekg (0, std::ios::end);
	length = is.tellg();
	is.seekg (0, std::ios::beg);

	// allocate memory:
	buffer = new char [length];

	// read data as a block:
	is.read (buffer,length);
	is.close();

	ROS_INFO_STREAM("urdf: " << buffer);

	nh_.setParam(URDF_NAME, buffer);

	delete[] buffer;
}

} //namespace
