/*
 * marker_selector.cpp
 *
 *  Created on: 2010-07-09
 *      Author: Hao Dang
 */

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "augmented_grasp_planner/user_grasp_selection.h"
#include "augmented_grasp_planner/grasp_selection_marker_publisher.h"

namespace user_selection_interface
{

UserGraspSelection::UserGraspSelection() :
						is_ready_(false), selected_(false), marker_selected_(-1)
{
	markers_.clear();
	marker_ID_.clear();
	//wait for user feedback
	marker_selection_sub_ = nh_.subscribe("sTopic", 10, &UserGraspSelection::selectionCallback, this);
}

void UserGraspSelection::init(geometry_msgs::PoseStamped target_pose, std::vector<geometry_msgs::Pose> poses)
{
	target_pose_ = target_pose;
	poses_ = poses;
	is_ready_ = true;
}

int UserGraspSelection::select()
{
	if(!is_ready_)
	{
		ROS_ERROR("User Grasp Selector is not ready to go, initialize it first\n");
		return -1;
	}
//	std::cout << "About to publish markers, select the topic and press any key to continue\n";
//	getchar();

	//visualize the markers
	for(size_t i = 0; i < poses_.size(); ++i)
	{
		publish(i);
	}

	ROS_INFO("Waiting for user selection, grasp markers published on %s", marker_publisher_.getMarkerTopic().c_str());
	ros::Time last = ros::Time::now();
	ros::Rate r (20);
	while(!selected_ && ros::ok())
	{
		ros::spinOnce();

		// republish markers every 5 seconds
		ros::Time n = ros::Time::now();
		if (( n.toSec() - last.toSec() ) > 5.0) {
		  ROS_INFO("Still waiting for user selection, grasp markers published on %s", marker_publisher_.getMarkerTopic().c_str());
		  last = n;
		  clean();
		  for(size_t i = 0; i < poses_.size(); ++i)
		  {
		    publish(i);
		  }
		}
		r.sleep();
	}

	clean();
	return marker_selected_;
}

void UserGraspSelection::publish(size_t i)
{
	geometry_msgs::PoseStamped marker_pose;
	marker_pose.pose = transformToModelFrame(poses_[i]);
	marker_pose.header.frame_id = target_pose_.header.frame_id;
//	std::cout << "frame_id " << marker_pose.header.frame_id << std::endl;
	marker_ID_.push_back(marker_publisher_.addGraspMarker(marker_pose));
}

void UserGraspSelection::selectionCallback(const interactive_rviz_msgs::SelectionMessageConstPtr& msg)
{
	if(msg->type!=interactive_rviz_msgs::SelectionMessage::MARKER_SELECTION)
		return;
	marker_selected_ = msg->markerID;
	std::cout << "selected grasp ID " << marker_selected_ << "\n";
	selected_ = true;
}

void UserGraspSelection::clean()
{
	marker_publisher_.clearAllMarkers();
	is_ready_ = false;
}

/* For debug, this is also used to modify the transform in some grasp-sepcific way, such as
 * moving it "back" (along negative X) some more.
 */
geometry_msgs::Pose UserGraspSelection::transformToModelFrame(const geometry_msgs::Pose &in_pose)
{
	tf::Transform model_trans;
	tf::poseMsgToTF(target_pose_.pose, model_trans);
	tf::Transform in_trans;
	tf::poseMsgToTF(in_pose, in_trans);
	in_trans = model_trans * in_trans;
	//debug: move back some more

	tf::Transform move_back_trans;
	move_back_trans.setIdentity();
	move_back_trans.setOrigin( btVector3(-0.001,0,0) );
	in_trans = in_trans * move_back_trans;

	geometry_msgs::Pose out_pose;
	tf::poseTFToMsg(in_trans, out_pose);
	return out_pose;
}

}// user_selection_interface
