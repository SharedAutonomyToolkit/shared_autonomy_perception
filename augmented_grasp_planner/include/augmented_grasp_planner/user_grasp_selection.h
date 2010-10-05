/*
 * user_grasp_selection.h
 *
 * Given a series of markers, this will visualize the markers in
 * RVIZ and return back the selection of the user's
 *
 *  Created on: 2010-07-09
 *      Author: Hao Dang
 */

#ifndef USER_GRASP_SELECTION_H_
#define USER_GRASP_SELECTION_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_rviz_msgs/SelectionMessage.h>
#include <tf/transform_listener.h>

#include "augmented_grasp_planner/grasp_selection_marker_publisher.h"


namespace user_selection_interface
{

class UserGraspSelection
{
private:
	// Private node handle
	ros::NodeHandle nh_;

	// The current set of visualization markers
	std::vector<visualization_msgs::Marker> markers_;

	std::vector<geometry_msgs::Pose> poses_;

	geometry_msgs::PoseStamped target_pose_;

	GraspSelectionMarkerPublisher marker_publisher_;

	ros::Subscriber marker_selection_sub_;

	// If this selector is ready to use
	bool is_ready_;

	// Has the user selected one marker
	bool selected_;

	// which one has been chosen
	size_t marker_selected_;

	std::vector<unsigned int> marker_ID_;

	void publish(size_t i);

	void clean();

	geometry_msgs::Pose transformToModelFrame(const geometry_msgs::Pose &in_pose);

	void selectionCallback(const interactive_rviz_msgs::SelectionMessageConstPtr& msg);

public:

	UserGraspSelection();

	/*
	 *  target_pose specifies the pose of the object to be grasped
	 *  poses are the poses of the grasps, used for marker pose
	 */
	void init(geometry_msgs::PoseStamped target_pose, std::vector<geometry_msgs::Pose> poses);

	//main function for user selection
	int select();

};

} // user_selection_interface

#endif /* USER_GRASP_SELECTION_H_ */
