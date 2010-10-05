/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "augmented_grasp_planner/grasp_selection_marker_publisher.h"

namespace user_selection_interface {

static const std::string MARKERS_OUT_NAME = "user_selection";
static const std::string MARKERS_NAMESPACE = "grasp_selection_markers";

GraspSelectionMarkerPublisher::GraspSelectionMarkerPublisher() :
  priv_nh_("~")
{
  marker_pub_ = priv_nh_.advertise<visualization_msgs::Marker>(MARKERS_OUT_NAME, 100);
}

std::string GraspSelectionMarkerPublisher::getMarkerTopic() {
  return priv_nh_.resolveName(MARKERS_OUT_NAME);
}

void GraspSelectionMarkerPublisher::clearAllMarkers()
{
  for (size_t g=0; g<grasp_markers_.size(); g++) 
  {
    grasp_markers_[g].action = visualization_msgs::Marker::DELETE;
    marker_pub_.publish(grasp_markers_[g]);
  }
  grasp_markers_.clear();
}

unsigned int GraspSelectionMarkerPublisher::addGraspMarker(const geometry_msgs::PoseStamped &marker_pose)
{
// ROS_INFO("IN addGraspMarker,press any key to continue");
// getchar();

  visualization_msgs::Marker marker;
  marker.pose = marker_pose.pose;
  marker.header.frame_id = marker_pose.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = MARKERS_NAMESPACE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 0.1235;
  marker.scale.y = 0.06;
  marker.scale.z = 0.06;

  /*
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.points.resize(2);
  marker.points[0].x = marker.points[0].y = marker.points[0].z = 0.0;
  marker.points[1].x = 0.1235;
  marker.points[1].y = marker.points[1].z = 0.0;
  marker.scale.x = 0.005;
  */

  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.id = grasp_markers_.size();

  grasp_markers_.push_back(marker);
  marker_pub_.publish(marker);
  //  std::cout << "IN addGraspMarker, about to publish this marker, select topic in RVIZ in case"
  //		  << marker.id << ", press any key to continue\n";
  //  getchar();
  //  std::cout << "found " << marker_pub_.getNumSubscribers() << " subscribers\n";
  return marker.id;
}

void GraspSelectionMarkerPublisher::colorGraspMarker(unsigned int marker_id, float r, float g, float b)
{
  if (marker_id >= grasp_markers_.size()) 
  {
    ROS_WARN("Failed to change color of grasp marker %d", marker_id);
    return;
  }
  grasp_markers_[marker_id].color.r = r;
  grasp_markers_[marker_id].color.g = g;
  grasp_markers_[marker_id].color.b = b;
  grasp_markers_[marker_id].header.stamp = ros::Time::now();
  marker_pub_.publish(grasp_markers_[marker_id]);
}

void GraspSelectionMarkerPublisher::setMarkerPose(unsigned int marker_id, const geometry_msgs::PoseStamped &marker_pose)
{
  if (marker_id >= grasp_markers_.size()) 
  {
    ROS_WARN("Failed to change pose of grasp marker %d", marker_id);
    return;
  }
  grasp_markers_[marker_id].pose = marker_pose.pose;
  grasp_markers_[marker_id].header.frame_id = marker_pose.header.frame_id;
  grasp_markers_[marker_id].header.stamp = ros::Time::now();
  marker_pub_.publish(grasp_markers_[marker_id]);
}

} //namespace grasp_execution
