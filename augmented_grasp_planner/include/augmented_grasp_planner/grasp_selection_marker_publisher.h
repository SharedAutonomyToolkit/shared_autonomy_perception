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
#ifndef _GRASP_SELECTION_MARKER_PUBLISHER_H_
#define _GRASP_SELECTION_MARKER_PUBLISHER_H_

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseStamped.h>

namespace user_selection_interface {

//! Publishes and keeps track of debug grasp markers
class GraspSelectionMarkerPublisher
{
 private:
  //! Private node handle
  ros::NodeHandle priv_nh_;
  
  //! Publisher for debug markers
  ros::Publisher marker_pub_;
  
  //! The current set of visualization markers 
  std::vector<visualization_msgs::Marker> grasp_markers_;

 public:
  //! Advertises the marker publishing topic
  GraspSelectionMarkerPublisher();

  //! Removes and deletes all currently displayed markers
  void clearAllMarkers();

  //! Adds and publishes a marker with default color (blue). Returns the id of the new marker.
  unsigned int addGraspMarker(const geometry_msgs::PoseStamped &marker_pose);

  //! Sets the color of the grasp marker with the given id
  void colorGraspMarker(unsigned int marker_id, float r, float g, float b);

  //! Sets the pose of the marker with the given id
  void setMarkerPose(unsigned int marker_id, const geometry_msgs::PoseStamped &marker_pose);

  std::string getMarkerTopic();

};

} //namespace user_selection_interface

#endif
