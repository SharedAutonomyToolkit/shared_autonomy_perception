/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Robert Bosch LLC.
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
 *   * Neither the name of the Robert Bosch nor the names of its
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
 *
 *********************************************************************/

#include <string>
#include <vector>
#include <ros/ros.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometric_shapes_msgs/Shape.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <household_objects_database_msgs/GetModelMesh.h>
#include <household_objects_database_msgs/GetModelDescription.h>
#include <tabletop_object_detector/model_fitter.h>
#include <tabletop_object_detector/iterative_distance_fitter.h>
#include <tabletop_object_detector/exhaustive_fit_detector.h>
#include <tabletop_object_detector/TabletopDetectionResult.h>

#include "augmented_object_detector/DatabaseModelComplete.h"
#include "augmented_object_detector/DetectObject.h"

using std::string;
using std::vector;
namespace tod = tabletop_object_detector;
namespace hodm = household_objects_database_msgs;

class ObjectDetector {
public:
  ObjectDetector(ros::NodeHandle&);
  bool serviceCallback(augmented_object_detector::DetectObject::Request&,
                       augmented_object_detector::DetectObject::Response&);

private:
  static const int MAX_RESULTS = 10;
  ros::NodeHandle nh_;
  ros::NodeHandle lnh_;
  tod::ExhaustiveFitDetector<
    tod::IterativeTranslationFitter> detector_;
  ros::ServiceServer object_detection_srv_;
  ros::ServiceClient get_model_mesh_srv_;
  ros::ServiceClient get_model_desc_srv_;
  string model_set_;

};

bool
ObjectDetector::serviceCallback(augmented_object_detector::DetectObject::Request& req,
                                augmented_object_detector::DetectObject::Response& rsp)
{
  ROS_DEBUG("ObjectDetector: serviceCallback called (%i results requested)",
          req.num_results);
  if (req.num_results < 1 || req.num_results > MAX_RESULTS)
  {
    ROS_ERROR("ObjectDetector: invalid number of results (%d)", 
        req.num_results);
    return false;
  }

  // Find specified number of models that best fit the incoming point cloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg<pcl::PointXYZRGB>(req.cloud, cloud);
  vector<tod::ModelFitInfo> model_fits = 
    detector_.fitBestModels<pcl::PointCloud<pcl::PointXYZRGB> > (cloud,req.num_results);

  for (int i = 0; i < req.num_results; i++) {
    augmented_object_detector::DatabaseModelComplete model_msg;
    model_msg.model_id    = model_fits[i].getModelId();
    model_msg.score       = model_fits[i].getScore();
    model_msg.pose.pose   = model_fits[i].getPose(); 
    model_msg.pose.header = cloud.header;

    // Get description
    hodm::GetModelDescription get_desc;
    get_desc.request.model_id = model_msg.model_id;
    if ( !get_model_desc_srv_.call(get_desc) || 
          get_desc.response.return_code.code != get_desc.response.return_code.SUCCESS )
    {
      ROS_ERROR("ObjectDetector: GetModelDescription service call failed");
      rsp.status.code = get_desc.response.return_code.code;
      return false;
    }
    model_msg.name = get_desc.response.name;

    ROS_DEBUG("ObjectDetector: fit model %i (%s) with score %f", 
       model_msg.model_id, model_msg.name.c_str(), model_msg.score);

    // Get mesh
    hodm::GetModelMesh get_mesh;
    get_mesh.request.model_id = model_msg.model_id;
    if ( !get_model_mesh_srv_.call(get_mesh) || 
          get_mesh.response.return_code.code != get_mesh.response.return_code.SUCCESS )
    {
      ROS_ERROR("ObjectDetector: GetModelMesh service call failed");
      rsp.status.code = get_mesh.response.return_code.code;
      return false;
    }
    else
      model_msg.mesh = get_mesh.response.mesh;

    rsp.model_list.push_back(model_msg);
  }
  rsp.status.code = rsp.status.SUCCESS;
  return true;
}


ObjectDetector::ObjectDetector(ros::NodeHandle& _nh) : nh_(_nh) , lnh_("~")
{
  // Wait for model mesh service
  string get_model_mesh_srv_name;
  lnh_.param("get_model_mesh_srv", get_model_mesh_srv_name, 
      string("get_model_mesh_srv"));
  ROS_INFO("Waiting for %s service", get_model_mesh_srv_name.c_str());
  while ( !ros::service::waitForService(get_model_mesh_srv_name)) 
    ;
  get_model_mesh_srv_ = nh_.serviceClient<hodm::GetModelMesh> 
                              (get_model_mesh_srv_name, true);

  // Wait for model description service
  string get_model_desc_srv_name;
  lnh_.param("get_model_desc_srv", get_model_desc_srv_name, 
      string("get_model_desc_srv"));
  ROS_INFO("Waiting for %s service", get_model_desc_srv_name.c_str());
  while ( !ros::service::waitForService(get_model_desc_srv_name)) 
    ;
  get_model_desc_srv_ = 
    nh_.serviceClient<hodm::GetModelDescription> 
                              (get_model_desc_srv_name, true);

  // Load models from database
  lnh_.param<string>("model_set", model_set_, "DEBUG_MODEL_SET");
  detector_.loadDatabaseModels(model_set_);

  // Advertise object detection service
  object_detection_srv_ = nh_.advertiseService(
          nh_.getNamespace() + "/" + ros::this_node::getName() + "/detect_object", 
          &ObjectDetector::serviceCallback, this);
}


int main (int argc, char ** argv)
{
  ros::init(argc, argv, "object_detector");
  ros::NodeHandle nh;
  ROS_INFO("object_detector: starting");
  ObjectDetector od(nh);
  ros::spin();
  ROS_INFO("object_detector: exiting");
  return 0; 
}

