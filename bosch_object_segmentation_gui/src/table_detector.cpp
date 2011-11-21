/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Robert Bosch LLC.
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

#include "bosch_object_segmentation_gui/table_detector.h"

#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

namespace bosch_object_segmentation_gui {
  
TableDetector::TableDetector() : current_marker_id_(0)
{
  //initialize operational flags
  inlier_threshold_ = 300;
  plane_detection_voxel_size_ = 0.01;
  clustering_voxel_size_ = 0.003;
  z_filter_min_ = 0.4;
  z_filter_max_ = 1.25;
  table_z_filter_min_ = 0.01;
  table_z_filter_max_ = 0.50;
  cluster_distance_ = 0.03;
  min_cluster_size_ = 30;
  processing_frame_ = "";
  up_direction_ = -1.0;
}

TableDetector::~TableDetector()
{
}

bool TableDetector::detectTable(const sensor_msgs::PointCloud2 &cloud, tabletop_object_detector::Table& table)
{
  ROS_INFO("Starting process on new cloud");
  ROS_INFO("In frame %s", cloud.header.frame_id.c_str());

  // PCL objects
  KdTreePtr normals_tree_, clusters_tree_;
  pcl::VoxelGrid<Point> grid_, grid_objects_;
  pcl::PassThrough<Point> pass_;
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  pcl::ProjectInliers<Point> proj_;
  pcl::ConvexHull<Point> hull_;
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::EuclideanClusterExtraction<Point> pcl_cluster_;

  // Filtering parameters
  grid_.setLeafSize (plane_detection_voxel_size_, plane_detection_voxel_size_, plane_detection_voxel_size_);
  grid_objects_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);
  grid_.setFilterFieldName ("z");
  pass_.setFilterFieldName ("z");

  pass_.setFilterLimits (z_filter_min_, z_filter_max_);
  grid_.setFilterLimits (z_filter_min_, z_filter_max_);
  grid_.setDownsampleAllData (false);
  grid_objects_.setDownsampleAllData (false);

  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
  clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();

  // Normal estimation parameters
  n3d_.setKSearch (10);
  n3d_.setSearchMethod (normals_tree_);
  // Table model fitting parameters
  seg_.setDistanceThreshold (0.05);
  seg_.setMaxIterations (10000);
  seg_.setNormalDistanceWeight (0.1);
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setProbability (0.99);

  proj_.setModelType (pcl::SACMODEL_PLANE);

  // Consider only objects in a given layer above the table
  prism_.setHeightLimits (table_z_filter_min_, table_z_filter_max_);

  // Clustering parameters
  pcl_cluster_.setClusterTolerance (cluster_distance_);
  pcl_cluster_.setMinClusterSize (min_cluster_size_);
  pcl_cluster_.setSearchMethod (clusters_tree_);

  // Step 1 : Filter, remove NaNs and downsample
  pcl::PointCloud<Point> cloud_t;
  pcl::fromROSMsg (cloud, cloud_t);
  pcl::PointCloud<Point>::ConstPtr cloud_ptr = boost::make_shared<const pcl::PointCloud<Point> > (cloud_t);

  pcl::PointCloud<Point> cloud_filtered;
  pass_.setInputCloud (cloud_ptr);
  pass_.filter (cloud_filtered);
  pcl::PointCloud<Point>::ConstPtr cloud_filtered_ptr =
    boost::make_shared<const pcl::PointCloud<Point> > (cloud_filtered);
  ROS_INFO("Step 1 done");
  if (cloud_filtered.points.size() < (unsigned int)min_cluster_size_)
  {
    ROS_INFO("Filtered cloud only has %d points", (int)cloud_filtered.points.size());
    return false;
  }

  pcl::PointCloud<Point> cloud_downsampled;
  grid_.setInputCloud (cloud_filtered_ptr);
  grid_.filter (cloud_downsampled);
  pcl::PointCloud<Point>::ConstPtr cloud_downsampled_ptr =
    boost::make_shared<const pcl::PointCloud<Point> > (cloud_downsampled);
  if (cloud_downsampled.points.size() < (unsigned int)min_cluster_size_)
  {
    ROS_INFO("Downsampled cloud only has %d points", (int)cloud_downsampled.points.size());
    return false;
  }

  // Step 2 : Estimate normals
  pcl::PointCloud<pcl::Normal> cloud_normals;
  n3d_.setInputCloud (cloud_downsampled_ptr);
  n3d_.compute (cloud_normals);
  pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr =
    boost::make_shared<const pcl::PointCloud<pcl::Normal> > (cloud_normals);
  ROS_INFO("Step 2 done");

  // Step 3 : Perform planar segmentation
  pcl::PointIndices table_inliers; pcl::ModelCoefficients table_coefficients;
  seg_.setInputCloud (cloud_downsampled_ptr);
  seg_.setInputNormals (cloud_normals_ptr);
  seg_.segment (table_inliers, table_coefficients);
  pcl::PointIndices::ConstPtr table_inliers_ptr = boost::make_shared<const pcl::PointIndices> (table_inliers);
  pcl::ModelCoefficients::ConstPtr table_coefficients_ptr =
    boost::make_shared<const pcl::ModelCoefficients> (table_coefficients);

  if (table_coefficients.values.size () <=3)
  {
    ROS_INFO("Failed to detect table in scan");
    return false;
  }

  if ( table_inliers.indices.size() < (unsigned int)inlier_threshold_)
  {
    ROS_INFO("Plane detection has %d inliers, below min threshold of %d", (int)table_inliers.indices.size(),
       inlier_threshold_);
    return false;
  }

  ROS_INFO ("[TableObjectDetector::input_callback] Model found with %d inliers: [%f %f %f %f].",
      (int)table_inliers.indices.size (),
      table_coefficients.values[0], table_coefficients.values[1],
      table_coefficients.values[2], table_coefficients.values[3]);
  ROS_INFO("Step 3 done");

  // Step 4 : Project the table inliers on the table
  pcl::PointCloud<Point> table_projected;
  proj_.setInputCloud (cloud_downsampled_ptr);
  proj_.setIndices (table_inliers_ptr);
  proj_.setModelCoefficients (table_coefficients_ptr);
  proj_.filter (table_projected);
  pcl::PointCloud<Point>::ConstPtr table_projected_ptr =
    boost::make_shared<const pcl::PointCloud<Point> > (table_projected);
  ROS_INFO("Step 4 done");

  sensor_msgs::PointCloud table_points;
  tf::Transform table_plane_trans = getPlaneTransform (table_coefficients, up_direction_);
  //takes the points projected on the table and transforms them into the PointCloud message
  //while also transforming them into the table's coordinate system
  if (!getPlanePoints(table_projected, table_plane_trans, table_points))
  {
    return false;
  }
  ROS_INFO("Table computed");

  table = getTable(cloud.header, table_plane_trans, table_points);
  return true;
}

tabletop_object_detector::Table
TableDetector::getTable(const std_msgs::Header& cloud_header, const tf::Transform &table_plane_trans, const sensor_msgs::PointCloud &table_points)
{
  tabletop_object_detector::Table table;

  //get the extents of the table
  if (!table_points.points.empty())
  {
    table.x_min = table_points.points[0].x;
    table.x_max = table_points.points[0].x;
    table.y_min = table_points.points[0].y;
    table.y_max = table_points.points[0].y;
  }
  for (size_t i=1; i<table_points.points.size(); ++i)
  {
    if (table_points.points[i].x<table.x_min && table_points.points[i].x>-3.0) table.x_min = table_points.points[i].x;
    if (table_points.points[i].x>table.x_max && table_points.points[i].x< 3.0) table.x_max = table_points.points[i].x;
    if (table_points.points[i].y<table.y_min && table_points.points[i].y>-3.0) table.y_min = table_points.points[i].y;
    if (table_points.points[i].y>table.y_max && table_points.points[i].y< 3.0) table.y_max = table_points.points[i].y;
  }

  geometry_msgs::Pose table_pose;
  tf::poseTFToMsg(table_plane_trans, table_pose);
  table.pose.pose = table_pose;
  table.pose.header.frame_id = cloud_header.frame_id;
  table.pose.header.stamp = ros::Time::now();
  return table;
}

/*! The point cloud is a set of points belonging to the plane, in the plane coordinate system
  (with the origin in the plane and the z axis normal to the plane).

  It is the responsibility of the caller to set the appropriate pose for the marker so that
  it shows up in the right reference frame.
 */
visualization_msgs::Marker TableDetector::getTableMarker(const tabletop_object_detector::Table& table)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = table.pose.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.pose = table.pose.pose;
  marker.ns = "table_detector";
  marker.id = current_marker_id_++;

  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.lifetime = ros::Duration();

  //create the marker in the table reference frame
  //the caller is responsible for setting the pose of the marker to match

  marker.scale.x = 0.001;
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;

  marker.points.resize(5);
  marker.points[0].x = table.x_min;
  marker.points[0].y = table.y_min;
  marker.points[0].z = 0;

  marker.points[1].x = table.x_min;
  marker.points[1].y = table.y_max;
  marker.points[1].z = 0;

  marker.points[2].x = table.x_max;
  marker.points[2].y = table.y_max;
  marker.points[2].z = 0;

  marker.points[3].x = table.x_max;
  marker.points[3].y = table.y_min;
  marker.points[3].z = 0;

  marker.points[4].x = table.x_min;
  marker.points[4].y = table.y_min;
  marker.points[4].z = 0;

  marker.points.resize(6);
  marker.points[5].x = table.x_min;
  marker.points[5].y = table.y_min;
  marker.points[5].z = 0.02;

  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  return marker;
}

/*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
tf::Transform TableDetector::getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction)
{
  ROS_ASSERT(coeffs.values.size() > 3);
  double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
  //asume plane coefficients are normalized
  btVector3 position(-a*d, -b*d, -c*d);
  btVector3 z(a, b, c);
  //make sure z points "up"
  ROS_DEBUG("z.dot: %0.3f", z.dot(btVector3(0,0,1)));
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
  if ( z.dot( btVector3(0, 0, up_direction) ) < 0)
  {
    z = -1.0 * z;
    ROS_INFO("flipped z");
  }
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);

  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  btVector3 x(1, 0, 0);
  if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = btVector3(0, 1, 0);
  btVector3 y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  btMatrix3x3 rotation;
  rotation[0] = x;  // x
  rotation[1] = y;  // y
  rotation[2] = z;  // z
  rotation = rotation.transpose();
  btQuaternion orientation;
  rotation.getRotation(orientation);
  return tf::Transform(orientation, position);
}

bool TableDetector::getPlanePoints (const pcl::PointCloud<Point> &table,
                                     const tf::Transform& table_plane_trans,
                                     sensor_msgs::PointCloud &table_points)
{
  // Prepare the output
  table_points.header.frame_id = table.header.frame_id;
  table_points.header.stamp = table.header.stamp;
  table_points.points.resize (table.points.size ());
  for (size_t i = 0; i < table.points.size (); ++i)
  {
    table_points.points[i].x = table.points[i].x;
    table_points.points[i].y = table.points[i].y;
    table_points.points[i].z = table.points[i].z;
  }

  // Transform the data
  tf::TransformListener listener;
  tf::StampedTransform table_pose_frame(table_plane_trans, table.header.stamp,
                                        table.header.frame_id, "table_frame");
  listener.setTransform(table_pose_frame);
  std::string error_msg;
  if (!listener.canTransform("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
  {
    ROS_ERROR("Can not transform point cloud from frame %s to table frame; error %s",
        table_points.header.frame_id.c_str(), error_msg.c_str());
    return false;
  }
  try
  {
    listener.transformPointCloud("table_frame", table_points, table_points);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s",
        table_points.header.frame_id.c_str(), ex.what());
    return false;
  }
  table_points.header.stamp = ros::Time::now();
  table_points.header.frame_id = "table_frame";
  return true;
}


} // object_segmentation_gui


