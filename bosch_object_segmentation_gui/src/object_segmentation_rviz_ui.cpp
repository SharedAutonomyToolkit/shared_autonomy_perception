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


#include "bosch_object_segmentation_gui/object_segmentation_rviz_ui.h"
#include "bosch_object_segmentation_gui/utils.h"

#include "rviz_interaction_tools/image_tools.h"
#include "rviz_interaction_tools/image_overlay.h"
#include "rviz_interaction_tools/camera_tools.h"
#include "rviz_interaction_tools/unique_string_manager.h"

#include <rviz/window_manager_interface.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/validate_floats.h>
#include <rviz/frame_manager.h>

#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>

#include "tabletop_object_detector/marker_generator.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;


namespace bosch_object_segmentation_gui
{

static const char
    * HELP_TEXT =
        "Left Mouse Button: Drag a rectangle around an object to segment it. \n\
         SHIFT+Left Mouse Button: Mark pixel belonging to the object. \n\
         CTRL+Left Mouse Button: Mark pixel belonging to the background. \n\
         \n\
         Segment Button: Perform Segmentation. \n\
         Reset Button: Reset Segmentation. \n\
         Cancel Button: Cancel Segmentation. \n\
         OK Button: Accept Segmentation. ";

ObjectSegmentationRvizUI::ObjectSegmentationRvizUI(
    rviz::VisualizationManager *visualization_manager ) :
  ObjectSegmentationFrame(
      visualization_manager->getWindowManager()->getParentWindow()),
      object_segmentation_server_(0), nh_(""), priv_nh_("~"),
      object_segmenter_(0), num_markers_published_(1), current_marker_id_(1)
{
  rviz_interaction_tools::UniqueStringManager usm;

  //construct basic ogre scene
  scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, usm.unique("ObjectSegmentationRvizUI"));
  scene_root_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  image_overlay_ = new rviz_interaction_tools::ImageOverlay(scene_root_, Ogre::RENDER_QUEUE_OVERLAY - 2);

  //put a render panel into the window
  createRenderPanel(visualization_manager);

  SetTitle(wxString::FromAscii("GrabCut3D Interactive Object Segmentation"));
  bottom_label_->SetLabel(wxString::FromAscii(HELP_TEXT));

  marker_pub_ = nh_.advertise<visualization_msgs::Marker> (nh_.resolveName("tabletop_segmentation_markers"), 10);

  //initialize operational flags
  priv_nh_.param<int>("inlier_threshold", table_detector_.inlier_threshold_, 300);
  priv_nh_.param<double>("plane_detection_voxel_size", table_detector_.plane_detection_voxel_size_, 0.01);
  priv_nh_.param<double>("clustering_voxel_size", table_detector_.clustering_voxel_size_, 0.003);
  priv_nh_.param<double>("z_filter_min", table_detector_.z_filter_min_, 0.4);
  priv_nh_.param<double>("z_filter_max", table_detector_.z_filter_max_, 1.25);
  priv_nh_.param<double>("table_z_filter_min", table_detector_.table_z_filter_min_, 0.01);
  priv_nh_.param<double>("table_z_filter_max", table_detector_.table_z_filter_max_, 0.50);
  priv_nh_.param<double>("cluster_distance", table_detector_.cluster_distance_, 0.03);
  priv_nh_.param<int>("min_cluster_size", table_detector_.min_cluster_size_, 300);
  priv_nh_.param<std::string>("processing_frame", table_detector_.processing_frame_, "");
  priv_nh_.param<double>("up_direction", table_detector_.up_direction_, -1.0);

  object_segmenter_ = new GrabCut3DObjectSegmenter();
}

ObjectSegmentationRvizUI::~ObjectSegmentationRvizUI()
{
  if (object_segmentation_server_)
    stopActionServer();

  render_panel_->getRenderWindow()->setActive(false);
  delete render_panel_;
  delete image_overlay_;
  delete object_segmenter_;
}

void ObjectSegmentationRvizUI::createRenderPanel(
    rviz::VisualizationManager *visualization_manager )
{
  //put a render panel into the window
  render_panel_ = new rviz::RenderPanel(this, false);

  render_panel_->SetSize(m_panel->GetRect());

  // notify us when the user does something with the mouse
  render_panel_->Connect(wxEVT_MOTION,wxMouseEventHandler( ObjectSegmentationRvizUI::onRenderWindowMouseEvents ), NULL, this);
  render_panel_->Connect(wxEVT_LEFT_DOWN,wxMouseEventHandler( ObjectSegmentationRvizUI::onRenderWindowMouseEvents ), NULL, this);
  render_panel_->Connect(wxEVT_MIDDLE_DOWN,wxMouseEventHandler( ObjectSegmentationRvizUI::onRenderWindowMouseEvents ), NULL, this);
  render_panel_->Connect(wxEVT_RIGHT_DOWN,wxMouseEventHandler( ObjectSegmentationRvizUI::onRenderWindowMouseEvents ), NULL, this);
  render_panel_->Connect(wxEVT_LEFT_UP,wxMouseEventHandler( ObjectSegmentationRvizUI::onRenderWindowMouseEvents ), NULL, this);
  render_panel_->Connect(wxEVT_MIDDLE_UP, wxMouseEventHandler( ObjectSegmentationRvizUI::onRenderWindowMouseEvents ), NULL, this);
  render_panel_->Connect(wxEVT_RIGHT_UP, wxMouseEventHandler( ObjectSegmentationRvizUI::onRenderWindowMouseEvents ), NULL, this);
  render_panel_->Connect(wxEVT_MOUSEWHEEL, wxMouseEventHandler( ObjectSegmentationRvizUI::onRenderWindowMouseEvents ), NULL, this);
  render_panel_->Connect(wxEVT_LEFT_DCLICK, wxMouseEventHandler( ObjectSegmentationRvizUI::onRenderWindowMouseEvents ), NULL, this);

  render_panel_->createRenderWindow();
  render_panel_->initialize(scene_manager_, visualization_manager);

  render_panel_->setAutoRender(false);
  render_panel_->getViewport()->setOverlaysEnabled(false);
  render_panel_->getViewport()->setClearEveryFrame(true);
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive(true);

}

void ObjectSegmentationRvizUI::startActionServer( ros::NodeHandle &node_handle )
{
  if (object_segmentation_server_)
  {
    ROS_ERROR( "ObjectSegmentationGuiAction server already started!" );
    return;
  }

  ROS_INFO("Starting ObjectSegmentationGuiAction server.");

  //create non-threaded action server
  object_segmentation_server_ = new actionlib::SimpleActionServer< ObjectSegmentationGuiAction>(node_handle, "segmentation_popup", false);

  object_segmentation_server_->registerGoalCallback(boost::bind(&ObjectSegmentationRvizUI::acceptNewGoal, this));
  object_segmentation_server_->registerPreemptCallback(boost::bind(&ObjectSegmentationRvizUI::preempt, this));

  object_segmentation_server_->start();
}

void ObjectSegmentationRvizUI::stopActionServer()
{
  if (!object_segmentation_server_)
  {
    ROS_ERROR("ObjectSegmentationGuiAction server cannot be stopped because it is not running.");
    return;
  }

  //if we're currently being active, we have to cancel everything, clean up & hide the window
  if (object_segmentation_server_->isActive())
  {
    ROS_WARN("Aborting ObjectSegmentationGuiAction goal.");
    stopSegmentation();
    object_segmentation_server_->setAborted();
    cleanupAndHide();
  }

  ROS_INFO("Stopping ObjectSegmentationGuiAction server.");
  delete object_segmentation_server_;
  object_segmentation_server_ = 0;
}

bool ObjectSegmentationRvizUI::getDepthImage(sensor_msgs::Image& image_msg)
{
 // get image
 std::string image_topic = nh_.resolveName("/camera/depth/image");
 ROS_INFO("Waiting for image on topic %s", image_topic.c_str());
 sensor_msgs::ImageConstPtr image_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic, nh_, ros::Duration(30.0));
 if(!image_ptr) {
   ROS_ERROR("Could not receive an image!");
   return false;
 }
 image_msg = *image_ptr;

 float max_val = -FLT_MAX;
 float min_val = FLT_MAX;
 for(unsigned int x=0; x<image_msg.width; ++x)
 {
   for(unsigned int y=0; y<image_msg.height; ++y)
   {
     int index_src = y * image_ptr->step + x*4;
     float* val = (float*)(&image_ptr->data[index_src]);
     max_val = std::max(*val,max_val);
     min_val = std::min(*val,min_val);
   }
 }

 image_msg.encoding = "bgr8";
 image_msg.width = image_ptr->width;
 image_msg.height = image_ptr->height;
 image_msg.step = 3 * image_ptr->width;
 image_msg.data.resize(image_ptr->step * image_ptr->height);
 for(unsigned int x=0; x<image_msg.width; ++x)
 {
   for(unsigned int y=0; y<image_msg.height; ++y)
   {
     int index_src = y * image_ptr->step + x*4;
     float* val = (float*)(&image_ptr->data[index_src]);

     int index_dest = y * image_msg.step + x*3;
     image_msg.data[index_dest+0] = round(*val);
     image_msg.data[index_dest+1] = round(*val);
     image_msg.data[index_dest+2] = round(*val);
   }
 }

 return true;
}

void ObjectSegmentationRvizUI::acceptNewGoal()
{

  // only if segmentation is running labeling can be accepted
  accept_button_->Enable(false);

  // only if segmentation is not running, it can be started
  segment_button_->Enable(true);

  const ObjectSegmentationGuiGoal::ConstPtr &goal = object_segmentation_server_->acceptNewGoal();

  rviz_interaction_tools::updateCamera(render_panel_->getCamera(), goal->camera_info);

  ROS_ASSERT( goal->point_cloud.height == goal->disparity_image.image.height &&
              goal->point_cloud.width == goal->disparity_image.image.width);

  current_point_cloud_ = goal->point_cloud;
  current_camera_info_ = goal->camera_info;
  current_disparity_image_ = goal->disparity_image;


  fillRgbImage(current_image_, current_point_cloud_);
  //current_image_ = goal->image;

  getDepthImage(current_depth_image_);

  //store data for later use in update()
  //we can't directly put in in ogre because
  //this runs in a different thread
  image_overlay_->setImage(current_image_);
  image_overlay_->update();

  Show();

  // initialise segmentation
  Mat image, depth_image;
  convertImageMessageToMat(current_image_, image);
  convertImageMessageToMat(current_depth_image_, depth_image);
  object_segmenter_->setImages(image, depth_image);
}

bool ObjectSegmentationRvizUI::convertImageMessageToMat(const sensor_msgs::Image& image_msg, Mat& image)
{
  // converts ROS images to OpenCV
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  image = cv_ptr->image;

  return true;
}

bool ObjectSegmentationRvizUI::convertMatToImageMessage(const Mat& image, sensor_msgs::Image& image_msg)
{
  // converts OpenCV to ROS images
  cv_bridge::CvImage cv_image;
  cv_image.image = image;
  cv_image.encoding = "bgr8";

  try
  {
    cv_image.toImageMsg(image_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
  return true;
}

void ObjectSegmentationRvizUI::preempt()
{
  stopSegmentation();
  object_segmentation_server_->setPreempted();
  cleanupAndHide();
}

void ObjectSegmentationRvizUI::cleanupAndHide()
{
  image_overlay_->clear();
  resetVars();
  Hide();
}

void ObjectSegmentationRvizUI::stopSegmentation()
{
}

void ObjectSegmentationRvizUI::resetVars()
{
  for (size_t i = 0; i < clusters_.size(); ++i)
    clusters_[i].points.clear();

  clusters_.clear();

  image_overlay_->setImage(current_image_);
  image_overlay_->update();

  // only if segmentation is running labeling can be accepted
  accept_button_->Enable(false);

  // only if segmentation is not running, it can be started
  segment_button_->Enable(true);
}

void ObjectSegmentationRvizUI::update( float wall_dt, float ros_dt )
{
  render_panel_->getRenderWindow()->update();
}

void ObjectSegmentationRvizUI::onRenderWindowMouseEvents( wxMouseEvent& event )
{
  int x = event.GetX();
  int y = event.GetY();
  //convert to coordinates in the original image resolution
  wxSize size = render_panel_->GetSize();
  x = floor(x * current_image_.width / size.GetWidth());
  y = floor(y * current_image_.height / size.GetHeight());

  GrabCut3DObjectSegmenter::MouseEvent mouse_event;
  if (event.ButtonDown(wxMOUSE_BTN_LEFT))
  {
    mouse_event = GrabCut3DObjectSegmenter::LEFT_BUTTON_DOWN;
  }
  else if (event.ButtonUp(wxMOUSE_BTN_LEFT))
  {
    mouse_event = GrabCut3DObjectSegmenter::LEFT_BUTTON_UP;
  }
  else if (event.ButtonDown(wxMOUSE_BTN_RIGHT))
  {
    mouse_event = GrabCut3DObjectSegmenter::RIGHT_BUTTON_DOWN;
  }
  else if (event.ButtonUp(wxMOUSE_BTN_RIGHT))
  {
    mouse_event = GrabCut3DObjectSegmenter::RIGHT_BUTTON_UP;
  }
  else
  {
    mouse_event = GrabCut3DObjectSegmenter::MOUSE_MOVE;
  }

  if(event.Dragging())
  {
    mouse_event = GrabCut3DObjectSegmenter::MOUSE_MOVE;
  }

  object_segmenter_->mouseClick(mouse_event, x, y, event.ControlDown(), event.ShiftDown());

  // update image overlay
  updateImageOverlay();
}

void ObjectSegmentationRvizUI::updateImageOverlay()
{
  sensor_msgs::Image display_image;
  convertMatToImageMessage(object_segmenter_->displayImage(), display_image);
  image_overlay_->setImage(display_image);
  image_overlay_->update();
}

void ObjectSegmentationRvizUI::fillRgbImage(sensor_msgs::Image &rgb_img,
    const sensor_msgs::PointCloud2 &point_cloud)
{

  ROS_DEBUG("Width and Height: (%d %d)",point_cloud.height, point_cloud.width );

  rgb_img.header = point_cloud.header;
  rgb_img.height = point_cloud.height;
  rgb_img.width = point_cloud.width;
  rgb_img.encoding = enc::RGB8;
  rgb_img.is_bigendian = false;
  rgb_img.step = 3 * rgb_img.width;
  size_t size = rgb_img.step * rgb_img.height;
  rgb_img.data.resize(size);

  for(unsigned int x=0; x<rgb_img.width; ++x)
  {
    for(unsigned int y=0; y<rgb_img.height; ++y)
    {
      int i = y * rgb_img.width + x;

      float rgb;
      memcpy ( &rgb, &point_cloud.data[i * point_cloud.point_step + point_cloud.fields[3].offset], sizeof (float));
      float r, g, b;
      transformRgb(rgb, r, g, b);

      int wide_i = y * rgb_img.step + x*3;
      rgb_img.data[wide_i+0] = round(r*255.0f);
      rgb_img.data[wide_i+1] = round(g*255.0f);
      rgb_img.data[wide_i+2] = round(b*255.0f);

    }
  }
}

void ObjectSegmentationRvizUI::acceptButtonClicked(wxCommandEvent&)
{
  ObjectSegmentationGuiResult result;

  // Fetch binary mask from GCApp object
  binary_mask_ = object_segmenter_->binaryMask();

  // reconstruct clusters
  reconstructAndClusterPointCloud(result);

  // get table parameters in 3d
  if(table_detector_.detectTable(current_point_cloud_, result.table))
    result.result = result.SUCCESS;
  else
    result.result = result.NO_TABLE;


  ROS_INFO("ObjectSegmentation was successful.");

  object_segmentation_server_->setSucceeded(result);

  cleanupAndHide();
}

void ObjectSegmentationRvizUI::cancelButtonClicked(wxCommandEvent&)
{
  stopSegmentation();
  object_segmentation_server_->setAborted();
  cleanupAndHide();
}

void ObjectSegmentationRvizUI::resetButtonClicked( wxCommandEvent& )
{
  object_segmenter_->initializedIs(false);
  image_overlay_->clear();

  resetVars();
}

void ObjectSegmentationRvizUI::segmentButtonClicked( wxCommandEvent& )
{
  if( object_segmenter_->rectState() == GrabCut3DObjectSegmenter::SET)
  {
    object_segmenter_->iterCountInc();
    // update image overlay
    updateImageOverlay();
  }
  else
    ROS_INFO("Rectangle must be drawn first.");

  // only if segmentation is ran once labeling can be accepted
  accept_button_->Enable(true);
}

void ObjectSegmentationRvizUI::reconstructAndClusterPointCloud( ObjectSegmentationGuiResult &result)
{
  clusters_.clear();
  clusters_.resize(1);
  clusters_[0].header.frame_id = current_point_cloud_.header.frame_id;
  clusters_[0].header.stamp = ros::Time(0);
  clusters_[0].points.reserve(1000);

  for(unsigned int x=0; x<current_point_cloud_.width; ++x)
  {
    for(unsigned int y=0; y<current_point_cloud_.height; ++y)
    {
      int i = y * current_point_cloud_.width + x;
      int mask_val = int(binary_mask_.at<unsigned char>(y, x));
      float nan;
      memcpy (&nan,
          &current_point_cloud_.data[i * current_point_cloud_.point_step + current_point_cloud_.fields[0].offset],
          sizeof (float));
      if(!isnan(nan))
      {
        geometry_msgs::Point32 p;
        float x, y, z;
        memcpy (&x,
            &current_point_cloud_.data[i
            * current_point_cloud_.point_step
            + current_point_cloud_.fields[0].offset],
            sizeof (float));
        memcpy (&y,
            &current_point_cloud_.data[i
            * current_point_cloud_.point_step
            + current_point_cloud_.fields[1].offset],
            sizeof (float));
        memcpy (&z,
            &current_point_cloud_.data[i
            * current_point_cloud_.point_step
            + current_point_cloud_.fields[2].offset],
            sizeof (float));
        p.x = x;
        p.y = y;
        p.z = z;

        if (mask_val > 0)
          clusters_[0].points.push_back(p);
      }
    }
  }
  result.clusters = clusters_;
}

} // namespace object_segmentation_gui

