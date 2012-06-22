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



#ifndef OBJECT_SEGMENTATION_UI
#define OBJECT_SEGMENTATION_UI

#include <wx/wx.h>
#include <wx/event.h>

#include <ros/ros.h>
#include <time.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_object_detector/TabletopDetectionResult.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <actionlib/client/simple_action_client.h>
#include <rgbd_assembler/RgbdAssembly.h>

#include <simple_robot_control/robot_control.h>

#include "ogre_tools/wx_ogre_render_window.h"
#include "ogre_tools/initialization.h"

#include <boost/shared_ptr.hpp>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureUnitState.h>


#include "image_segmentation_demo/grabcut3d_object_segmenter.h"
#include "image_segmentation_demo/table_detector.h"
#include "image_segmentation_demo/ros_image_texture.h"
#include "object_segmentation_frame.h"

namespace image_segmentation_demo {

class ObjectSegmentationUI : public ObjectSegmentationFrame
 {
 public:
     ObjectSegmentationUI(wxWindow* parentwindow);
     virtual ~ObjectSegmentationUI();
     virtual void acceptButtonClicked( wxCommandEvent& );
     virtual void cancelButtonClicked( wxCommandEvent& );
     virtual void resetButtonClicked( wxCommandEvent& );
     virtual void segmentButtonClicked( wxCommandEvent& );
     virtual void refreshButtonClicked(wxCommandEvent&);
     virtual void onRenderWindowMouseEvents( wxMouseEvent& event );
     virtual void update(float wall_dt, float ros_dt);
     void cleanupAndQuit();
 private:

     Ogre::Root* root_;
     Ogre::SceneManager* scene_manager_;
     ogre_tools::wxOgreRenderWindow* render_window_;
     Ogre::Camera* camera_;

     //! The node handle
     ros::NodeHandlePtr nh;
     ros::Publisher chatter_pub;
     	
	
     std::string OBJECT_DETECTION_SERVICE_NAME;
     std::string COLLISION_PROCESSING_SERVICE_NAME;
     std::string PICKUP_ACTION_NAME;
     std::string PLACE_ACTION_NAME;
     std::string RGBD_ASSEMBLY_SERVICE_NAME;
     std::string ARM_ACTION_NAME;

     std::string data_location_;
     bool recording_time_;
     time_t start_time_;
     time_t stop_time_;
     std::string data_string_;
     int num_resets_;
     bool object_grasp_;

     ros::ServiceClient rgbd_assembler_client_;
     ros::ServiceClient object_detection_srv;
     ros::ServiceClient collision_processing_srv;

     // collection of the sensor data
     sensor_msgs::Image current_image_;
     sensor_msgs::Image display_image_;
     sensor_msgs::Image current_depth_image_;
     stereo_msgs::DisparityImage current_disparity_image_;
     sensor_msgs::CameraInfo  current_camera_info_;
     sensor_msgs::PointCloud2 current_point_cloud_;

     // Segmentation object
     GrabCut3DObjectSegmenter* object_segmenter_;

     //! A tf transform listener
  tf::TransformListener listener_;
     // detect table
     TableDetector  table_detector_;
     tabletop_object_detector::TabletopDetectionResult detection_call_;
     ROSImageTexture* texture_;

     rgbd_assembler::RgbdAssembly rgbd_assembler_srv_;

     simple_robot_control::Robot robot_;

     actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> *pickup_client;
     actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction>  *place_client;


     // segmentation mask
         cv::Mat                     binary_mask_;


     /** Point clusters */
     std::vector<sensor_msgs::PointCloud> clusters_;

     //! Publisher for markers
     ros::Publisher marker_pub_;
     //! Used to remember the number of markers we publish so we can delete them later
     int num_markers_published_;
     //! The current marker being published
     int current_marker_id_;


     // creates an RGB image from point cloud that comes in
     void fillRgbImage(sensor_msgs::Image &rgb_img,  const sensor_msgs::PointCloud2 &point_cloud);
     void setUpEventHandlers();
     void update();
     bool GetSensorData();
     bool getDepthImage(sensor_msgs::Image& image_msg);
     void SetUpPickandPlaceApp();
     void RunRestOfPickAndPlace();
     void updateImageOverlay();
     void updateView();
     void saveData();
     // convert cv to image message
     bool convertMatToImageMessage(const Mat& image, sensor_msgs::Image& image_msg);
     bool convertImageMessageToMat(const sensor_msgs::Image& image_msg, Mat& image);
     void resetVars();
     void reconstructAndClusterPointCloud();
     bool transformPointCloud(const std::string &target_frame, const sensor_msgs::PointCloud2& cloud_in, sensor_msgs::PointCloud2& cloud_out);
 };
  
}
#endif
