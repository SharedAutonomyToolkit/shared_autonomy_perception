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

#include <ros/ros.h>

//#include "bosch_object_segmentation_gui/ObjectSegmentationGuiAction.h"
#include <actionlib/server/simple_action_server.h>

#include "image_segmentation_demo/grabcut3d_object_segmenter.h"
#include "image_segmentation_demo/table_detector.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <tabletop_object_detector/TabletopDetection.h>

#include <wx/event.h>

//#include <rviz/visualization_manager.h>
#include <OGRE/OgreManualObject.h>

#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>

#include "object_segmentation_frame.h"
#include <image_transport/image_transport.h>

namespace rviz_interaction_tools {
class ImageOverlay;
class DisparityRenderer;
}

namespace rviz {
class WindowManagerInterface;
class RenderPanel;
}

namespace image_segmentation_demo {
  
  class ObjectSegmentationUI : public ObjectSegmentationFrame
  {
    typedef geometry_msgs::Point32    Point;
    
  public:
    ObjectSegmentationUI();
    virtual ~ObjectSegmentationUI();
    
    virtual void update(float wall_dt, float ros_dt);


    //start listening to action goals
 //   void startActionServer( ros::NodeHandle &node_handle );
    
    //stop action server, cancel current goal & hide if necessary
 //   void stopActionServer();

  protected:

    // callback for new action server goals
//    void acceptNewGoal();

    // callback for action server preempt (cancel) requests
 //   void preempt();
  
    virtual void onRenderWindowMouseEvents( wxMouseEvent& event );

    // button events
    virtual void acceptButtonClicked( wxCommandEvent& );
    virtual void cancelButtonClicked( wxCommandEvent& );
    virtual void resetButtonClicked( wxCommandEvent& );
    virtual void segmentButtonClicked( wxCommandEvent& );

    //cleanup ogre scene, hide window
    void cleanupAndHide();

    // stop segmentation by sending a stop message
    void stopSegmentation();

    //put ogre panel into the window
    void createRenderPanel();

    rviz::WindowManagerInterface* window_manager_;
    rviz::RenderPanel* render_panel_;

    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* scene_root_;

    rviz_interaction_tools::ImageOverlay* image_overlay_;

    void reconstructAndClusterPointCloud(tabletop_object_detector::TabletopDetection &result);

    
 //   actionlib::SimpleActionServer<ObjectSegmentationGuiAction> *object_segmentation_server_;
    
  private:
  
    enum ResetType{PLAIN, COLOR, SURFACE, DISP, HOLES, UNIFORM};

    //! The node handle
    ros::NodeHandle nh_;
    
     //! The node handle
    ros::NodeHandle priv_nh_;
    
    //! Publishing images (for debug only)
    image_transport::Publisher image_pub_;

    // overlays current result from segmenter with image from camera
    void overlaySegmentationMask();
    
    // creates an RGB image from point cloud that comes in
    void fillRgbImage(sensor_msgs::Image &rgb_img,
		      const sensor_msgs::PointCloud2 &point_cloud);
 
    // get a single depth image
    bool getDepthImage(sensor_msgs::Image& image_msg);
    // convert image message to cv
    bool convertImageMessageToMat(const sensor_msgs::Image& image_msg, Mat& image);
    // convert cv to image message
    bool convertMatToImageMessage(const Mat& image, sensor_msgs::Image& image_msg);
    // update display image
    void updateImageOverlay();
    // reset internal state
    void resetVars( );


    // Segmentation object
    GrabCut3DObjectSegmenter* object_segmenter_;

    // detect table
    TableDetector  table_detector_;
    
    // stores sensor data
    sensor_msgs::Image          current_image_;
    sensor_msgs::Image          current_depth_image_;
    stereo_msgs::DisparityImage current_disparity_image_;
    sensor_msgs::PointCloud2    current_point_cloud_;
    sensor_msgs::CameraInfo     current_camera_info_;
    
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
  };

}

#endif
