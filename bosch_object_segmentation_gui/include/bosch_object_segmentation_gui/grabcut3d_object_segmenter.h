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

#ifndef GRABCUT3D_OBJECT_SEGMENTER_H
#define GRABCUT3D_OBJECT_SEGMENTER_H

#include "bosch_object_segmentation_gui/ObjectSegmentationGuiAction.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <ros/ros.h>

using namespace std;
using namespace cv;

namespace bosch_object_segmentation_gui {
  
  class GrabCut3DObjectSegmenter
  {
  public:
      /* Allowed background colors for OpenCV window */
      enum MouseEvent { LEFT_BUTTON_DOWN, RIGHT_BUTTON_DOWN, LEFT_BUTTON_UP, RIGHT_BUTTON_UP, MOUSE_MOVE};

      /* States of grabbing/marking action */
      enum GrabState { NOT_SET, IN_PROCESS, SET };

      /* Allowed background colors for OpenCV window */
      enum WinColor { BLACK, GRAY, WHITE, GREEN, BLUE };

      /* Marker for default rectangle. */
      static const Rect DEFAULT_RECT;

      /* Constructor/destructor */
      GrabCut3DObjectSegmenter();
      ~GrabCut3DObjectSegmenter();

      /* Accessors for application state */
      inline bool initialized() { return initialized_; }
      void initializedIs(bool);

      /* Accessors/mutators for base image and mask */
      void setImages(const Mat& image, const Mat& depth_image);
      inline const Mat& image() { return image_; }
      inline const Mat& mask() { return mask_; }
      inline const Mat& displayImage() { return display_image_; }
      inline const Mat& depthImage() { return depth_image_; }
      Mat binaryMask();

      /* update display image */
      void updateDisplayImage();
      /* get window color */
      inline WinColor winColor() { return win_color_; }
      /* set window color */
      void setWinColor(WinColor _c);

      /* Mutator for GC rectangle. */
      void rectIs(const Rect&);

      /* Accessor for state variables */
      inline GrabState rectState() { return rect_state_; }

      /* Mouse callback for image topic subscription */
      void mouseClick( MouseEvent event, int x, int y, bool control_down, bool shift_down);

      /* Iteration control accessors */
      inline int iterCount() const { return iter_count_; }
      void iterCountIs(int _icnt);
      inline void iterCountInc() { iterCountIs(iterCount()+1); }

  private:
      /* Transfer marked rectangle to mask image */
      void setRectInMask();
      /* Transfer marked labels to mask image */
      void setLblsInMask(bool control_down, bool shift_down, Point p, bool isPr );

      /* Current state of OpenCV window image */
      WinColor win_color_;

      /* Segmentation process data */
      Mat image_;                 // image to be segmented
      Mat depth_image_;           // depth image to be segmented
      Mat display_image_;         // image used for rendering the results
      Mat mask_;                  // fg/bg mask for grabcut algo.
      Mat bgd_model_, fgd_model_; // temporary arrays used in segmentation
      Rect rect_;                 // marked rectangle
      vector<Point> fgd_pxls_;    // definite foreground points
      vector<Point> bgd_pxls_;    // definite background points
      vector<Point> pr_fgd_pxls_; // probable foreground points
      vector<Point> pr_bgd_pxls_; // probable background points
      int iter_count_;            // number of most recent completed iteration

      /* Current state of grabbing / marking process */
      GrabState rect_state_, lbls_state_, pr_lbls_state_;

      /* Used to allow client to return application to uninitialized state. */
      bool initialized_;

  };
}

#endif // GRABCUT3D_OBJECT_SEGMENTER_H
