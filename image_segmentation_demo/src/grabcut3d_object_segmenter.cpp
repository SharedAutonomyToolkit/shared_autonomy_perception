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

#include "image_segmentation_demo/grabcut3d_object_segmenter.h"
#include "grabcut_3d/grabcut_3d.h"

using namespace std;
using namespace cv;

namespace image_segmentation_demo{

static const Scalar RD = Scalar(0,0,255);
static const Scalar PK = Scalar(230,130,255);
static const Scalar BL = Scalar(255,0,0);
static const Scalar LB = Scalar(255,255,160);
static const Scalar GR = Scalar(0,255,0);
static const Scalar WH = Scalar(255,255,255);
static const Scalar GY = Scalar(128,128,128);
static const Scalar BK = Scalar(0,0,0);
static const int DOT_RADIUS = 2;
static const int LINE_THICKNESS = -1;

// Initialize default rectangle marker (not actually used as a rectangle)
const Rect GrabCut3DObjectSegmenter::DEFAULT_RECT(0,0,0,0);
// This scale factor is what is actually used to calculate the default
// rectangle.
const int DEFAULT_RECT_SCALE_FACTOR = 2;

/* Helper function to convert mask in 4-value GrabCut format to a binary mask
 * with foreground = 1, background = 0. */
static void
binaryMaskFromGCMask(const Mat &_gc_mask, Mat &_bin_mask)
{
    if( _gc_mask.empty() || _gc_mask.type() != CV_8UC1 )
        CV_Error( CV_StsBadArg, "_gc_mask is empty or has incorrect type (not CV_8UC1)" );

    if(    _bin_mask.empty()
        || _bin_mask.rows!=_gc_mask.rows
        || _bin_mask.cols!=_gc_mask.cols )
        _bin_mask.create( _gc_mask.size(), CV_8UC1 );

    // Convert GC_BGD and GC_PR_BGD to 0, GC_FGD and GC_PR_FGD to 1
    _bin_mask = _gc_mask & 1;
}

GrabCut3DObjectSegmenter::GrabCut3DObjectSegmenter()
{
  // Initialize window
  //setImages(_img,_depth_image);
  updateDisplayImage();

  iter_count_ = 0;
}

GrabCut3DObjectSegmenter::~GrabCut3DObjectSegmenter()
{

}

/* Mutator for initialized state of GrabCut3DObjectSegmenter object. Client can set
 * initialized=true with no side effects; setting initialized=false clears
 * background and foreground pixel arrays and returns application state
 * variables to NOT_SET. */
// TODO: Should this be public?
void
GrabCut3DObjectSegmenter::initializedIs(bool _init)
{
  if (_init)
    initialized_=_init;
  else
  {
    if ( !mask_.empty() ) mask_.setTo(Scalar::all(GC_BGD));
    bgd_pxls_.clear();   fgd_pxls_.clear();
    pr_bgd_pxls_.clear();  pr_fgd_pxls_.clear();
    initialized_   = _init;
    rect_state_ = NOT_SET;
    lbls_state_ = NOT_SET;
    pr_lbls_state_ = NOT_SET;
    iterCountIs(0);
    updateDisplayImage();
  }
}

/* Mutator for stored image. */
void GrabCut3DObjectSegmenter::setImages(const Mat& _image, const Mat& _depth_image)
{
    if( _image.empty() ) {
      ROS_WARN("GrabCut3DObjectSegmenter: setImages called with empty image");
      return;
    }
    if (_depth_image.empty()){
    	ROS_WARN("GrabCut3DObjectSegmenter: setImages called with empty depth image");
    	return;
    }
    ROS_INFO_STREAM("image size " << _image.size().width << " "<<_image.size().height);
    image_ = _image.clone();
    ROS_INFO_STREAM("depth image size " << _depth_image.size().width << " "<<_depth_image.size().height);
    depth_image_ = _depth_image.clone();
    display_image_ = image_.clone();

    mask_.create( image_.size(), CV_8UC1);
    initializedIs(false);
}

/* Mutator for rectangle. */
void GrabCut3DObjectSegmenter::rectIs(const Rect &_r)
{
  if (_r == DEFAULT_RECT)
    rect_ = Rect(image_.cols/(DEFAULT_RECT_SCALE_FACTOR*2),
                 image_.rows/(DEFAULT_RECT_SCALE_FACTOR*2),
                 image_.cols/DEFAULT_RECT_SCALE_FACTOR,
                 image_.rows/DEFAULT_RECT_SCALE_FACTOR);
  else
    rect_=_r;

  rect_state_ = SET;

  ROS_INFO_STREAM("rect area is " << rect_.area());
  if (rect_.area()==0)
	rect_state_=EMPTY;

  setRectInMask();
  assert( bgd_pxls_.empty()
  && fgd_pxls_.empty()
  && pr_bgd_pxls_.empty()
  && pr_fgd_pxls_.empty() );
  updateDisplayImage();

}

/* Accessor for binary mask. */
Mat GrabCut3DObjectSegmenter::binaryMask()
{
  Mat bin_mask;
  /*
  cv::imwrite("/home/stm1pal/ros/stacks/bosch-ros-pkg/mas/grabcut_app/full_mask.pbm", mask_);
  double fm_max;
  double fm_min;
  minMaxLoc(mask_, &fm_min, &fm_max);
  std::cout << "fm_max: " << fm_max << std::endl;
  std::cout << "fm_min: " << fm_min << std::endl;
  */

  binaryMaskFromGCMask(mask_, bin_mask);
  /*
  cv::imwrite("/home/stm1pal/ros/stacks/bosch-ros-pkg/mas/grabcut_app/bin_mask.pbm", bin_mask);
  double bm_max;
  double bm_min;
  minMaxLoc(bin_mask, &bm_min, &bm_max);
  std::cout << "bm_max: " << bm_max << std::endl;
  std::cout << "bm_min: " << bm_min << std::endl;
  */
  return bin_mask;
}

/* Mutator for window background color. */
void GrabCut3DObjectSegmenter::setWinColor(WinColor _c)
{
  win_color_=_c;
  updateDisplayImage();
}

void GrabCut3DObjectSegmenter::updateDisplayImage()
{

  if( image_.empty() )
  {
    ROS_WARN("GrabCut3DObjectSegmenter: window image state updated with empty image");
    return;
  }

  // Generate the display image
  Scalar color = BK;
  if (win_color_ == WHITE) color = WH;
  else if (win_color_ == GRAY) color = GY;
  else if (win_color_ == GREEN) color = GR;
  else if (win_color_ == BLUE) color = BL;

  display_image_.setTo(color);
  Mat bin_mask;
  if ( !initialized_ )
    // If we haven't created a mask, just copy the base image
    image_.copyTo( display_image_ );
  else
  {
    binaryMaskFromGCMask( mask_, bin_mask );
    image_.copyTo( display_image_, bin_mask );
  }

  // Overlay the marked points, BG and FG, on the display image
  vector<Point>::const_iterator it;
  for( it = bgd_pxls_.begin(); it != bgd_pxls_.end(); ++it )
    circle( display_image_, *it, DOT_RADIUS, BL, LINE_THICKNESS );
  for( it = fgd_pxls_.begin(); it != fgd_pxls_.end(); ++it )
    circle( display_image_, *it, DOT_RADIUS, RD, LINE_THICKNESS );
  for( it = pr_bgd_pxls_.begin(); it != pr_bgd_pxls_.end(); ++it )
    circle( display_image_, *it, DOT_RADIUS, LB, LINE_THICKNESS );
  for( it = pr_fgd_pxls_.begin(); it != pr_fgd_pxls_.end(); ++it )
    circle( display_image_, *it, DOT_RADIUS, PK, LINE_THICKNESS );

  // Add the rectangle
  if( rect_state_ == IN_PROCESS || rect_state_ == SET || rect_state_==EMPTY)
    rectangle( display_image_,
        Point( rect_.x, rect_.y ),
        Point(rect_.x + rect_.width, rect_.y + rect_.height ),
        GR, 2);
}

/* Private helper method to transfer pixels from designated rectangle to
 * Grabcut mask. */
void GrabCut3DObjectSegmenter::setRectInMask()
{
    assert( !mask_.empty() );
    mask_.setTo( GC_BGD );
    rect_.x = max(0, rect_.x);
    rect_.y = max(0, rect_.y);
    rect_.width = min(rect_.width, image_.cols-rect_.x);
    rect_.height = min(rect_.height, image_.rows-rect_.y);
    (mask_(rect_)).setTo( Scalar(GC_PR_FGD) );
}

/* Private helper method to transfer labels designated in UI to mask. */
void GrabCut3DObjectSegmenter::setLblsInMask(bool control_down, bool shift_down, Point p, bool isPr )
{
    vector<Point> *bpxls, *fpxls;
    uchar bvalue, fvalue;
    if( !isPr )
    {
      bpxls = &bgd_pxls_;
      fpxls = &fgd_pxls_;
      bvalue = GC_BGD;
      fvalue = GC_FGD;
    }
    else
    {
      bpxls = &pr_bgd_pxls_;
      fpxls = &pr_fgd_pxls_;
      bvalue = GC_PR_BGD;
      fvalue = GC_PR_FGD;
    }

    if( control_down )
    {
      bpxls->push_back(p);
      circle( mask_, p, DOT_RADIUS, bvalue, LINE_THICKNESS );
    }
    if( shift_down )
    {
      fpxls->push_back(p);
      circle( mask_, p, DOT_RADIUS, fvalue, LINE_THICKNESS );
    }
}

/* Mouse callback function passed to OpenCV window. Handles marking the ROI
 * rectangle and foreground and background pixel hints. */
void GrabCut3DObjectSegmenter::mouseClick( MouseEvent event, int x, int y, bool control_down, bool shift_down)
{
    // TODO add bad args check
  switch( event )
  {
    case LEFT_BUTTON_DOWN: // set rect or GC_BGD(GC_FGD) labels
      {
        bool isb = control_down;
        bool isf = shift_down;
        if( rect_state_ == NOT_SET && !isb && !isf )
        {
          rect_state_ = IN_PROCESS;
          rect_ = Rect( x, y, 1, 1 );
        }
        if ( (isb || isf) && rect_state_ == SET )
          lbls_state_ = IN_PROCESS;
      }
      break;

    case RIGHT_BUTTON_DOWN: // set GC_PR_BGD(GC_PR_FGD) labels
      {
        bool isb = control_down;
        bool isf = shift_down;
        if ( (isb || isf) && rect_state_ == SET )
          pr_lbls_state_ = IN_PROCESS;
      }
      break;

    case LEFT_BUTTON_UP:
      if( rect_state_ == IN_PROCESS )
      {
        rect_ = Rect( Point(rect_.x, rect_.y), Point(x,y) );
        rect_state_ = SET;

	ROS_INFO_STREAM("rect area is " << rect_.area());
        if (rect_.area()==0)
	 rect_state_=EMPTY;

        setRectInMask();
        assert( bgd_pxls_.empty()
               && fgd_pxls_.empty()
               && pr_bgd_pxls_.empty()
               && pr_fgd_pxls_.empty() );
        updateDisplayImage();
      }
      if( lbls_state_ == IN_PROCESS )
      {
        setLblsInMask(control_down, shift_down, Point(x,y), false);
        lbls_state_ = SET;
        updateDisplayImage();
      }
      break;

    case RIGHT_BUTTON_UP:
      if( pr_lbls_state_ == IN_PROCESS )
      {
        setLblsInMask(control_down, shift_down, Point(x,y), true);
        pr_lbls_state_ = SET;
        updateDisplayImage();
      }
      break;

    case MOUSE_MOVE:
      if( rect_state_ == IN_PROCESS )
      {
        rect_ = Rect( Point(rect_.x, rect_.y), Point(x,y) );
        assert( bgd_pxls_.empty()
               && fgd_pxls_.empty()
               && pr_bgd_pxls_.empty()
               && pr_fgd_pxls_.empty() );
        updateDisplayImage();
      }
      else if( lbls_state_ == IN_PROCESS )
      {
        setLblsInMask(control_down, shift_down, Point(x,y), false);
        updateDisplayImage();
      }
      else if( pr_lbls_state_ == IN_PROCESS )
      {
        setLblsInMask(control_down, shift_down, Point(x,y), true);
        updateDisplayImage();
      }
      break;
  }
}

void GrabCut3DObjectSegmenter::iterCountIs(int _icnt)
{
  if (_icnt == 0) {
    iter_count_ = 0;
    return;
  }

  int iter_inc = _icnt - iter_count_;
  if (iter_inc <= 0) return;

  if( initialized() )
    grabCut3D( image_, depth_image_, mask_, rect_, bgd_model_, fgd_model_, iter_inc );
  else
  {
    if( rect_state_ != SET ) return;
	
    if (rect_.area()==0) return;
	//  rect_state_=EMPTY;

    if( lbls_state_ == SET || pr_lbls_state_ == SET )
      grabCut3D( image_, depth_image_, mask_, rect_, bgd_model_, fgd_model_,
               iter_inc, GC_INIT_WITH_MASK );
    else
      grabCut3D( image_, depth_image_, mask_, rect_, bgd_model_, fgd_model_,
               iter_inc, GC_INIT_WITH_RECT );

    initializedIs(true);
  }
  iter_count_ = _icnt;

  bgd_pxls_.clear(); fgd_pxls_.clear();
  pr_bgd_pxls_.clear(); pr_fgd_pxls_.clear();

  updateDisplayImage();

  return;
}

}

