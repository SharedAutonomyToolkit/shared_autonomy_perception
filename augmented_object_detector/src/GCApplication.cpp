#include "augmented_object_detector/GCApplication.hpp"

using namespace std;
using namespace cv;

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
const Rect GCApplication::DEFAULT_RECT(0,0,0,0);
// This scale factor is what is actually used to calculate the default
// rectangle.
const int DEFAULT_RECT_SCALE_FACTOR = 2;

const int BGD_KEY = CV_EVENT_FLAG_CTRLKEY;
const int FGD_KEY = CV_EVENT_FLAG_SHIFTKEY;

/* NB: GrabCut pixel classes from imgproc.hpp for reference:
 * GC_BGD    = 0
 * GC_FGD    = 1
 * GC_PR_BGD = 2
 * GC_PR_FGD = 3
 */

/* Mouse callback trampoline for OpenCV window */
void on_mouse( int event, int x, int y, int flags, void* obj )
{
  static_cast<GCApplication*>(obj)->mouseClick(event, x, y, flags);
}

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

GCApplication::GCApplication(const string& _name, Mat _img)
  : win_name_(_name)
{
  // Create OpenCV window
  namedWindow( win_name_, CV_WINDOW_AUTOSIZE );

  setMouseCallback( win_name_, on_mouse, this );
  ROS_DEBUG("GCApplication: creating window");

  // Initialize window
  imageIs(_img);
  winImageStateIs(GCApplication::UPDATED);

  iter_count_ = 0;
}

GCApplication::~GCApplication()
{
  destroyWindow( win_name_ );
}

/* Mutator for initialized state of GCApp object. Client can set
 * initialized=true with no side effects; setting initialized=false clears
 * background and foreground pixel arrays and returns application state
 * variables to NOT_SET. */
// TODO: Should this be public?
void 
GCApplication::initializedIs(bool _init)
{
  if (_init)
    initialized_=_init;
  else
  {
    winImageStateIs(STALE);
    if ( !mask_.empty() ) mask_.setTo(Scalar::all(GC_BGD));
    bgd_pxls_.clear();   fgd_pxls_.clear();
    pr_bgd_pxls_.clear();  pr_fgd_pxls_.clear();
    initialized_   = _init;
    rect_state_ = NOT_SET;
    lbls_state_ = NOT_SET;
    pr_lbls_state_ = NOT_SET;
    iterCountIs(0);
    winImageStateIs(UPDATED);
  }
}

/* Mutator for GCApp stored image. */
void GCApplication::imageIs(const Mat& _image)
{
    if( _image.empty() ) {
      ROS_WARN("GCApp: imageIs called with empty image");
      return;
    }
    _image.copyTo(image_);
    mask_.create( image_.size(), CV_8UC1);
    initializedIs(false);
}

/* Mutator for GC rectangle. */
void GCApplication::rectIs(const Rect &_r)
{
  if (_r == DEFAULT_RECT) 
    rect_ = Rect(image_.cols/(DEFAULT_RECT_SCALE_FACTOR*2), 
                 image_.rows/(DEFAULT_RECT_SCALE_FACTOR*2), 
                 image_.cols/DEFAULT_RECT_SCALE_FACTOR, 
                 image_.rows/DEFAULT_RECT_SCALE_FACTOR);
  else 
    rect_=_r;

  rect_state_ = SET;
  setRectInMask();
  assert( bgd_pxls_.empty() 
  && fgd_pxls_.empty() 
  && pr_bgd_pxls_.empty() 
  && pr_fgd_pxls_.empty() );
  winImageStateIs(UPDATED);

}

/* Accessor for binary mask. */
Mat GCApplication::binaryMask()
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
void GCApplication::winColorIs(WinColor _c) 
{
  winImageStateIs(STALE);
  win_color_=_c; 
  winImageStateIs(UPDATED);
}
/* Mutator for state of image window. Client can set state=STALE with no side
 * effects. Setting state=UPDATED when state=STALE causes new image to be drawn
 * in window. */
void GCApplication::winImageStateIs(WinImageState _wis)
{
  // Set state=STALE with no side effects
  if (_wis == STALE) 
  {
    win_image_state_=_wis;
    return;
  }

  // Setting state=UPDATED when already UPDATED is a no-op.
  else if (win_image_state_ == UPDATED)
    return;

  // Otherwise setting state=UPDATED generates a new image and updates the
  // window.
  else
  {
    if( image_.empty() ) 
    {
      ROS_WARN("GCApp: window image state updated with empty image");
      return;
    }

    // Generate the display image
    Scalar color = BK;
    if (win_color_ == WHITE) color = WH;
    else if (win_color_ == GRAY) color = GY;
    else if (win_color_ == GREEN) color = GR;
    else if (win_color_ == BLUE) color = BL;

    Mat display_image(image_.size().height, image_.size().width, 
                      image_.type(), color);
    Mat bin_mask;
    if ( !initialized_ )
      // If we haven't created a mask, just copy the base image
      image_.copyTo( display_image );
    else
    {
      binaryMaskFromGCMask( mask_, bin_mask );
      image_.copyTo( display_image, bin_mask );
    }

    // Overlay the marked points, BG and FG, on the display image
    vector<Point>::const_iterator it;
    for( it = bgd_pxls_.begin(); it != bgd_pxls_.end(); ++it )
      circle( display_image, *it, DOT_RADIUS, BL, LINE_THICKNESS );
    for( it = fgd_pxls_.begin(); it != fgd_pxls_.end(); ++it )
      circle( display_image, *it, DOT_RADIUS, RD, LINE_THICKNESS );
    for( it = pr_bgd_pxls_.begin(); it != pr_bgd_pxls_.end(); ++it )
      circle( display_image, *it, DOT_RADIUS, LB, LINE_THICKNESS );
    for( it = pr_fgd_pxls_.begin(); it != pr_fgd_pxls_.end(); ++it )
      circle( display_image, *it, DOT_RADIUS, PK, LINE_THICKNESS );

    // Add the rectangle
    if( rect_state_ == IN_PROCESS || rect_state_ == SET )
      rectangle( display_image, 
          Point( rect_.x, rect_.y ), 
          Point(rect_.x + rect_.width, rect_.y + rect_.height ), 
          GR, 2);

    // Display the image
    imshow( win_name_, display_image );
  }
}

/* Private helper method to transfer pixels from designated rectangle to
 * Grabcut mask. */
void GCApplication::setRectInMask()
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
void GCApplication::setLblsInMask( int flags, Point p, bool isPr )
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

    if( flags & BGD_KEY )
    {
      bpxls->push_back(p);
      circle( mask_, p, DOT_RADIUS, bvalue, LINE_THICKNESS );
    }
    if( flags & FGD_KEY )
    {
      fpxls->push_back(p);
      circle( mask_, p, DOT_RADIUS, fvalue, LINE_THICKNESS );
    }
}

/* Mouse callback function passed to OpenCV window. Handles marking the ROI
 * rectangle and foreground and background pixel hints. */
void GCApplication::mouseClick( int event, int x, int y, int flags)
{
    // TODO add bad args check
  switch( event )
  {
    case CV_EVENT_LBUTTONDOWN: // set rect or GC_BGD(GC_FGD) labels
      {
        bool isb = (flags & BGD_KEY) != 0,
             isf = (flags & FGD_KEY) != 0;
        if( rect_state_ == NOT_SET && !isb && !isf )
        {
          rect_state_ = IN_PROCESS;
          rect_ = Rect( x, y, 1, 1 );
        }
        if ( (isb || isf) && rect_state_ == SET )
          lbls_state_ = IN_PROCESS;
      }
      break;

    case CV_EVENT_RBUTTONDOWN: // set GC_PR_BGD(GC_PR_FGD) labels
      {
        bool isb = (flags & BGD_KEY) != 0;
        bool isf = (flags & FGD_KEY) != 0;
        if ( (isb || isf) && rect_state_ == SET )
          pr_lbls_state_ = IN_PROCESS;
      }
      break;

    case CV_EVENT_LBUTTONUP:
      if( rect_state_ == IN_PROCESS )
      {
        rect_ = Rect( Point(rect_.x, rect_.y), Point(x,y) );
        rect_state_ = SET;
        setRectInMask();
        assert( bgd_pxls_.empty() 
               && fgd_pxls_.empty() 
               && pr_bgd_pxls_.empty() 
               && pr_fgd_pxls_.empty() );
        winImageStateIs(UPDATED);
      }
      if( lbls_state_ == IN_PROCESS )
      {
        setLblsInMask(flags, Point(x,y), false);
        lbls_state_ = SET;
        winImageStateIs(UPDATED);
      }
      break;

    case CV_EVENT_RBUTTONUP:
      if( pr_lbls_state_ == IN_PROCESS )
      {
        setLblsInMask(flags, Point(x,y), true);
        pr_lbls_state_ = SET;
        winImageStateIs(UPDATED);
      }
      break;
      
    case CV_EVENT_MOUSEMOVE:
      if( rect_state_ == IN_PROCESS )
      {
        rect_ = Rect( Point(rect_.x, rect_.y), Point(x,y) );
        assert( bgd_pxls_.empty() 
               && fgd_pxls_.empty() 
               && pr_bgd_pxls_.empty() 
               && pr_fgd_pxls_.empty() );
        winImageStateIs(UPDATED);
      }
      else if( lbls_state_ == IN_PROCESS )
      {
        setLblsInMask(flags, Point(x,y), false);
        winImageStateIs(UPDATED);
      }
      else if( pr_lbls_state_ == IN_PROCESS )
      {
        setLblsInMask(flags, Point(x,y), true);
        winImageStateIs(UPDATED);
      }
      break;
  }
}

void GCApplication::iterCountIs(int _icnt)
{
  if (_icnt == 0) {
    iter_count_ = 0;
    return;
  }

  int iter_inc = _icnt - iter_count_;
  if (iter_inc <= 0) return;
  
  if( initialized() )
    grabCut( image_, mask_, rect_, bgd_model_, fgd_model_, iter_inc );
  else
  {
    if( rect_state_ != SET ) return;

    if( lbls_state_ == SET || pr_lbls_state_ == SET )
      grabCut( image_, mask_, rect_, bgd_model_, fgd_model_, 
               iter_inc, GC_INIT_WITH_MASK );
    else
      grabCut( image_, mask_, rect_, bgd_model_, fgd_model_, 
               iter_inc, GC_INIT_WITH_RECT );

    initializedIs(true);
  }
  iter_count_ = _icnt;

  bgd_pxls_.clear(); fgd_pxls_.clear();
  pr_bgd_pxls_.clear(); pr_fgd_pxls_.clear();

  winImageStateIs(UPDATED);

  return;
}


