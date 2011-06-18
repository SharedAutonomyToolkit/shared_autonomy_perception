#ifndef GC3DAPPLICATION_H
#define GC3DAPPLICATION_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <ros/ros.h>

using namespace std;
using namespace cv;

class GC3DApplication
{
public:
    /* States of grabbing/marking action */
    enum GrabState { NOT_SET, IN_PROCESS, SET };

    /* States of OpenCV window -- updated if the window has been redisplayed
     * since the last change to the image, stale otherwise. */
    enum WinImageState { UPDATED, STALE };

    /* Allowed background colors for OpenCV window */
    enum WinColor { BLACK, GRAY, WHITE, GREEN, BLUE };

    /* Marker for default rectangle. */
    static const Rect DEFAULT_RECT;

    /* Constructor/destructor */
    GC3DApplication(const string& _name, const Mat& _img, const Mat& _depth_image);
    ~GC3DApplication();

    /* Accessors for application state */
    inline bool initialized() { return initialized_; }
    void initializedIs(bool);

    /* Accessors/mutators for base image and mask */
    inline Mat image() { return image_; }
    void imageIs(const Mat&, const Mat&);
    inline Mat mask() { return mask_; }
    Mat binaryMask();

    /* Mutator for GC rectangle. */
    void rectIs(const Rect&);

    /* Accessors/mutators for OpenCV window attributes */
    inline string winName() { return win_name_; }
    inline WinImageState winImageState() { return win_image_state_; }
    void winImageStateIs(WinImageState);
    inline WinColor winColor() { return win_color_; }
    void winColorIs(WinColor _c);

    /* Accessor for state variables */
    inline GrabState rectState() { return rect_state_; }

    /* Mouse callback for image topic subscription */
    void mouseClick( int event, int x, int y, int flags);

    /* Iteration control accessors */
    inline int iterCount() const { return iter_count_; }
    void iterCountIs(int _icnt);
    inline void iterCountInc() { iterCountIs(iterCount()+1); }
    
private:
    /* Transfer marked rectangle to mask image */
    void setRectInMask();
    /* Transfer marked labels to mask image */
    void setLblsInMask( int flags, Point p, bool isPr );

    /* Current state of OpenCV window image */
    WinImageState win_image_state_;
    WinColor win_color_;
    
    /* Segmentation process data */
    string win_name_;           // Name of OpenCV window
    Mat image_;                 // image to be segmented
    Mat depth_image_;                 // depth image to be segmented
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


#endif
