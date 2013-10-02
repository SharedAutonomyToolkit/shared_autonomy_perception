shared_autonomy_perception
===============

For now, lots of random pieces that I've been experimenting with. 
They'll eventually come together into the pipeline ... at this point, 
the segmentation is a simple pass-through, labeling all points within 
the bounding box as foreground. 

This pipeline depends on the catkenized version of smach. To obtain that:
  * cd ~/catkin_ws/src
  * git clone https://github.com/ros/executive_smach.git executive_smach
  * cd executive_smach
  * git checkout groovy-devel
  
To run the current setup:

1) roscore

2) roscd shared_autonomy_launch/launch

3) roslaunch run_segmentation.launch

4) rosrun rviz rviz; set it up with:
  * Fixed Frame /camera_link
  * PointCloud2 /camera/depth_registered/points 
  * PointCloud2 /segmented_points
  * InteractiveMarkers /im_gui/update

5) in rviz:
  * drag corners of the interactive marker to enclose the object you want to segment
  * right click on IM -> accept ROI 
  * check that the points published on /segmented_points were all of the 3d points inside the bounding box
  
================

Module organization:

* assemble_kinect - 
  * Listens to data coming from the kinect and packaging it up for segmentation tasks. 
  * I feel like somebody has got to have done this before - rgbd_assembler is pretty close, but I don't want to depend on the whole manipulation pipeline! 
  * Written in C++ b/c python doesn't appear to support approximate synchronization, although this looks promising:
https://github.com/ros-perception/image_pipeline/blob/groovy-devel/camera_calibration/src/camera_calibration/approxsync.py 


* grabcut3d_segmentation - c++ node that attempts to reimplement ben's code, but isolated from the GUI stuff

* im_hmi - Python node that implements both the interactive marker server and an actionlib server that handles requests for obtaining a bounding box

* shared_autonomy_msgs - isolating all of the messages we use for shared autonomy
  * actions: 
    - BoundingBox - given an image, receives {minx, maxx, miny, maxy}
    - EditPixel - given an image and a mask, returns list of (mislabeled) foreground and background pixels
    - Segment - given RGB and D images, returns mask of object
  * messages:
    - Pixel - I couldn't find this anywhere else ... seems like it should exist?
  * services:
    - KinectAssembly - returns associated depth/rgb data

* clear_table 
  * run_segmentation.py - standalone script that handles actionlib stuff for running segmentation but nothing else. 
  * clear_table.py (not yet finished!) - smach state machine that'll eventually have setup->segment->getGrasps->pickUp->drop. For now, it transitions states on human keyboard input, uses simple_robot_control to move torso/head/arms (commented out for testing on laptop), and the starts of an actionlib client for getting segmentations.

* webtools (Not yet fully integrated!)
  * im_gui.html - page to show interactive markers produced by im_hmi
  * ros_testing.html - page to demonstrate subscribing/publishing to ros topics + creating html buttons/fields

* random_snippets:
  * simple_robot_control.ipynb - examples of how to interact with all the simple_robot_control commands
  * ImageServer.ipynb - probably a dead end, just playing with requesting images in python and rgbd_assembler
  * testing_conversions.ipynb - looks at using the (at this point, absent) read_points method from sensor_msgs.point_cloud2 to look at and plot points in python. However, the float->RGB has not yet been implemented, so I probably can't use this one. Also plays with assemble messages, but I discovered that the rgb image is out of sync with the depth one, so I need a message filter that doesn't yet exist in python. 
