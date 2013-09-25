shared_autonomy
===============

For now, lots of random pieces that I've been experimenting with. They'll eventually come together into the pipeline ...

* ben_segmentation - c++ node that attempts to reimplement ben's code, but isolated from the GUI stuff

* im_hmi - Python node that implements both the interactive marker server and an actionlib server that handles requests for obtaining a bounding box

* shared_autonomy_msgs - isolating all of the actionlib messages
  * BoundingBox - given an image, receives {minx, maxx, miny, maxy}
  * Segment - given RGB and D images, returns mask of object

* webtools
  * im_gui.html - page to show interactive markers produced by im_hmi
  * ros_testing.html - page to demonstrate subscribing/publishing to ros topics + creating html buttons/fields

* clear_table 
  * clear_table.py - smach state machine that'll eventually have setup->segment->getGrasps->pickUp->drop. For now, it transitions states on human keyboard input, uses simple_robot_control to move torso/head/arms (commented out for testing on laptop), and the starts of an actionlib client for getting segmentations.
  * run_segmentation.py - standalone script that handles actionlib stuff for running segmentation but nothing else. 

* random_snippets:
  * simple_robot_control.ipynb - examples of how to interact with all the simple_robot_control commands
  * ImageServer.ipynb - probably a dead end, just playing with requesting images in python and rgbd_assembler

