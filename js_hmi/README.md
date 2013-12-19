grabcutsebmentationlib.js 
================

### Usage

For now, this depends on modifications to roslibjs, and requires that the Bosch fork of roslibjs be downloaded and installed in the same location as shared_autonomy_perception.

To run:

You must have downloaded rosbridge_server and mjpeg_server

1) rosrun rosbridge_server rosbridge_websocket
2) in web browser, open: pages/interactive_segmentation_interfaces.html
3) ./client.py -f /path/to/bag/file.bag -i 1
(where the bagfile needs to contain KinectAssembly responses)

Note: this assumes that the computer that is serving the page is the computer that you are on.  If this is not the case you will need to edit pages/interactive_segmentation_interfaces.html to have the properhost name (look for localhost and replace with your computer name)

ROS API:
* /head_mount_kinect/rgb/image_color (sensor_msgs/Image) - attempts to display this as the background when nothing else is going on
* /get_bounding_box (shared_autonomy_msgs/BoundingBox.action) - provides this action server
* /edit_pixel_labels (shared_autonomy_msgs/EditPixel.action) - provides this action server

Open Issues:
* bbox/edit windows need to have titles 
* bbox/edit windows need to default to be taller
* refactor to make Segmenter cleaner, push stuff onto bbox/editor (+ rename segmenter to be less grabcut-specific)
* actually serve the page
* need to 
* mjpeg_server not working for streaming from kinect


Open your web browser: localhost:8000/pages/interactive_segmentation_interfaces.html


### Dependencies
robot2020lib.js depends on:

[EventEmitter2](https://github.com/hij1nx/EventEmitter2). The current supported version is 0.4.11. The current supported version can be found on the Robot Web Tools CDN: ([full](http://cdn.robotwebtools.org/EventEmitter2/0.4.11/eventemitter2.js)) | ([min](http://cdn.robotwebtools.org/EventEmitter2/0.4.11/eventemitter2.min.js))

[roslibjs](https://github.com/RobotWebTools/roslibjs). The current supported version is r5. The current supported version can be found on the Robot Web Tools CDN: ([full](http://cdn.robotwebtools.org/roslibjs/r5/roslib.js)) | ([min](http://cdn.robotwebtools.org/roslibjs/r5/roslib.min.js)).

[jquery](http://code.jquery.com). The current supported version is r5. The current supported version can be found on the jquery CDN: ([full](http://code.jquery.com/jquery-1.10.2.js)) | ([min](http://code.jquery.com/jquery-migrate-1.2.1.min.js)).

### Build
Checkout [utils/README.md](utils/README.md) for details on building.

### Authors
See the [AUTHORS.md](AUTHORS) file for a full list of contributors.

