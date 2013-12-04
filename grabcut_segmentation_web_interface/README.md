robot2020lib.js 
================

### Usage
Pre-built files can be found in [robot2020lib.js](build/robot2020lib.js) or [robot2020lib.min.js](build/robot2020lib.min.js).

This is a work in progress...not a final version.

To run an example:

You must have downloaded rosbridge_server and mjpeg_server

1) rosrun rosbridge_server rosbridge_websocket
2) rosrun mjpeg_server mjpeg_server
3) in the directory of this package run: python -m SimpleHTTPServer

Note: this assumes that the computer that is serving the page is the computer that you are on.  If this is not the case you will need to edit pages/interactive_segmentation_interfaces.html to have the properhost name (look for localhost and replace with your computer name)

Additionally this assumes that images are being published on the /kinect_image topic.  (this should eventually be changed)

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

