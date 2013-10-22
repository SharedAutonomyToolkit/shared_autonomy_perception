#! /usr/bin/env python

import argparse
import re
import sys


import rospy

import cv
import cv_bridge


import rosbag
from shared_autonomy_msgs.srv import KinectAssemblyResponse


if __name__=="__main__":
    rospy.init_node("extract_image")
    
    # strip out the ROS arguments and parse 
    myargs = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', dest='infile')
    parsed_args = parser.parse_args(myargs[1:])
    infile = parsed_args.infile

    outfile = re.sub('.bag', '.pgm', infile)

    bag = rosbag.Bag(infile)
    for topic, msg, t in bag.read_messages():
        if topic == "assemble_kinect_response":
            resp = msg
            break
    
    cv_image = cv_bridge.CvBridge().imgmsg_to_cv(resp.image, "bgr8")
    cv.SaveImage(outfile, cv_image)

    print infile, outfile

