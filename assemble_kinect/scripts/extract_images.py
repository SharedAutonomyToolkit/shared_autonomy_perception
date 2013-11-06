#! /usr/bin/env python

import argparse
import os
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

    outfile_dir = re.sub('.bag', '', infile)
    try:
        os.mkdir(outfile_dir)
    except OSError:
        # if directory already exists, that's fine. 
        pass

    tokens = outfile_dir.split('/')
    outfile_base = outfile_dir + '/' + tokens[-1]
    msg_count = 0
    
    bag = rosbag.Bag(infile)
    for topic, msg, t in bag.read_messages():
        if topic == "assemble_kinect_response":
            outfile = outfile_base + str(msg_count) + '.pgm'
            msg_count = msg_count + 1
            resp = msg
            cv_image = cv_bridge.CvBridge().imgmsg_to_cv(resp.image, "bgr8")
            cv.SaveImage(outfile, cv_image)


