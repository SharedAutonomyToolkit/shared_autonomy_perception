#! /usr/bin/env python

import argparse
import sys

import rospy

import rosbag
from shared_autonomy_msgs.srv import KinectAssemblyRequest, KinectAssemblyResponse, KinectAssembly


if __name__=="__main__":
    rospy.init_node("save_kinect_assembly")

     # strip out the ROS arguments and parse 
    myargs = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', dest='outfile')
    parsed_args = parser.parse_args(myargs[1:])
    filename = parsed_args.outfile

    # get the info we'll be saving to file
    # TODO: will topic remapping work in this context?
    rospy.wait_for_service('camera/assemble_kinect') 
    rospy.loginfo("got assemble_kinect service!")
    assemble_kinect = rospy.ServiceProxy('camera/assemble_kinect', KinectAssembly)
    rospy.loginfo("and, got response!")
    req = KinectAssemblyRequest()
    resp = assemble_kinect(req)

    bag = rosbag.Bag(filename, 'w')
    try:
        # TODO: this is also hardcoded in spoof_assembly.py
        bag.write('assemble_kinect_response', resp)
    finally:
        bag.close()

    rospy.loginfo("wrote to bag - exiting!")
