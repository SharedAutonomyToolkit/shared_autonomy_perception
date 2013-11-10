#! /usr/bin/env python

# This node periodically querries the assemble_kinect service for data, and saves
# the resulting message to a bagfile. Note that the rate limiting is not exact: it
# specifies a *maximum* rate, and sleeps for 1/r. Even without any sleeping, 
# this will miss some of the messages. 
# Usage: ./save_assembly.py -f [filename] -r [rate]

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
    parser.add_argument('-r', dest='rate')
    parsed_args = parser.parse_args(myargs[1:])
    filename = parsed_args.outfile
    rate = float(parsed_args.rate)

    # get the info we'll be saving to file
    # TODO: will topic remapping work in this context?
    rospy.wait_for_service('camera/assemble_kinect') 
    rospy.loginfo("got assemble_kinect service!")
    assemble_kinect = rospy.ServiceProxy('camera/assemble_kinect', KinectAssembly)
    req = KinectAssemblyRequest()

    bag = rosbag.Bag(filename, 'w')

    rr = rospy.Rate(rate)
    prev_seq = -1
    while not rospy.is_shutdown():
        resp = assemble_kinect(req)
        if resp.header.seq != prev_seq:
            bag.write('assemble_kinect_response', resp)
            rospy.loginfo('wrote message %d', resp.header.seq)
            prev_seq = resp.header.seq
        else:
            rospy.loginfo('repeat of message %d', prev_seq)
        rr.sleep()
        
    bag.close()

