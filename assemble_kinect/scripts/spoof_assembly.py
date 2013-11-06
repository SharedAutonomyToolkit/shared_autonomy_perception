#! /usr/bin/env python

import argparse
import sys

import rospy

import rosbag
from shared_autonomy_msgs.srv import KinectAssemblyRequest, KinectAssemblyResponse, KinectAssembly


class AssembleKinectServer():
    def __init__(self, resp):
        self.resp = resp
    def handle_assemble_kinect(self, req):
        return self.resp

if __name__=="__main__":

    rospy.init_node("spoof_kinect_asssembly")
    
     # strip out the ROS arguments and parse 
    myargs = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', dest='infile')
    parser.add_argument('-i', dest='index')
    parsed_args = parser.parse_args(myargs[1:])
    filename = parsed_args.infile
    index = int(parsed_args.index)

    bag = rosbag.Bag(filename)
    
    # find the index-th image in the bag
    msg_count = 0
    resp = None
    for topic,msg,t in bag.read_messages():
        if topic == "assemble_kinect_response":
            if msg_count == index:
                resp = msg
                break
            # needs to be here, b/c images are generated starting w/ 0
            msg_count = msg_count + 1

    if resp is None:
        rospy.logerr('spoof_assembly failed. Requested message %d of %d.', index, msg_count)
    else:
        my_server = AssembleKinectServer(resp)
        ss = rospy.Service('camera/assemble_kinect', KinectAssembly, my_server.handle_assemble_kinect)
        rospy.spin()
        
