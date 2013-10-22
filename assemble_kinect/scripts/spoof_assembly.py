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
    parsed_args = parser.parse_args(myargs[1:])
    filename = parsed_args.infile

    bag = rosbag.Bag(filename)
    
    for topic,msg,t in bag.read_messages():
        if topic == "assemble_kinect_response":
            resp = msg
            break
    my_server = AssembleKinectServer(resp)
    ss = rospy.Service('camera/assemble_kinect', KinectAssembly, my_server.handle_assemble_kinect)
    rospy.spin()
        
