#! /usr/bin/env python

import rospy
import actionlib

import argparse
import sys


import rosbag
from sensor_msgs.msg import PointCloud2, Image

from shared_autonomy_msgs.srv import KinectAssemblyRequest, KinectAssemblyResponse, KinectAssembly
from shared_autonomy_msgs.msg import BoundingBoxGoal, BoundingBoxAction, EditPixelGoal, EditPixelAction
from shared_autonomy_msgs.msg import SegmentGoal, SegmentAction

def bbox_client(input_image):

    client = actionlib.SimpleActionClient('get_bounding_box', BoundingBoxAction)
    rospy.loginfo('constructed simple action client')

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    rospy.loginfo('got server!')

    # tests preemption by "cancel_all_goals"
    # TODO: right now, it's up to the server to figure out how to apply the mask ... 
    # definitely doesn't work in the current implementation, not sure how it'll work 
    # if Sarah gets her image display stuff working ... 
    goal = BoundingBoxGoal(image=input_image)
    client.send_goal(goal)
    rospy.loginfo('sent first goal')
    client.wait_for_result()
    rospy.loginfo('and, got result!')
    print client.get_state()
    print client.get_result()

def edit_client(input_image):

    client = actionlib.SimpleActionClient('edit_pixel_labels',EditPixelAction)
    rospy.loginfo('constructed simple action client')

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    rospy.loginfo('got server!')

    # tests preemption by "cancel_all_goals"
    goal = EditPixelGoal(image=input_image, mask=input_image)
    client.send_goal(goal)
    rospy.loginfo('sent first goal')
    client.wait_for_result()
    rospy.loginfo('and, got result!')
    print client.get_state()
    print client.get_result()

def segment_client(input_image, input_depth):

    client = actionlib.SimpleActionClient('segment_service', SegmentAction)
    rospy.loginfo('constructed simple action client')

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    rospy.loginfo('got server!')

    # tests preemption by "cancel_all_goals"
    goal = SegmentGoal(image=input_image, depth=input_depth)
    client.send_goal(goal)
    rospy.loginfo('sent first goal')
    client.wait_for_result()
    rospy.loginfo('and, got result!')
    print client.get_state()

if __name__=="__main__":

    rospy.init_node("test_bbox_server")
    
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
        segment_client(resp.image, resp.depth)
        #edit_client(resp.image)
        #bbox_client(resp.image)

