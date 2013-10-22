#! /usr/bin/env python

import rospy

import rosbag
from shared_autonomy_msgs.srv import KinectAssemblyRequest, KinectAssemblyResponse, KinectAssembly


if __name__=="__main__":
    rospy.init_node("save_kinect_assembly")

    # get the info we'll be saving to file
    # TODO: will topic remapping work in this context?
    rospy.wait_for_service('camera/assemble_kinect') 
    rospy.loginfo("got assemble_kinect service!")
    assemble_kinect = rospy.ServiceProxy('camera/assemble_kinect', KinectAssembly)
    rospy.loginfo("and, got response!")
    req = KinectAssemblyRequest()
    resp = assemble_kinect(req)

    # TODO: make this a REQUIRED parameter!
    filename = "/home/lil1pal/table_logs/foo.bag"
    bag = rosbag.Bag(filename, 'w')
    try:
        # TODO: this is also hardcoded in spoof_assembly.py
        bag.write('assemble_kinect_response', resp)
    finally:
        bag.close()

    rospy.loginfo("wrote to bag - exiting!")
