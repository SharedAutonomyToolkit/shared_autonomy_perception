#! /usr/bin/env python

import rospy

import actionlib
#import handle_point_cloud2 as pts

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from shared_autonomy_msgs.msg import SegmentGoal, SegmentAction
from shared_autonomy_msgs.srv import KinectAssembly


# This code needs to:
# * obtain a set of data from the kinect_assembler
# * repackage it into the appropriate format
# * call the actionlib server that actually does segmentation
# * use the resulting mask to get desired image coords; use the point_cloud2 package to 

class RunSegmentation():

    def __init__(self):
        self.kinect_client = rospy.ServiceProxy('assemble_kinect', KinectAssembly)
        self.segment_client = actionlib.SimpleActionClient('/ben_segmentation_node', SegmentAction)
        self.segment_client.wait_for_server()

    def get_data(self):
        resp = self.kinect_client()
        return resp

    def get_segmentation(self, data):
    
        goal = SegmentGoal()
        goal.image = data.image
        goal.depth = data.depth

        # this callback will set the member variable mask
        self.segment_client.send_goal(goal, done_cb=self.segmentDoneCB, 
                                      feedback_cb=self.segmentFeedbackCB)
        self.segment_client.wait_for_result()
        
        return self.mask

    def publish_points(self, data, mask):
        print "I should be publishing points!"

    def segmentDoneCB(self, state, result):
        print "segmentation done"
        self.mask = result.mask
        
    def segmentFeedbackCB(self, feedback):
        print "feedback received"
        print feedback
    


if __name__ == "__main__":
    rospy.init_node('run_segmentation')

    mysegmenter = RunSegmentation()
    data = mysegmenter.get_data()
    mask = mysegmenter.get_segmentation(data)
    mysegmenter.publish_points(data, mask)
