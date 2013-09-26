#! /usr/bin/env python

import rospy
import actionlib
from shared_autonomy_msgs.msg import BoundingBoxAction, BoundingBoxResult

class IM_HMI():
    def __init__(self):
        # create bounding_box interactive markers ...
        # create actionlib server for obtaining feedback 

        # TODO: make topic a parameter
        self.bb_server = actionlib.SimpleActionServer('/get_bounding_box', 
                                               BoundingBoxAction, 
                                               execute_cb=self.execute_bounding_box)
        
    def execute_bounding_box(self, goal):
        # TODO: this is where the interaction marker stuff needs to be called
        # for now, I'm leaving out all the preemption/etc
        resp = BoundingBoxResult()        
        resp.min_row.data = goal.image.height/4
        resp.max_row.data = 3*goal.image.height/4
        resp.min_col.data = goal.image.width/4
        resp.max_col.data = 3*goal.image.width/4
        self.bb_server.set_succeeded(resp)
        
if __name__ == "__main__":
    rospy.init_node("im_hmi")
    my_hmi = IM_HMI()
    rospy.spin()
    

