#! /usr/bin/env python



import actionlib
from cv_bridge import CvBridge
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rospy

from shared_autonomy_msgs.msg import BoundingBoxAction, BoundingBoxResult

# the classes that actually handle the different actionlib request types
from bounding_box import BoundingBox

# The role of this file is to match the appropriate IM-handling class to the
# actionlib request received. 
class IM_HMI():
    def __init__(self):
        # create bounding_box interactive markers ...
        # create actionlib server for obtaining feedback 

        # TODO: make topic a parameter
        self.bb_server = actionlib.SimpleActionServer('/get_bounding_box', 
                                                      BoundingBoxAction, 
                                                      execute_cb=self.execute_bounding_box,
                                                      auto_start=False)
        self.bb_server.start()
        self.bb_active = False

        self.im_server = InteractiveMarkerServer("im_gui")
        self.cv_bridge = CvBridge()

    def bb_done_callback(self):
        self.bb_active = False

    def execute_bounding_box(self, goal):
        print "execute_bounding_box called"

        # convert goal image to opencv
        cv_im = self.cv_bridge.imgmsg_to_cv(goal.image)

        self.bb_active = True
        mybb = BoundingBox(self.bb_done_callback, self.im_server, cv_im)

        rr = rospy.Rate(10)
        while (self.bb_active and (not self.bb_server.is_preempt_requested()) and 
               (not rospy.is_shutdown())):
            rr.sleep()

        print "im_hmi/execute_bounding_box - done looping"
        print "bb_active: ", self.bb_active
        print "ros shutdown: ", rospy.is_shutdown()
        print "preempt req: ", self.bb_server.is_preempt_requested()

        if self.bb_server.is_preempt_requested():
            mybb.cancel()
            self.bb_server.set_preempted()
        else:
            result = mybb.get_result()
            if result is None:
                self.bb_server.set_aborted()
            else:
                (row1, row2, col1, col2) = result
                resp = BoundingBoxResult()
                resp.min_row.data = row1
                resp.max_row.data = row2
                resp.min_col.data = col1
                resp.max_col.data = col2
                self.bb_server.set_succeeded(resp)
        
if __name__ == "__main__":
    rospy.init_node("im_hmi")
    my_hmi = IM_HMI()
    print "im_hmi initialized"
    rospy.spin()
    

