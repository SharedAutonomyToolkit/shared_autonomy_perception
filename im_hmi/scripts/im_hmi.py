#! /usr/bin/env python

import actionlib

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rospy

from shared_autonomy_msgs.msg import BoundingBoxAction, BoundingBoxResult, EditPixelAction, EditPixelResult

# the classes that actually handle the different actionlib request types
from bounding_box import BoundingBox
from edit_pixel_labels import EditPixelLabels

# The role of this file is to match the appropriate IM-handling class to the
# actionlib request received.
class IM_HMI():
    def __init__(self):
        # TODO: make topic a parameter
        self.bb_server = actionlib.SimpleActionServer('bbox_service', 
                                                      BoundingBoxAction, 
                                                      execute_cb=self.execute_bounding_box,
                                                      auto_start=False)
        self.bb_server.start()
        self.bb_active = False


        self.label_server = actionlib.SimpleActionServer('pixel_service',
                                                         EditPixelAction,
                                                         execute_cb = self.execute_label_pixel,
                                                         auto_start=False)
        self.label_server.start()
        self.label_active = False

        self.im_server = InteractiveMarkerServer("im_gui")

        
    def label_done_callback(self):
        self.label_active = False

    # TODO: There's a lot of overlap between this and execute_bounding_box ...
    #       any way to get rid of it?
    def execute_label_pixel(self, goal):
        rospy.loginfo("execute_label_pixel called")

        self.label_active = True
        mylabeller = EditPixelLabels(self.label_done_callback, self.im_server, 
                                     goal.image, goal.mask)

        rr = rospy.Rate(10)
        while (self.label_active and 
               (not self.label_server.is_preempt_requested()) and
               (not rospy.is_shutdown())):
            rr.sleep()

        if self.label_server.is_preempt_requested():
            mylabeller.cancel()
            self.label_server.set_preempted()
        elif rospy.is_shutdown():
            self.label_server.set_aborted()
        else:
            rospy.loginfo("foreground: %r" % mylabeller.get_foreground())
            rospy.loginfo("background: %r" % mylabeller.get_background())
            resp = EditPixelResult()
            resp.fg = mylabeller.get_foreground()
            resp.bg = mylabeller.get_background()
            self.label_server.set_succeeded(resp)
        rospy.loginfo("returning from execute_label_pixel")


    def bb_done_callback(self):
        self.bb_active = False

    def execute_bounding_box(self, goal):
        rospy.loginfo("execute_bounding_box called")

        self.bb_active = True
        mybb = BoundingBox(self.bb_done_callback, self.im_server, goal.image)

        rr = rospy.Rate(10)
        while (self.bb_active and (not self.bb_server.is_preempt_requested()) and 
               (not rospy.is_shutdown())):
            rr.sleep()

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
        rospy.loginfo("exiting execute_bounding_box")
        
if __name__ == "__main__":
    rospy.init_node("im_hmi")
    my_hmi = IM_HMI()
    rospy.loginfo("im_hmi initialized")
    rospy.spin()
