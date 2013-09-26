#! /usr/bin/env python

from math import cos
from math import pi

import roslib; roslib.load_manifest("interactive_markers")

import rospy
import tf

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from std_msgs.msg import Empty


class bounding_box():
    def __init__(self):
        self.server = InteractiveMarkerServer("im_gui")

        # the box that'll be drawn to show the ROI
        #self.roi_server = InteractiveMarkerServer("im_gui/roi")
        # top-left and bottom-right corners of ROI
        #self.tl_server = InteractiveMarkerServer("im_gui/tl")
        #self.br_server = InteractiveMarkerServer("im_gui/br")
    
        # starting coordinates of ROI
        self.x1 = 0.0
        self.x2 = 1.0 # needs to be set as float here ... 
        self.y1 = 1.0
        self.y2 = 0.0

        self.menu_handler = MenuHandler()
        only_entry = self.menu_handler.insert("Accept ROI", callback=self.acceptCB)

        # creating marker & control for roi; class var b/c other callbacks modify it
        self.roi = InteractiveMarker()
        self.roi.header.frame_id = "/base_link"
        self.roi.name = "ROI"
        self.roi.description = "Selected ROI of image"
        self.roi.pose.position.x = (self.x1 + self.x2) / 2
        self.roi.pose.position.y = (self.y1 + self.y2) / 2
        # Create marker to display ROI in webtools/rviz
        roi_marker = Marker()
        roi_marker.type = Marker.CUBE
        roi_marker.scale.x = self.x2 - self.x1
        roi_marker.scale.y = self.y2 - self.y1
        roi_marker.scale.z = 0.5
        roi_marker.color.r = 0.0
        roi_marker.color.g = 0.5
        roi_marker.color.b = 0.5
        roi_marker.color.a = 0.5
        roi_control = InteractiveMarkerControl()
        roi_control.interaction_mode = InteractiveMarkerControl.BUTTON
        roi_control.always_visible = True
        roi_control.markers.append(roi_marker)
        self.roi.controls.append(roi_control)
        self.server.insert(self.roi)
        self.menu_handler.apply(self.server, "ROI")
        
        # creating marker & control for corners
        tl = InteractiveMarker()
        tl.header.frame_id = "/base_link"
        tl.name = "tl"
        tl.description = "top left corner of ROI"
        tl.pose.position.x = self.x1
        tl.pose.position.y = self.y1
        # marker s.t. we can have an initial pose
        tl_marker = Marker()
        tl_marker.type = Marker.SPHERE
        tl_marker.scale.x = 0.5
        tl_marker.scale.y = 0.5
        tl_marker.scale.z = 0.5
        tl_marker.color.r = 1.0
        tl_marker.color.g = 0.0
        tl_marker.color.b = 0.0
        tl_marker.color.g = 1.0

        tl_control = InteractiveMarkerControl()
        tl_control.name = "tl_corner"
        qq = tf.transformations.quaternion_from_euler(0, pi/2, 0)
        tl_control.orientation.x = qq[0]
        tl_control.orientation.y = qq[1]
        tl_control.orientation.z = qq[2]
        tl_control.orientation.w = qq[3]

        tl_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        #tl_control.markers.append(tl_marker)
        tl.controls.append(tl_control)

        self.server.insert(tl, self.processTL)

        br = InteractiveMarker()
        br.header.frame_id = "/base_link"
        br.name = "br"
        br.description = "bottom right corner of ROI"
        br.pose.position.x = self.x2
        br.pose.position.y = self.y2
        br_control = InteractiveMarkerControl()
        br_control.name = "br_corner"
        qq = tf.transformations.quaternion_from_euler(0, pi/2, 0)
        br_control.orientation.x = qq[0]
        br_control.orientation.y = qq[1]
        br_control.orientation.z = qq[2]
        br_control.orientation.w = qq[3]
        br_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        br.controls.append(br_control)
        self.server.insert(br, self.processBR)


        # separate menu handler for now - will merge with ROI eventually
        menu = InteractiveMarker()
        menu.header.frame_id = "/base_link"
        menu.pose.position.y = -3.0
        menu.scale = 1
        menu.name = "acceptROI"

        menu_marker = Marker()
        menu_marker.type = Marker.CUBE
        menu_marker.scale.x = 0.5
        menu_marker.scale.y = 0.5
        menu_marker.scale.z = 0.5
        menu_marker.color.r = 0.5
        menu_marker.color.g = 0.0
        menu_marker.color.b = 0.0
        menu_marker.color.a = 1.0

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        menu_control.markers.append(menu_marker)
        menu.controls.append(menu_control)
        #self.server.insert(menu)
        #self.menu_handler.apply(self.server, "acceptROI")

        self.server.applyChanges()

    def acceptCB(self, feedback):
        # This would be sloppy, but since there's only one menu item,
        # can assume that it means return
        print feedback

    def updateROI(self):
        self.roi.pose.position.x = (self.x1 + self.x2)/2
        self.roi.pose.position.y = (self.y1 + self.y2)/2
        scale = self.roi.controls[0].markers[0].scale
        scale.x = self.x2 - self.x1
        scale.y = self.y2 - self.y1
        self.server.insert(self.roi)
        self.server.applyChanges()


    # TODO: figure out how to get this to update the position of the ROI marker ... 
    # TODO: should only need one callback, and can switch on the originating marker topic?
    def processTL(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            #raise Exception("WTF? this marker should only be able to generate pose updates")
            return
        pp = feedback.pose.position
        self.x1 = pp.x
        self.y1 = pp.y
        self.updateROI()

    def processBR(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            #raise Exception("WTF? this marker should only be able to generate pose updates")
            return

        pp = feedback.pose.position
        self.x2 = pp.x
        self.y2 = pp.y
        self.updateROI()
                            
if __name__=="__main__":
    rospy.init_node('im_gui')
    bb = bounding_box()

    rospy.spin()

        
