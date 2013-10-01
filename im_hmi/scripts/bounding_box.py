from math import pi

from geometry_msgs.msg import Point
import im_utils
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *        
from std_msgs.msg import ColorRGBA
import tf

class BoundingBox():
    def __init__(self, hmi_callback, im_server, image):
        self.hmi_callback = hmi_callback
        self.im_server = im_server
        self.image = image

        # starting coordinates of ROI
        self.x1 = 1.0
        self.x2 = 2.0 # needs to be set as float here ... 
        self.y1 = 1.0
        self.y2 = 2.0
        
        # pixels-per-m in published markers
        self.ppm = 100

        # used to store the result once it's been computed
        self.result = None

        self.add_image()
        self.add_bounding_box()


    def get_result(self):
        return self.result

    # used to set the im_server back to initial state ... 
    def cancel(self):
        self.im_server.clear()
        self.im_server.applyChanges()


    def add_image(self):
        rospy.loginfo("add image called!")

        image_marker = Marker()
        image_marker.type = image_marker.POINTS
        image_marker.scale.x = 0.05
        image_marker.scale.y = 0.05
        image_marker.scale.z = 0.05

        for jj in xrange(0, self.image.cols, 3):
            for ii in xrange(0, self.image.rows, 3):
                # sinking it a bit s.t. the Interactive Markers are easier to grab
                (xx, yy) = im_utils.metersFromPixels(ii, jj, self.ppm, self.image.rows, self.image.cols)
                pt = Point(xx, yy, -0.05)

                image_marker.points.append(pt)
                # TODO: does this conversion belong in im_utils?
                # ROS is rgba, opencv is bgr
                cc = ColorRGBA(1.0-self.image[ii,jj][2], 1.0-self.image[ii,jj][1], 1.0-self.image[ii,jj][0], 1.0)
                image_marker.colors.append(cc)

        image_control = InteractiveMarkerControl()
        image_control.always_visible = True
        image_control.markers.append(image_marker)

        image_im = InteractiveMarker()
        image_im.header.frame_id = "camera_link"
        image_im.name = "CameraIM"
        image_im.description = ""
        image_im.controls.append(image_control)
        self.im_server.insert(image_im)
        self.im_server.applyChanges()

    def add_bounding_box(self):
        rospy.loginfo("add_bounding_box called!")

        menu_handler = MenuHandler()
        only_entry = menu_handler.insert("Accept ROI", callback=self.acceptCB)

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

        roi_control_button = InteractiveMarkerControl()
        roi_control_button.interaction_mode = InteractiveMarkerControl.BUTTON
        roi_control_button.always_visible = True
        roi_control_button.markers.append(roi_marker)


        # creating marker & control for roi; class var b/c other callbacks modify it
        self.roi_im = InteractiveMarker()
        self.roi_im.header.frame_id = "/camera_link"
        self.roi_im.name = "ROI"
        self.roi_im.description = ""
        self.roi_im.pose.position.x = (self.x1 + self.x2) / 2
        self.roi_im.pose.position.y = (self.y1 + self.y2) / 2
        self.roi_im.controls.append(roi_control_button)
        #self.im_server.insert(self.roi_im)

        
        # creating marker & control for corners
        # TODO: I think that we could have all these controls attached to the same IM
        # However, I'm not sure how to get them to start offset from the IM's frame ... 
        #tl_marker = Marker()
        #tl_marker.type = Marker.CUBE
        #tl_marker.scale.x = 1.01
        #tl_marker.scale.y = 1.01
        #tl_marker.scale.z = 1.01
        #tl_marker.color.r = 0.5
        #tl_marker.color.g = 0.5
        #tl_marker.color.b = 0.5
        #tl_marker.color.a = 0.5
        #tl_marker.pose.position.x = self.x1 
        #tl_marker.pose.position.y = self.y1


        tl_control = InteractiveMarkerControl()
        tl_control.name = "tl_corner"
        qq = tf.transformations.quaternion_from_euler(0, pi/2, 0)
        tl_control.orientation.x = qq[0]
        tl_control.orientation.y = qq[1]
        tl_control.orientation.z = qq[2]
        tl_control.orientation.w = qq[3]
        tl_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        tl_control.always_visible = True
        #tl_control.markers.append(tl_marker)
        #self.roi_im.controls.append(tl_control)

        tl_im = InteractiveMarker()
        tl_im.header.frame_id = "/camera_link"
        tl_im.name = "tl"
        tl_im.description = ""
        tl_im.pose.position.x = self.x1
        tl_im.pose.position.y = self.y1
        tl_im.controls.append(tl_control)

        self.im_server.insert(tl_im, self.processTL)
        self.im_server.insert(self.roi_im, self.process_feedback)

        # And, for bottom-right corner
        br_control = InteractiveMarkerControl()
        br_control.name = "br_corner"
        qq = tf.transformations.quaternion_from_euler(0, pi/2, 0)
        br_control.orientation.x = qq[0]
        br_control.orientation.y = qq[1]
        br_control.orientation.z = qq[2]
        br_control.orientation.w = qq[3]
        br_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

        br_im = InteractiveMarker()
        br_im.header.frame_id = "/camera_link"
        br_im.name = "br"
        br_im.description = ""
        br_im.pose.position.x = self.x2
        br_im.pose.position.y = self.y2
        br_im.controls.append(br_control)
        self.im_server.insert(br_im, self.processBR)

        # separate menu handler for now - will merge with ROI eventually
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

        menu_im = InteractiveMarker()
        menu_im.header.frame_id = "/camera_link"
        menu_im.pose.position.y = -3.0
        menu_im.scale = 1
        menu_im.name = "acceptROI"

        menu_im.controls.append(menu_control)

        menu_handler.apply(self.im_server, "ROI")

        self.im_server.applyChanges()

    def acceptCB(self, feedback):
        (row1, col1) = im_utils.pixelsFromMeters(self.x1, self.y1, self.ppm, self.image.rows, self.image.cols)
        (row2, col2) = im_utils.pixelsFromMeters(self.x2, self.y2, self.ppm, self.image.rows, self.image.cols)
        
        min_row = max(0, min(row1, row2))
        max_row = min(self.image.rows-1, max(row1, row2))
        min_col = max(0, min(col1, col2))
        max_col = min(self.image.cols-1, max(col1, col2))
        
        self.result = (min_row, max_row, min_col, max_col)
        self.im_server.clear()
        self.im_server.applyChanges()
        self.hmi_callback()

    def updateROI(self):
        self.roi_im.pose.position.x = (self.x1 + self.x2)/2
        self.roi_im.pose.position.y = (self.y1 + self.y2)/2
        scale = self.roi_im.controls[0].markers[0].scale
        scale.x = self.x2 - self.x1
        scale.y = self.y2 - self.y1
        self.im_server.insert(self.roi_im)
        self.im_server.applyChanges()

    # TODO: I'm not sure how to make this work w/ 
    def process_feedback(self, feedback):
        if feedback.control_name == "tl_corner":
            rospy.loginfo("got tl feedback")
            #self.processTL(feedback)
        elif feedback.control_name == "br_corner":
            rospy.loginfo("got br feedback")
            #self.processBR(feedback)
        

    # TODO: figure out how to get this to update the position of the ROI marker ... 
    # TODO: should only need one callback, and can switch on the originating marker topic?
    def processTL(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            pp = feedback.pose.position
            self.x1 = pp.x
            self.y1 = pp.y
            self.updateROI()

    def processBR(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            pp = feedback.pose.position
            self.x2 = pp.x
            self.y2 = pp.y
            self.updateROI()
