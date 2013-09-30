from math import pi

from geometry_msgs.msg import Point
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *        
from std_msgs.msg import ColorRGBA
import tf

# Lots of code copied from boundingbox, as it's also displaying images
class EditPixelLabels():
    def __init__(self, hmi_callback, im_server, image, mask):
        self.hmi_callback = hmi_callback
        self.im_server = im_server
        self.image = image
        self.mask = mask

        # pixels-per-m for published markers
        self.ppm = 100

        # lists of pixels that'll be returned
        self.fg = None
        self.bg = None

        # draws the image using interactive markers
        self.add_image()
        # creates the button that'll be the done callback
        self.add_button()

    # used to set the im_server back to initial state ... 
    def cancel(self):
        self.im_server.clear()
        self.im_server.applyChanges()

    def add_image(self):
        print "add image called!"

        image_marker = Marker()
        image_marker.type = image_marker.POINTS
        image_marker.scale.x = 0.05
        image_marker.scale.y = 0.05
        image_marker.scale.z = 0.05

        for jj in xrange(0, self.image.cols, 3):
            for ii in xrange(0, self.image.rows, 3):
                # sinking it a bit s.t. the Interactive Markers are easier to grab
                pt = Point(1.0*jj/self.ppm, 1.0*(self.image.rows - ii)/self.ppm, -.05)
                image_marker.points.append(pt)
                # ROS is rgba, opencv is bgr
                mask_val = self.mask[ii,jj]
                red = 1.0-self.image[ii,jj][2]
                green = 1.0-self.image[ii,jj][1]
                blue = 1.0-self.image[ii,jj][0]
                if (mask_val == 1) or (mask_val == 3):
                    cc = ColorRGBA(red, green, blue, 1.0)
                else:
                    grey = 0.2989*red + 0.5870*green + 0.1140*blue
                    if mask_val == 0:
                        cc = ColorRGBA(grey, grey, grey, 0.25)
                    else:
                        cc = ColorRGBA(grey, grey, grey, 0.5)
                image_marker.colors.append(cc)

        image_control = InteractiveMarkerControl()
        image_control.always_visible = True
        image_control.markers.append(image_marker)

        image_im = InteractiveMarker()
        image_im.header.frame_id = "camera_link"
        image_im.name = "LabeledImage"
        image_im.description = ""
        image_im.controls.append(image_control)
        self.im_server.insert(image_im)
        self.im_server.applyChanges()

    def add_button(self):
        print "add button called!"

        button_marker = Marker()
        button_marker.type = button_marker.TEXT_VIEW_FACING
        button_marker.scale.z = 1.0 # height of letter A
        button_marker.pose.position.x = 3.2
        button_marker.pose.position.y = -0.5
        button_marker.pose.position.z = 0.0
        button_marker.color.r = 0.5
        button_marker.color.g = 0.5
        button_marker.color.b = 0.5
        button_marker.color.a = 1.0

        button_marker.text = "Done Labeling"

        button_control = InteractiveMarkerControl()
        button_control.always_visible = False
        button_control.interaction_mode = button_control.BUTTON
        button_control.name = "DoneLabeling"
        button_control.markers.append(button_marker)

        button_im = InteractiveMarker()
        button_im.header.frame_id = "camera_link"
        button_im.name = "DoneLabeling"
        button_im.description = ""
        button_im.controls.append(button_control)

        self.im_server.insert(button_im)
        self.im_server.setCallback(button_im.name, self.doneLabellingCB)
        self.im_server.applyChanges()


    # called when done button is clicked
    def doneLabellingCB(self, feedback):
        # This would be sloppy, but since there's only one menu item,
        # can assume that it means return
        print "doneLabellingCB called!!"

        # TODO: update fg/bg lists?

        self.im_server.clear()
        self.im_server.applyChanges()

        self.hmi_callback()

