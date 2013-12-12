#! /usr/bin/env python

import math

import rospy

import actionlib
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pts

from sensor_msgs.msg import PointCloud2
from shared_autonomy_msgs.msg import SegmentGoal, SegmentAction
from shared_autonomy_msgs.srv import KinectAssembly

class SensorHandler():
    def __init__(self):
        self.kinect_client = rospy.ServiceProxy('head_mount_kinect/assemble_kinect', KinectAssembly)
        self.segment_client = actionlib.SimpleActionClient('segment_service', SegmentAction)
        self.point_publisher = rospy.Publisher('segmented_points', PointCloud2)

    def get_kinect(self, delay):
        """ this handles calling the kinect_assembly service"""

        print "waiting for assemble_kinect service"
        rospy.wait_for_service('head_mount_kinect/assemble_kinect')
        print "... got service"

        # wait 15 seconds to get data
        rospy.loginfo('Trying to get kinect data')
        kinect_data = []
        rr = rospy.Rate(1.0)
        count = 0
        while not kinect_data and count < delay:
            try:
                kinect_data = self.kinect_client()
            except rospy.ServiceException:
                rospy.loginfo('...attempt %d' % count)
                count += 1
                rr.sleep()

        return kinect_data
        
    def get_segmentation(self, kinect_data):
        """ And this handles calling the segmentation actionlib """

        print "waiting for segmentation server"
        self.segment_client.wait_for_server()    
        print "... got segmentation server"

        goal = SegmentGoal()
        goal.image = kinect_data.image
        goal.depth = kinect_data.depth

        print "Attempting to get segmentation"
        self.segment_client.send_goal(goal)
        self.segment_client.wait_for_result()
        state = self.segment_client.get_state()
        result = self.segment_client.get_result()

        return (state, result)

    def get_point_cloud(self, data, mask):
        print "I should be publishing points!"
        mybridge = CvBridge()
        try:
            img = mybridge.imgmsg_to_cv(mask)
        except CvBridgeError, e:
            print e
            return []

        # TODO: why didn't the uvs=idxs code work? x
        pt_gen = pts.read_points(data.points)#, uvs=idxs, skip_nans=True) # this didn't work!!

        out_pts = []
        for jj in range(data.points.height):
            for ii in range(data.points.width):
                pt = pt_gen.next()
                if not math.isnan(pt[0]):
                    if (img[jj,ii] == 1 or img[jj,ii]==3):
                        out_pts.append(pt[0:3])
        print "done creating output point cloud"

        out_cloud = pts.create_cloud_xyz32(data.points.header, out_pts)
        self.point_publisher.publish(out_cloud)
        return out_cloud

