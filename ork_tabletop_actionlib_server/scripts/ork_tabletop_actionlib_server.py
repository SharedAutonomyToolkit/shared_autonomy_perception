#! /usr/bin/env python

#import roslib; roslib.load_manifest('ork_tabletop_actionlib_server')
import rospy
import actionlib
import shared_autonomy_msgs.msg
import threading
import sys

import tf
from object_recognition_msgs.msg import RecognizedObjectArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
import tf.transformations as trans
from tf import TransformListener
#from ork_tabletop_actionlib_server import pointclouds
import pointclouds
import numpy as np

class ORKTabletop(object):

    def __init__(self, name):
        self.tl = TransformListener()

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, shared_autonomy_msgs.msg.TabletopAction, execute_cb=self.execute_cb, auto_start=False)

        self.sub = rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, self.callback)
        self.pub = rospy.Publisher('/recognized_object_array_as_point_cloud', PointCloud2)
        self.pose_pub = rospy.Publisher('/recognized_object_array_as_pose_stamped', PoseStamped)

        self.table_topic = "/marker_tables"
        # TODO: Are we still actively using the transforms broadcast here? I'd thought that that disappeared in a previous implementation...
        self.br = tf.TransformBroadcaster()
        self.centroid_table_pose = PoseStamped()

        # create messages that are used to publish feedback/result.
        # accessed by multiple threads
        self._result = shared_autonomy_msgs.msg.TabletopResult()
        self.result_lock = threading.Lock()

        self.table_dim = Point()
        # we do not compute the height
        # TODO: if we don't (can't!) compute the height, should we at least give it non-zero depth? 
        # (would also require shifting the observed centroid down by table_dim.z/2)
        self.table_dim.z = 0.0

        # TODO: should this be a parameter??
        self.table_link = 'table_link'

        self._as.start()


    # TODO: Is this really the best structure for handling the callbacks?
    # Would it be possible to have separate callbacks for table and objects, each updating a most-recently-seen variable?
    # Or maybe use the message_filters.TimeSynchronizer class if corresponding table/object data has identical timestamps?
    def callback(self, data):
        with self.result_lock:
            rospy.loginfo("Objects %d", data.objects.__len__())

            # list of points in table (I think ...)
            table_points = []

            # obtain table_offset and table_pose
            to = rospy.wait_for_message(self.table_topic, MarkerArray);

            # obtain Table corners ...
            rospy.loginfo("Tables hull size %d", to.markers.__len__())
            if to.markers.__len__() == 0:
                rospy.loginfo("No tables detected")
                return
            else:
                # TODO: What does this magic number signify?
                point_array_size = 4
                for i in range (0, point_array_size):
                    p = Point()
                    p.x = to.markers[0].points[i].x
                    p.y = to.markers[0].points[i].y
                    p.z = to.markers[0].points[i].z
                    table_points.append(p)
                # this is a table pose at the edge close to the robot, in the center of x axis
                table_pose = PoseStamped()
                table_pose.header = to.markers[0].header
                table_pose.header.stamp = rospy.Time.now()
                table_pose.pose = to.markers[0].pose
                
            # Determine table dimensions
            min_x = sys.float_info.max
            min_y = sys.float_info.max
            max_x = -sys.float_info.max
            max_y = -sys.float_info.max

            for i in range (table_points.__len__()):
                if table_points[i].x > max_x:
                    max_x = table_points[i].x
                if table_points[i].y > max_y:
                    max_y = table_points[i].y
                if table_points[i].x < min_x:
                    min_x = table_points[i].x
                if table_points[i].y < min_y:
                    min_y = table_points[i].y

            self.table_dim.x = abs(max_x - min_x)
            self.table_dim.y = abs(max_y - min_y)
            print "Dimensions: ", self.table_dim.x, self.table_dim.y

            # centroid of a table in table_link frame
            centroid = PoseStamped()
            centroid.header.frame_id = self.table_link
            centroid.header.stamp = rospy.Time.now()
            centroid.pose.position.x = 0.0
            centroid.pose.position.y = (max_y - min_y)/2.
            centroid.pose.position.z = 0.0
            centroid.pose.orientation.x = 0.0
            centroid.pose.orientation.y = 0.0
            centroid.pose.orientation.z = 0.0
            centroid.pose.orientation.w = 1.0

            # clusters
            cluster_list = []
            # transform each object into desired frame; add to list of clusters
            for i in range (data.objects.__len__()):
                time = rospy.Time.now()
                rospy.loginfo("Point clouds %s", data.objects[i].point_clouds.__len__())
                # table to camera transform
                pos = table_pose.pose.position
                rot = table_pose.pose.orientation
                self.br.sendTransform((pos.x, pos.y, pos.z), (rot.x, rot.y, rot.z, rot.w),
                                      time, self.table_link, to.markers[0].header.frame_id)

                # centroid of the table in head_mount_kinect_rgb_optical_frame
                if self.tl.canTransform(to.markers[0].header.frame_id, self.table_link, rospy.Time(0)):
                    self.centroid_table_pose = self.tl.transformPose(to.markers[0].header.frame_id, centroid)
                    self.pose_pub.publish(self.centroid_table_pose)
                else:
                    rospy.logwarn("No transform between %s and %s possible",to.markers[0].header.frame_id, self.table_link)
                    return

                # transform point cloud
                pc = PointCloud2()
                pc = data.objects[i].point_clouds[0]

                arr = pointclouds.pointcloud2_to_array(pc, 1)
                arr_xyz = pointclouds.get_xyz_points(arr)

                arr_xyz_trans = []
                for j in range (arr_xyz.__len__()):
                    ps = PointStamped();
                    ps.header.frame_id = self.table_link
                    ps.header.stamp = time
                    ps.point.x = arr_xyz[j][0]
                    ps.point.y = arr_xyz[j][1]
                    ps.point.z = arr_xyz[j][2]
                    if self.tl.canTransform(to.markers[0].header.frame_id, self.table_link, rospy.Time(0)):
                        ps_in_kinect_frame = self.tl.transformPoint(to.markers[0].header.frame_id, ps)
                    else:
                        rospy.logwarn("No transform between %s and %s possible",to.markers[0].header.frame_id, self.table_link)
                        return
                    arr_xyz_trans.append([ps_in_kinect_frame.point.x, ps_in_kinect_frame.point.y, ps_in_kinect_frame.point.z])
                # arr_xyz_trans.append([ps.point.x, ps.point.y, ps.point.z])
                pc_trans = PointCloud2()
                pc_trans = pointclouds.xyz_array_to_pointcloud2(np.asarray([arr_xyz_trans]), rospy.Time.now(), to.markers[0].header.frame_id)
                self.pub.publish(pc_trans)
                cluster_list.append(pc_trans)
                rospy.loginfo("cluster size %d", cluster_list.__len__())
            self._result.objects = cluster_list 

    def execute_cb(self, goal):
        # helper variables
        success = True

        rospy.loginfo('Executing ORKTabletop action')

        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
            return

        if success:
            with self.result_lock:
                self._result.table_pose = self.centroid_table_pose
                self._result.table_dims = self.table_dim
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)
                
if __name__ == '__main__':
    rospy.init_node('ork_tabletop')
    ORKTabletop(rospy.get_name())
    rospy.spin()
