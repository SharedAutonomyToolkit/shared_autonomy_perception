#! /usr/bin/env python

import numpy as np
#import scipy
import scipy.spatial as ss
import threading
import sys

import rospy
import actionlib

from tf import TransformListener

from object_recognition_msgs.msg import RecognizedObjectArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, PointStamped, Point, TransformStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from shared_autonomy_msgs.msg import TabletopAction, TabletopResult
from visualization_msgs.msg import MarkerArray

# TODO: Fix the module organization of this package (this belongs in ../src/ork_tabletop_action_server/)
# TODO: there's some other pointcloud2 handling code in clear_table ... is there some canonical tool for this, so each module doesn't have its own library?
#from ork_tabletop_actionlib_server import pointclouds
import pointclouds

class ORKTabletop(object):
    """ Listens to the table and object messages from ORK. Provides ActionServer
    that assembles table and object into same message. 
    NB - the table is an axis-aligned bounding box in the kinect's frame"""
    def __init__(self, name):

        self.sub = rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, self.callback)
        self.pub = rospy.Publisher('/recognized_object_array_as_point_cloud', PointCloud2)
        self.pose_pub = rospy.Publisher('/recognized_object_array_as_pose_stamped', PoseStamped)

        # We listen for ORK's MarkerArray of tables on this topic
        self.table_topic = "/marker_tables"
        self.clusters_topic = "/marker_array_clusters"
        self.extract_clusters = True
        self.recognize_objects = True
        self.search_radius = 0.02

        self.tl = TransformListener()

        # create messages that are used to publish feedback/result.
        # accessed by multiple threads
        self._result = TabletopResult()
        self.result_lock = threading.Lock()
        # used s.t. we don't return a _result message that hasn't been updated yet. 
        self.has_data = False

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, TabletopAction, 
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def extract_clusters_f(self):
        clusters = rospy.wait_for_message(self.clusters_topic, MarkerArray);
        rospy.loginfo("Clusters %d", clusters.markers.__len__())

        cluster_list = []
        for i in range(clusters.markers.__len__()):
#            rospy.loginfo("Clusters' header %s", clusters.markers[i].header.frame_id)
#            rospy.loginfo("Clusters' points %s", clusters.markers[i].points.__len__())
            arr_xyz = []
            for points in range(clusters.markers[i].points.__len__()):
                arr_xyz.append([clusters.markers[i].points[points].x, clusters.markers[i].points[points].y, clusters.markers[i].points[points].z])
            pc = PointCloud2()
            pc = pointclouds.xyz_array_to_pointcloud2(np.asarray([arr_xyz]), clusters.markers[i].header.stamp, clusters.markers[i].header.frame_id)
#            rospy.sleep(1.0)
            self.pub.publish(pc)
            cluster_list.append(pc)
        return cluster_list

    def compute_centroid (self, cloud):
        arr = pointclouds.pointcloud2_to_array(cloud, 0)
        arr_xyz = pointclouds.get_xyz_points(arr)
        min_x = sys.float_info.max
        min_y = sys.float_info.max
        min_z = sys.float_info.max
        max_x = -sys.float_info.max
        max_y = -sys.float_info.max
        max_z = -sys.float_info.max

        for j in range (arr_xyz.__len__()):
            if arr_xyz[j][0] > max_x:
                max_x = arr_xyz[j][0]
            if arr_xyz[j][1] > max_y:
                max_y = arr_xyz[j][1]
            if arr_xyz[j][2] > max_z:
                max_z = arr_xyz[j][2]
            if arr_xyz[j][0] < min_x:
                min_x = arr_xyz[j][0]
            if arr_xyz[j][1] < min_y:
                min_y = arr_xyz[j][1]
            if arr_xyz[j][2] < min_z:
                min_z = arr_xyz[j][2]
        c_x = (max_x + min_x)/2.
        c_y = (max_y + min_y)/2.
        c_z = (max_z + min_z)/2.
#        print "c_x: ", c_x, " ", "c_y: ", c_y, "c_z: ", c_z 
#        print "min, max", min_x, " ",  max_x, " ",  min_y, " ",  max_y, " ",  min_z, " ",  max_z
        return [c_x, c_y, c_z]


    def closest(self, X, p):
        disp = X - p
        return np.argmin((disp*disp).sum(1))

    
    def do_kdtree(self, array, point, radius):
        "Uses KD-Tree to query set of points and returns indices of those within a radius"
        tree = ss.KDTree(array)
        a = tree.query_ball_point(point, radius)
        return a

   # TODO: Is this really the best structure for handling the callbacks?
    # Would it be possible to have separate callbacks for table and objects, each updating a most-recently-seen variable?
    # Or maybe use the message_filters.TimeSynchronizer class if corresponding table/object data has identical timestamps?
    def callback(self, data):
        rospy.loginfo("Recognized objects %d", data.objects.__len__())

        table_corners = []

        # obtain table_offset and table_pose
        to = rospy.wait_for_message(self.table_topic, MarkerArray)

        # obtain Table corners ...
        rospy.loginfo("Tables hull size %d", to.markers.__len__())
        if not to.markers:
            rospy.loginfo("No tables detected")
            return
        else:
            # NB - D says that ORK has this set to filter based on height.
            # IIRC, it's 0.6-0.8m above whatever frame is set as the floor
            point_array_size = 4      # for the 4 corners of the bounding box
            for i in range (0, point_array_size):
                p = Point()
                p.x = to.markers[0].points[i].x
                p.y = to.markers[0].points[i].y
                p.z = to.markers[0].points[i].z
                table_corners.append(p)
            # this is a table pose at the edge close to the robot, in the center of x axis
            table_pose = PoseStamped()
            table_pose.header = to.markers[0].header
            table_pose.pose = to.markers[0].pose
            
        # Determine table dimensions
        rospy.loginfo('calculating table pose bounding box in frame: %s' % table_pose.header.frame_id)
        min_x = sys.float_info.max
        min_y = sys.float_info.max
        max_x = -sys.float_info.max
        max_y = -sys.float_info.max

        for i in range (table_corners.__len__()):
            if table_corners[i].x > max_x:
                max_x = table_corners[i].x
            if table_corners[i].y > max_y:
                max_y = table_corners[i].y
            if table_corners[i].x < min_x:
                min_x = table_corners[i].x
            if table_corners[i].y < min_y:
                min_y = table_corners[i].y

        table_dim = Point()
        # TODO: if we don't (can't!) compute the height, should we at least give it non-zero depth? 
        # (would also require shifting the observed centroid down by table_dim.z/2)
        table_dim.z = 0.0

        table_dim.x = abs(max_x - min_x)
        table_dim.y = abs(max_y - min_y)
        print "Dimensions: ", table_dim.x, table_dim.y

        # Temporary frame used for transformations
        table_link = 'table_link'

        # centroid of a table in table_link frame
        centroid = PoseStamped()
        centroid.header.frame_id = table_link
        centroid.header.stamp = table_pose.header.stamp
        centroid.pose.position.x = (max_x + min_x)/2.
        centroid.pose.position.y = (max_y + min_y)/2.
        centroid.pose.position.z = 0.0
        centroid.pose.orientation.x = 0.0
        centroid.pose.orientation.y = 0.0
        centroid.pose.orientation.z = 0.0
        centroid.pose.orientation.w = 1.0

        # generate transform from table_pose to our newly-defined table_link
        tt = TransformStamped()
        tt.header = table_pose.header
        tt.child_frame_id = table_link
        tt.transform.translation = table_pose.pose.position
        tt.transform.rotation = table_pose.pose.orientation
        self.tl.setTransform(tt)
        self.tl.waitForTransform(table_link, table_pose.header.frame_id, table_pose.header.stamp, rospy.Duration(3.0))
        if self.tl.canTransform(table_pose.header.frame_id, table_link, table_pose.header.stamp):
            centroid_table_pose = self.tl.transformPose(table_pose.header.frame_id, centroid)
            self.pose_pub.publish(centroid_table_pose)
        else:
            rospy.logwarn("No transform between %s and %s possible",table_pose.header.frame_id, table_link)
            return

        # transform each object into desired frame; add to list of clusters
        cluster_list = []
        object_list = []
        #if only clusters on the table should be extracted
        if self.extract_clusters:
            cluster_list = self.extract_clusters_f()
 #else try to recognize objects
        if self.recognize_objects:
            for i in range (data.objects.__len__()):
                #            rospy.loginfo("Point clouds %s", data.objects[i].point_clouds.__len__())              
                pc = PointCloud2()
                pc = data.objects[i].point_clouds[0]
                arr = pointclouds.pointcloud2_to_array(pc, 1)
                arr_xyz = pointclouds.get_xyz_points(arr)
                
                arr_xyz_trans = []
                for j in range (arr_xyz.__len__()):
                    ps = PointStamped();
                    ps.header.frame_id = table_link
                    ps.header.stamp = table_pose.header.stamp
                    ps.point.x = arr_xyz[j][0]
                    ps.point.y = arr_xyz[j][1]
                    ps.point.z = arr_xyz[j][2]
                    if self.tl.canTransform(table_pose.header.frame_id, table_link, table_pose.header.stamp):
                        ps_in_kinect_frame = self.tl.transformPoint(table_pose.header.frame_id, ps)
                    else:
                        rospy.logwarn("No transform between %s and %s possible",table_pose.header.frame_id, table_link)
                        return
                    arr_xyz_trans.append([ps_in_kinect_frame.point.x, ps_in_kinect_frame.point.y, ps_in_kinect_frame.point.z])
                    
                pc_trans = PointCloud2()
                pc_trans = pointclouds.xyz_array_to_pointcloud2(np.asarray([arr_xyz_trans]), 
                                                                    table_pose.header.stamp, table_pose.header.frame_id)
                    
                #self.pub.publish(pc_trans)
                object_list.append(pc_trans)
        
        rospy.loginfo("object size %d", object_list.__len__())

        cluster_centroids = []
        object_centroids = []
        for cloud in range (cluster_list.__len__()):
            cluster_centroids.append(self.compute_centroid(cluster_list[cloud]))

        for cloud in range (object_list.__len__()):
            object_centroids.append(self.compute_centroid(object_list[cloud]))

        recognized_objects = []
        indices = []

        for centroid in range (cluster_centroids.__len__()):
        # dist = self.closest(np_cluster_centroids, np.asarray(cluster_centroids[centroid]))
            if object_centroids:
                indices = self.do_kdtree(np.asarray(object_centroids), np.asarray(cluster_centroids[centroid]), self.search_radius)
            if not indices:
                recognized_objects.append(0)
            else:
                recognized_objects.append(1)
        print "recognized objects: ", recognized_objects

        
        # finally - save all data in the object that'll be sent in response to actionserver requests
        with self.result_lock:
            self._result.objects = cluster_list 
            self._result.recognized_objects = recognized_objects 
            self._result.table_dims = table_dim
            self._result.table_pose = centroid_table_pose
            self.has_data = True

    def execute_cb(self, goal):
        rospy.loginfo('Executing ORKTabletop action')

        # want to get the NEXT data coming in, rather than the current one. 
        with self.result_lock:
            self.has_data = False

        rr = rospy.Rate(1.0)
        while not rospy.is_shutdown() and not self._as.is_preempt_requested():
            with self.result_lock:
                if self.has_data:
                    break
            rr.sleep()

        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
        elif rospy.is_shutdown():
            self._as.set_aborted()
        else:
            with self.result_lock:
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)
                
if __name__ == '__main__':
    rospy.init_node('ork_tabletop')
    ORKTabletop(rospy.get_name())
    rospy.spin()
