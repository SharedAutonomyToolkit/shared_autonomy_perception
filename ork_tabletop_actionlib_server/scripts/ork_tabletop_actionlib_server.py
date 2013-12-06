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
import pointclouds
import numpy as np

class ORKTabletop(object):
  # create messages that are used to publish feedback/result
  _result   = shared_autonomy_msgs.msg.tabletopResult()

  def __init__(self, name):
    self.tl = TransformListener()
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, shared_autonomy_msgs.msg.tabletopAction, execute_cb=self.execute_cb)
    self._as.start()
    self.sub = rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, self.callback)
    self.pub = rospy.Publisher('/recognized_object_array_as_point_cloud', PointCloud2)
    self.pose_pub = rospy.Publisher('/recognized_object_array_as_pose_stamped', PoseStamped)
    self.table_topic = "/marker_tables"
    self.br = tf.TransformBroadcaster()
    self.table_pose = PoseStamped()
    self.centroid_table_pose = PoseStamped()
    self.lock = threading.Lock()
    self.table_dim = Point()
    #we do not compute the height
    self.table_dim.z = 0.0
    self.cloud_list = []
    self.point_list = []
    self.point_array_size = 4
    self.table_link = 'table_link'

  def callback(self, data):
    rospy.loginfo("Objects %d", data.objects.__len__())

    #table_offset and table_pose
    to = rospy.wait_for_message(self.table_topic, MarkerArray);
    rospy.loginfo("Tables hull size %d", to.markers.__len__())
    if to.markers.__len__() > 0:
        self.point_list = []
        for i in range (0, self.point_array_size):
          p = Point()
          p.x = to.markers[0].points[i].x
          p.y = to.markers[0].points[i].y
          p.z = to.markers[0].points[i].z
          self.point_list.append(p)
        #this is a table pose at the edge close to the robot, in the center of x axis
        self.table_pose.header = to.markers[0].header
        self.table_pose.header.stamp = rospy.Time.now()
        self.table_pose.pose = to.markers[0].pose
    else:
      rospy.loginfo("No tables detected")
      return
    
    min_x = min_y = sys.float_info.max
    max_x = max_y = -sys.float_info.max

    for i in range (self.point_list.__len__()):
      if self.point_list[i].x > max_x:
        max_x = self.point_list[i].x
      if self.point_list[i].y > max_y:
        max_y = self.point_list[i].y
      if self.point_list[i].x < min_x:
        min_x = self.point_list[i].x
      if self.point_list[i].y < min_y:
        min_y = self.point_list[i].y


    self.table_dim.x = abs(max_x - min_x)
    self.table_dim.y = abs(max_y - min_y)
    print "Dimensions: ", self.table_dim.x, self.table_dim.y

    #centroid of a table in table_link frame
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
    self.cloud_list = []
    for i in range (data.objects.__len__()):
        time = rospy.Time.now()
        rospy.loginfo("Point clouds %s", data.objects[i].point_clouds.__len__())
        #table to camera transform
        self.br.sendTransform((self.table_pose.pose.position.x, self.table_pose.pose.position.y, self.table_pose.pose.position.z),
                        (self.table_pose.pose.orientation.x, self.table_pose.pose.orientation.y, self.table_pose.pose.orientation.z, self.table_pose.pose.orientation.w),
#                         (q3[0], q3[1], q3[2], q3[3]),
                         time,
                         self.table_link,
                         to.markers[0].header.frame_id)
        
        #centroid of the table in head_mount_kinect_rgb_optical_frame
        if self.tl.canTransform(to.markers[0].header.frame_id, self.table_link, rospy.Time(0)):
          self.centroid_table_pose = self.tl.transformPose(to.markers[0].header.frame_id, centroid)
          self.pose_pub.publish(self.centroid_table_pose)
        else:
          rospy.logwarn("No transform between %s and %s possible",to.markers[0].header.frame_id, self.table_link)    
          return

        #transform point cloud
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
#          arr_xyz_trans.append([ps.point.x, ps.point.y, ps.point.z])
        pc_trans = PointCloud2()
        pc_trans = pointclouds.xyz_array_to_pointcloud2(np.asarray([arr_xyz_trans]), rospy.Time.now(), to.markers[0].header.frame_id)
        self.pub.publish(pc_trans)
        self.cloud_list.append(pc_trans)
        rospy.loginfo("cluster size %d", self.cloud_list.__len__())
   
  def execute_cb(self, goal):
    with self.lock:
      # helper variables
      success = True
      
    # publish info to the console for the user
      rospy.loginfo('Executing ORKTabletop action')
      
      # start executing the action
      # check that preempt has not been requested by the client
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
        return
      
      if success:
        self._result.table_pose = self.centroid_table_pose
        self._result.table_dims = self.table_dims
        self._result.objects = self.cloud_list  
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('ork_tabletop')
  ORKTabletop(rospy.get_name())
  rospy.spin()
