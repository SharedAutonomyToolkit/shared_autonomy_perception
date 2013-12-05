#import roslib; roslib.load_manifest('ork_tabletop_actionlib_server')
import rospy

import actionlib

import shared_autonomy_msgs.msg

import threading
#from ork_tabletop_actionlib_server.msg import tabletopResult

import tf
#from std_msgs.msg import String
from object_recognition_msgs.msg import RecognizedObjectArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray
import tf.transformations as trans

class ORKTabletop(object):
  # create messages that are used to publish feedback/result
  _result   = shared_autonomy_msgs.msg.tabletopResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, shared_autonomy_msgs.msg.tabletopAction, execute_cb=self.execute_cb)
    self._as.start()
    self.sub = rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, self.callback)
    self.pub = rospy.Publisher('/recognized_object_array_as_point_cloud', PointCloud2)
    self.pose_pub = rospy.Publisher('/recognized_object_array_as_pose_stamped', PoseStamped)
    self.br = tf.TransformBroadcaster()
    self.table_pose = PoseStamped()
    self.lock = threading.Lock()
    self.p1 = PointStamped()
    self.p2 = PointStamped() 
    self.p3 = PointStamped()
    self.p4 = PointStamped()
    self.cloud_list = []
    
  def callback(self, data):
    rospy.loginfo("Objects %d", data.objects.__len__())

    #table_offset and poses
    to = rospy.wait_for_message("/marker_tables", MarkerArray);
    rospy.loginfo("Tables hull size %d", to.markers.__len__())
    if to.markers.__len__() > 0:
        self.p1.header.frame_id = self.p2.header.frame_id = self.p3.header.frame_id = self.p4.header.frame_id = "table_link"
        self.p1.header.stamp = self.p2.header.stamp = self.p3.header.stamp = self.p4.header.stamp = rospy.Time.now()
        self.p1.point.x = to.markers[0].points[0].x
        self.p1.point.y = to.markers[0].points[0].y
        self.p1.point.z = to.markers[0].points[0].z
        self.p2.point.x = to.markers[0].points[1].x
        self.p2.point.y = to.markers[0].points[1].y
        self.p2.point.z = to.markers[0].points[1].z
        self.p3.point.x = to.markers[0].points[2].x
        self.p3.point.y = to.markers[0].points[2].y
        self.p3.point.z = to.markers[0].points[2].z
        self.p4.point.x = to.markers[0].points[3].x
        self.p4.point.y = to.markers[0].points[3].y
        self.p4.point.z = to.markers[0].points[3].z
#        print "point: ", p1, p2, p3, p4
        self.table_pose.header = to.markers[0].header
        self.table_pose.header.stamp = rospy.Time.now()
        self.table_pose.pose = to.markers[0].pose

    # clusters    
    self.cloud_list = []
    for i in range (data.objects.__len__()):
#        rospy.loginfo("Point clouds %d", data.objects[i].point_clouds.__len__())
        # rospy.loginfo("Objects frame %s", data.objects[i].header.frame_id)
        # rospy.loginfo("Objects pose frame %s", data.objects[i].pose.header.frame_id)
        rospy.loginfo("Point clouds %s", data.objects[i].point_clouds.__len__())
        
        #table pose
        # pose = PoseStamped()
        # ma = rospy.wait_for_message("/marker_table_origins", MarkerArray);
        # rospy.loginfo("Tables pose size %d", ma.markers.__len__())
        # if ma.markers.__len__() > 0:
        #     pose.header = ma.markers[0].header
        #     pose.header.stamp = rospy.Time.now()
        #     pose.pose = ma.markers[0].pose
        #     pose_pub.publish(pose)
        # else:
        #     continue
 
       
        q1 =[self.table_pose.pose.orientation.x, self.table_pose.pose.orientation.y, self.table_pose.pose.orientation.z, self.table_pose.pose.orientation.w]
        q2 = trans.quaternion_about_axis(1.57, (0, 1, 0))
        q3 = trans.quaternion_multiply(q1,q2)
        #table to camera transform
        self.br.sendTransform((self.table_pose.pose.position.x, self.table_pose.pose.position.y, self.table_pose.pose.position.z),
                        (self.table_pose.pose.orientation.x, self.table_pose.pose.orientation.y, self.table_pose.pose.orientation.z, self.table_pose.pose.orientation.w),
#                         (q3[0], q3[1], q3[2], q3[3]),
                         rospy.Time.now(),
                         "table_link",
                         to.markers[0].header.frame_id)

        #transform point cloud
        pc = data.objects[i].point_clouds[0]
        #        pc.header.frame_id = 'head_mount_kinect_rgb_optical_frame'
        pc.header.frame_id = 'table_link'
        pc.header.stamp = rospy.Time.now()
#        rospy.sleep(2.0)
        self.pub.publish(pc)
        self.cloud_list.append(pc)
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
        self._result.table_pose = self.table_pose  
        self._result.p1 = self.p1  
        self._result.p2 = self.p2
        self._result.p3 = self.p3
        self._result.p4 = self.p4
        self._result.objects = self.cloud_list  
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('ork_tabletop')
  ORKTabletop(rospy.get_name())
  rospy.spin()
