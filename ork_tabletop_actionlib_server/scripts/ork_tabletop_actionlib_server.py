import rospy
import tf
#from std_msgs.msg import String
from object_recognition_msgs.msg import RecognizedObjectArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
import tf.transformations as trans

pub = rospy.Publisher('/recognized_object_array_as_point_cloud', PointCloud2)
pose_pub = rospy.Publisher('/recognized_object_array_as_pose_stamped', PoseStamped)
br = tf.TransformBroadcaster()

def callback(data):
    rospy.loginfo("Objects %d", data.objects.__len__())

    for i in range (data.objects.__len__()):
#        rospy.loginfo("Point clouds %d", data.objects[i].point_clouds.__len__())
        rospy.loginfo("Objects frame %s", data.objects[i].header.frame_id)
        rospy.loginfo("Objects pose frame %s", data.objects[i].pose.header.frame_id)

        #table pose
        pose = PoseStamped()
        ma = rospy.wait_for_message("/marker_table_origins", MarkerArray);
        rospy.loginfo("Markers size %d", ma.markers.__len__())
        if ma.markers.__len__() > 1:
            pose.header = ma.markers[0].header
            pose.header.stamp = rospy.Time.now()
            pose.pose = ma.markers[0].pose
            pose_pub.publish(pose)
        else:
            continue
        
        q1 =[pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        q2 = trans.quaternion_about_axis(1.57, (0, 1, 0))
        q3 = trans.quaternion_multiply(q1,q2)
        #table to camera transform
        br.sendTransform((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
#                        (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
                         (q3[0], q3[1], q3[2], q3[3]),
                         rospy.Time.now(),
                         "table_link",
                         ma.markers[0].header.frame_id)

        #transform point cloud
        pc = data.objects[i].point_clouds[0]
        #        pc.header.frame_id = 'head_mount_kinect_rgb_optical_frame'
        pc.header.frame_id = 'table_link'
        pc.header.stamp = rospy.Time.now()
        pub.publish(pc)
        

def listener():
    rospy.init_node('ork_tabletop_actionlib_server', anonymous=True)
    rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
