import rospy
import tf
#from std_msgs.msg import String
from object_recognition_msgs.msg import RecognizedObjectArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray
import tf.transformations as trans

pub = rospy.Publisher('/recognized_object_array_as_point_cloud', PointCloud2)
pose_pub = rospy.Publisher('/recognized_object_array_as_pose_stamped', PoseStamped)
br = tf.TransformBroadcaster()

def callback(data):
    rospy.loginfo("Objects %d", data.objects.__len__())

    #table_offset and poses
    p1 = PointStamped()
    p2 = PointStamped() 
    p3 = PointStamped()
    p4 = PointStamped()
    table_pose = PoseStamped()
    to = rospy.wait_for_message("/marker_tables", MarkerArray);
    rospy.loginfo("Tables hull size %d", to.markers.__len__())
    if to.markers.__len__() > 0:
        p1.header.frame_id = p2.header.frame_id = p2.header.frame_id = p3.header.frame_id = "table_link"
        p1.header.stamp = p2.header.stamp = p2.header.stamp = p3.header.stamp = rospy.Time.now()
        p1.point.x = to.markers[0].points[0].x
        p1.point.y = to.markers[0].points[0].y
        p1.point.z = to.markers[0].points[0].z
        p2.point.x = to.markers[0].points[1].x
        p2.point.y = to.markers[0].points[1].y
        p2.point.z = to.markers[0].points[1].z
        p3.point.x = to.markers[0].points[2].x
        p3.point.y = to.markers[0].points[2].y
        p3.point.z = to.markers[0].points[2].z
        p4.point.x = to.markers[0].points[3].x
        p4.point.y = to.markers[0].points[3].y
        p4.point.z = to.markers[0].points[3].z
#        print "point: ", p1, p2, p3, p4
        table_pose.header = to.markers[0].header
        table_pose.header.stamp = rospy.Time.now()
        table_pose.pose = to.markers[0].pose

    # clusters    
    cloud_list = []
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
 
       
        q1 =[table_pose.pose.orientation.x, table_pose.pose.orientation.y, table_pose.pose.orientation.z, table_pose.pose.orientation.w]
        q2 = trans.quaternion_about_axis(1.57, (0, 1, 0))
        q3 = trans.quaternion_multiply(q1,q2)
        #table to camera transform
        br.sendTransform((table_pose.pose.position.x, table_pose.pose.position.y, table_pose.pose.position.z),
                        (table_pose.pose.orientation.x, table_pose.pose.orientation.y, table_pose.pose.orientation.z, table_pose.pose.orientation.w),
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
        pub.publish(pc)
        cloud_list.append(pc)
        rospy.loginfo("cluster size %d", cloud_list.__len__())
        

def listener():
    rospy.init_node('ork_tabletop_actionlib_server', anonymous=True)
    rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
