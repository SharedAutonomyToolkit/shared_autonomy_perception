# This is a utility class that handles all the sensor and PlanningScene interaction
# for the clear_table state machine

import sys

import rospy
import tf

import sensor_msgs.point_cloud2 as pts

from geometry_msgs.msg import Point, PoseStamped, Vector3
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningSceneWorld
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from sensor_msgs.msg import PointCloud
from shape_msgs.msg import SolidPrimitive


class SceneHandler():
    def __init__(self):
        self.collision_pub = rospy.Publisher('collision_object', CollisionObject)
        self.attach_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject)
        self.scene_diff_pub = rospy.Publisher('planning_scene_world', PlanningSceneWorld)
        
        self.listener = tf.TransformListener()
        self.scene_client = rospy.ServiceProxy('get_planning_scene', GetPlanningScene)

    def add_object(self, obj_name, object_points):
        """Adds bounding box of specified points to the PlanningScene"""
        print 'waiting to get transforms!'
        rr = rospy.Rate(1.0)

        while ((not self.listener.frameExists("odom_combined") or
                not self.listener.frameExists(object_points.header.frame_id)) and 
               not rospy.is_shutdown()):
            print self.listener.frameExists("odom_combined")
            print self.listener.frameExists(object_points.header.frame_id)
            rr.sleep()
        print 'got transforms!'
        
        (obj_pose, obj_dims) = self.get_bounding_box(object_points)
        self.insert_object(obj_pose, obj_dims, obj_name)
        
        
    def get_bounding_box(self, points):
        """ simple axis-aligned bounding box"""
        world_frame = "odom_combined"

        # get pointcloud in world frame
        if not self.listener.frameExists(points.header.frame_id):
            rospy.logerr("point cloud header frame %s does not exist!", points.header.frame_id)
            return 
        if not self.listener.frameExists(world_frame):
            rospy.logerr("world frame %s does not exist!", world_frame)
            return


        pc = PointCloud()
        pc.header = points.header
        iter_pt = pts.read_points(points)
        for pt in iter_pt:
            pc.points.append(Point(pt[0], pt[1], pt[2]))
        # TODO: technically, want to transform at time image was taken, but thanks 
        # to waiting for human input, will be arbitrary delay ... 
        pc.header.stamp = self.listener.getLatestCommonTime(world_frame, pc.header.frame_id)
        # TODO: should be wrapped in try/except block
        pc_world = self.listener.transformPointCloud(world_frame, pc)

        # get world-axis-aligned bounding box
        min_x = sys.float_info.max
        max_x = sys.float_info.min
        min_y = sys.float_info.max
        max_y = sys.float_info.min
        min_z = sys.float_info.max
        max_z = sys.float_info.min
        for pt in pc_world.points:
            min_x = min(min_x, pt.x)
            max_x = max(max_x, pt.x)
            min_y = min(min_y, pt.y)
            max_y = max(max_y, pt.y)
            min_z = min(min_z, pt.z)
            max_z = max(max_z, pt.z)
        # testing whether fudging it helps
        #min_z = min_z + 0.05
        
        # add bounding box to the planning scene
        obj_pose = PoseStamped()
        obj_pose.pose.position.x = (min_x + max_x)/2
        obj_pose.pose.position.y = (min_y + max_y)/2
        obj_pose.pose.position.z = (min_z + max_z)/2
        obj_pose.pose.orientation.z = 1.0

        obj_pose.header.frame_id = world_frame
        obj_dims = Vector3()
        obj_dims.x = max_x - min_x
        obj_dims.y = max_y - min_y
        obj_dims.z = max_z - min_z

        return (obj_pose, obj_dims)

    def add_table(self, table_pose=[], table_dims=[]):
        # TODO: actually have this find the plane - for now, it just adds the pre-set table
        if not table_pose:
            rospy.loginfo('adding default table!')
            table_pose = PoseStamped()
            table_pose.header.frame_id = "odom_combined"
            table_pose.pose.position.x = 0.55
            table_pose.pose.position.y = 0.0
            table_pose.pose.position.z = 0.75
            table_pose.pose.orientation.w = 1.0
        if not table_dims:
            table_dims = Point()
            table_dims.x = 0.7
            table_dims.y = 1.0
            table_dims.z = 0.05
        self.insert_object(table_pose, table_dims, "table")

    def insert_object(self, pose, dims, name):
        """ Expects a PoseStamped to be input, along with a Point and string """
        primitive = SolidPrimitive()
        primitive.type = primitive.BOX
        # TODO: this is a hack to make the buffer bigger.
        # By default, it appears to be 10cm x 10cm x 0cm, but I'm not sure which dimension is which
        primitive.dimensions = [dims.x, dims.y, dims.z]

        add_object = CollisionObject()
        add_object.id = name
        add_object.header.frame_id = pose.header.frame_id
        add_object.operation = add_object.ADD
        
        add_object.primitives.append(primitive)
        add_object.primitive_poses.append(pose.pose)
        
        self.collision_pub.publish(add_object)
        
    def remove_object(self, name):
        remove_object = CollisionObject()
        remove_object.header.frame_id = "odom_combined"
        remove_object.id = name
        remove_object.operation = remove_object.REMOVE

        self.collision_pub.publish(remove_object)
        
    def detach_object(self, name):

        detach_object = AttachedCollisionObject()
        detach_object.object.id = name
        detach_object.link_name = "l_wrist_roll_link"
        detach_object.object.operation = detach_object.object.REMOVE
        self.attach_pub.publish(detach_object)
                    
    def clear_scene(self):
        ps = PlanningSceneWorld()
        ps.octomap.header.frame_id = 'odom_combined'
        #ps.octomap.octomap.data.append(0) #TOTAL HACK. I have no clue what this actually does ...
        ps.octomap.octomap.id = 'OcTree'
        self.scene_diff_pub.publish(ps)
