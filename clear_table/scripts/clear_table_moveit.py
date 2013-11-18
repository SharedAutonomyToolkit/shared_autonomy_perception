#! /usr/bin/env python

# This is the version of Clear Table that uses Hydro's version of moveit to actually do the pickup tasks

# TODO: clean up service/topic names and turn into parameters! (+ launch file)
# TODo: parameterize arm name!

import math

import rospy

import actionlib
from cv_bridge import CvBridge
import handle_point_cloud2 as pts
import sensor_msgs # for converting point clouds
import smach
import smach_ros
import tf

# modules imported
#from moveit_commander import MoveGroupCommander
from object_manipulator import draw_functions

# messages imported
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Point, Vector3
from manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
from moveit_msgs.msg import CollisionObject, Grasp, PickupAction, PickupGoal
from shape_msgs.msg import SolidPrimitive
from object_manipulation_msgs.srv import FindClusterBoundingBox2, FindClusterBoundingBox2Request
from sensor_msgs.msg import PointCloud, PointCloud2
from shared_autonomy_msgs.msg import SegmentGoal, SegmentAction
from shared_autonomy_msgs.srv import KinectAssembly
from trajectory_msgs.msg import JointTrajectoryPoint

class Home(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_home'])
    def execute(self, userdata):
        return 'at_home'
                             

class Segment(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['done_segmenting', 'no_data', 'segmentation_failed', 'no_objects'], 
                             output_keys=['object_points'])

        print 'initializing Segment'

        self.kinect_client = rospy.ServiceProxy('head_mount_kinect/assemble_kinect', KinectAssembly)
        self.point_publisher = rospy.Publisher('segmented_points', PointCloud2)
        self.segment_client = actionlib.SimpleActionClient('segment_service', SegmentAction)


    def execute(self, userdata):

        print "waiting for assemble_kinect service"
        rospy.wait_for_service('head_mount_kinect/assemble_kinect')
        print "... got service"

        print "waiting for segmentation server"
        self.segment_client.wait_for_server()    
        print "... got segmentation server"

        try:
            kinect_data = self.kinect_client()
        except rospy.ServiceException:
            print 'clear_table unable to get kinect data!'
            rospy.sleep(5.0)
            return 'no_data'
        
        goal = SegmentGoal()
        goal.image = kinect_data.image
        goal.depth = kinect_data.depth

        print "Attempting to get segmentation"
        self.mask = None
        self.segment_client.send_goal(goal)
        self.segment_client.wait_for_result()
        if self.segment_client.get_state() != GoalStatus.SUCCEEDED:
            return 'segmentation_failed'
        result = self.segment_client.get_result()
        self.mask = result.mask
        print "... got segmentation"

        points = self.get_point_cloud(kinect_data, result.mask)
        self.point_publisher.publish(points)
        userdata.object_points = points

        return 'done_segmenting'

    def get_point_cloud(self, data, mask):
        print "I should be publishing points!"
        mybridge = CvBridge()
        img = mybridge.imgmsg_to_cv(mask)

        # TODO: depend on the read_points that should be in common_msgs?
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
        return out_cloud


class GenerateGrasps(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['no_grasps', 'grasps_found'], 
                             input_keys = ['object_points'],
                             output_keys = ['grasps', 'object_name'])
        #self.bb_client = rospy.ServiceProxy('/find_cluster_bounding_box2', FindClusterBoundingBox2)
        self.grasp_client = rospy.ServiceProxy('plan_point_cluster_grasp', GraspPlanning)

        self.draw_functions = draw_functions.DrawFunctions('grasp_markers')
        self.collision_pub = rospy.Publisher('collision_object', CollisionObject)

        self.listener = tf.TransformListener()
        self.transformer = tf.TransformerROS()

    def execute(self, userdata):
        #print 'waiting for cluster bb server'
        #rospy.wait_for_service('find_cluster_bounding_box2')
        #print '...got service'

        print 'waiting for cluster grasp planner server'
        rospy.wait_for_service('plan_point_cluster_grasp')
        print '...got service'
        
        print 'waiting to get transforms!'
        rr = rospy.Rate(1.0)
        while ((not self.listener.frameExists("odom_combined") or
                not self.listener.frameExists(userdata.object_points.header.frame_id)) and 
               not rospy.is_shutdown()):
            print self.listener.frameExists("odom_combined")
            print self.listener.frameExists(userdata.object_points.header.frame_id)
            rr.sleep()
        print 'got transforms!'
        
        bb = self.get_bounding_box(userdata.object_points)
        if bb is None:
            return 'no_grasps'
        # TODo: this appears multiple places ... 
        userdata.object_name = "box1"


        table_pose = PoseStamped()
        table_pose.header.frame_id = "odom_combined"
        table_pose.pose.position.x = 0.55
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.75
        table_pose.pose.orientation.w = 1.0
        table_dims = Point()
        table_dims.x = 0.7
        table_dims.y = 1.0
        table_dims.z = 0.05
        self.insert_object(table_pose, table_dims, "table")

        req = GraspPlanningRequest()
        req.arm_name = "left_arm"
        req.target.region.cloud = userdata.object_points
        # TODO: I have no idea what this "graspable object reference frame" thing is. 
        req.target.reference_frame_id = userdata.object_points.header.frame_id
    
        try:
            res = self.grasp_client(req)
        except rospy.ServiceException:
            print 'grasp server call failed'
            return 'no_grasps'

        grasps = res.grasps
        self.show_grasps(grasps)

        # Finally, need to convert to moveit-compatible grasps before we can 
        # send them along to the actual pickup node!
        moveit_grasps = [self.convert_grasp(grasp) for grasp in grasps]
        userdata.grasps = moveit_grasps

        return 'grasps_found'

    def get_bounding_box(self, points):
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
        min_x = 100.0
        max_x = -100.0
        min_y = 100.0
        max_y = -100.0
        min_z = 100.0
        max_z = -100.0
        for pt in pc_world.points:
            min_x = min(min_x, pt.x)
            max_x = max(max_x, pt.x)
            min_y = min(min_y, pt.y)
            max_y = max(max_y, pt.y)
            min_z = min(min_z, pt.z)
            max_z = max(max_z, pt.z)
        
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

        obj_name = "box1"
        self.insert_object(obj_pose, obj_dims, obj_name)
        # TODO: this is ugly - I'm using the None return type to indicate that this failed ... 
        return True

        #bb_req = FindClusterBoundingBox2Request()
        #bb_req.cluster = userdata.object_points
        #try:   
        #    bb_resp = self.bb_client(bb_req)
        #except rospy.ServiceException:
        #    print 'no response from cluster bb server!'
        #    return 'no_grasps'
        #print bb_resp
        #object_name = "box1"
        #userdata.object_name = object_name
        #self.insert_object(bb_resp.pose, bb_resp.box_dims, object_name)


    def convert_grasp(self, grasp):
        """Converts a manipulation_msgs/Grasp into moveit_msgs/Grasp
        NOT General - specifically for the PR2, given how the pose specification 
        and gripper controller changed"""
        m_grasp = Grasp()

        # determine which hand it's for
        if grasp.grasp_posture.name[0] == 'l_gripper_l_finger_joint':
            hand_joints = ['l_gripper_motor_screw_joint']
        else:
            hand_joints = ['r_gripper_motor_screw_joint']

        #print 'postures: %0.2f, %0.2f' % (grasp.pre_grasp_posture.position[0], 
        #                                  grasp.grasp_posture.position[0])

        first_point = JointTrajectoryPoint()
        first_point.positions = [grasp.pre_grasp_posture.position[0]]
        m_grasp.pre_grasp_posture.points.append(first_point)
        m_grasp.pre_grasp_posture.joint_names = hand_joints

        second_point = JointTrajectoryPoint()
        second_point.positions = [grasp.grasp_posture.position[0]]
        m_grasp.grasp_posture.points.append(second_point)
        m_grasp.grasp_posture.joint_names = hand_joints

        m_grasp.grasp_pose = grasp.grasp_pose
        m_grasp.grasp_quality = grasp.grasp_quality
        m_grasp.max_contact_force = grasp.max_contact_force

        m_grasp.pre_grasp_approach = grasp.approach
        m_grasp.post_grasp_retreat = grasp.retreat

        return m_grasp

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

    def show_grasps(self, grasps): 
        poses = [grasp.grasp_pose.pose for grasp in grasps] 
        self.draw_functions.clear_grasps()
        self.draw_functions.draw_grasps(poses, grasps[0].grasp_pose.header.frame_id, pause = 0) 

class Pickup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pickup_failed', 'pickup_done'],
                             input_keys=['object_name', 'grasps'])
        print 'initializing Pickup'
        #self.group = MoveGroupCommander("left_arm")
        self.pickup_client = actionlib.SimpleActionClient('pickup', PickupAction)
        self.pickup_client.wait_for_server()

    def execute(self, userdata):
        pickup_goal = PickupGoal()
        pickup_goal.target_name = userdata.object_name
        pickup_goal.group_name = "left_arm"
        pickup_goal.possible_grasps = userdata.grasps
        pickup_goal.support_surface_name = "table"
        pickup_goal.allow_gripper_support_collision = True
        pickup_goal.allowed_planning_time = 30.0
        # Fill in the goal here
        self.pickup_client.send_goal(pickup_goal)
        self.pickup_client.wait_for_result()
        pickup_result = self.pickup_client.get_result()
        print 'Pickup result: %d' % (pickup_result.error_code.val,)

        if pickup_result.error_code.val == 1:
            return 'pickup_done'
        else:
            return 'pickup_failed'

        #success = self.group.pick(userdata.object_name, grasp=userdata.grasps)
        #print 'pickup success = %r' % (success,)
        #if success:
        #    return 'pickup_done'
        #else:
        #    return 'pickup_failed'

class Drop(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['drop_done'],
                             input_keys = ['object_name'])
        # TODO: have separate movegroup commanders at the same time? is that OK? 
        # TODO: parameterize this arm!
        #self.group = MoveGroupCommander("left_arm")
        self.collision_pub = rospy.Publisher('collision_object', CollisionObject)

    def execute(self, userdata):
        # TODO: must be better thing to do than just prescripted place where it goes to drop!
        # TODO: where does the frame get specified here?
        obj_pose = Pose()

        obj_pose.position.x = 0.45
        obj_pose.position.y = 0.72
        obj_pose.position.z = 1.1
        obj_pose.orientation.w = 1.0

        #success = self.group.place(userdata.object_name, obj_pose)
        success = False
        if success:
            self.remove_object(userdata.object_name)
        else:
            print 'place failed!'

        return 'drop_done'

    def remove_object(self, name):
        remove_object = CollisionObject()
        remove_object.header.frame_id = "odom_combined"
        remove_object.id = name
        remove_object.operation = remove_object.REMOVE

        self.collision_pub.publish(remove_object)


# ready -> segment -> bbox + update environment (pass on object name) -> grasps (pass on generated grasps + object name) -> pickup -> drop -> reset
def main():
    rospy.init_node('clear_table')
    sm = smach.StateMachine(outcomes=['DONE'])
    
    # TODO: I'm avoiding makign the remapping explicit. May need to in the future. 
    with sm:
        smach.StateMachine.add('HOME', Home(),
                               transitions = {'at_home':'SEGMENT'})
        smach.StateMachine.add('SEGMENT', Segment(),
                               transitions = {'done_segmenting':'GENERATE_GRASPS',
                                              'segmentation_failed':'SEGMENT',
                                              'no_data':'SEGMENT',
                                              'no_objects':'HOME'})# NYI
        smach.StateMachine.add('GENERATE_GRASPS', GenerateGrasps(),
                               transitions = {'no_grasps':'HOME', 
                                              'grasps_found':'PICKUP'})
        smach.StateMachine.add('PICKUP', Pickup(),
                               transitions = {'pickup_failed':'HOME',
                                              'pickup_done':'DROP'})
        smach.StateMachine.add('DROP', Drop(),
                               transitions = {'drop_done':'HOME'})

    sis = smach_ros.IntrospectionServer('clear_table_sis', sm, '/SM_ROOT')
    sis.start()
    
    outcome = sm.execute()

    rospy.spin()
    sis.stop() 


if __name__ == "__main__":
    main()
