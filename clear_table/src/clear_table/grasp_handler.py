#! /usr/bin/env python

import rospy


from object_manipulator import draw_functions

from manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
from moveit_msgs.msg import Grasp
from trajectory_msgs.msg import JointTrajectoryPoint

class GraspHandler():
    def __init__(self):
        self.grasp_client = rospy.ServiceProxy('plan_point_cluster_grasp', GraspPlanning)
        self.draw_functions = draw_functions.DrawFunctions('grasp_markers')

    def get_grasps(self, object_points):
        print 'waiting for cluster grasp planner server'
        rospy.wait_for_service('plan_point_cluster_grasp')
        print '...got service'
     
        req = GraspPlanningRequest()
        req.arm_name = "left_arm"
        req.target.region.cloud = object_points
        # TODO: I have no idea what this "graspable object reference frame" thing is. 
        req.target.reference_frame_id = object_points.header.frame_id
    
        try:
            res = self.grasp_client(req)
        except rospy.ServiceException:
            print 'grasp server call failed'
            return None

        grasps = res.grasps
        self.show_grasps(grasps)

        # Finally, need to convert to moveit-compatible grasps before we can 
        # send them along to the actual pickup node!
        moveit_grasps = [self.convert_grasp(grasp) for grasp in grasps]

        return moveit_grasps

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

    def show_grasps(self, grasps): 
        poses = [grasp.grasp_pose.pose for grasp in grasps] 
        self.draw_functions.clear_grasps()
        self.draw_functions.draw_grasps(poses, grasps[0].grasp_pose.header.frame_id, pause = 0) 
