#! /usr/bin/env python

import rospy


from cluster_grasp_planner import draw_functions

from cluster_grasp_planner.srv import GraspPlanning, GraspPlanningRequest
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
        req.cluster2 = object_points
    
        try:
            res = self.grasp_client(req)
        except rospy.ServiceException:
            print 'grasp server call failed'
            return None

        grasps = res.grasps
        self.show_grasps(grasps)
        return grasps

    def show_grasps(self, grasps): 
        poses = [grasp.grasp_pose.pose for grasp in grasps] 
        self.draw_functions.clear_grasps()
        self.draw_functions.draw_grasps(poses, grasps[0].grasp_pose.header.frame_id, pause = 0) 
