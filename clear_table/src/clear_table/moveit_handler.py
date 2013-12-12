#! /usr/bin/env python

import numpy as np

import rospy

import actionlib

from moveit_msgs.msg import PickupAction, PickupGoal, PlaceAction, PlaceGoal, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint

class PickupHandler():
    def __init__(self):
    
        print 'initializing Pickup'
        self.pickup_client = actionlib.SimpleActionClient('pickup', PickupAction)
        self.pickup_client.wait_for_server()

    def run_pick(self, object_name, grasps):
        pickup_goal = PickupGoal()
        pickup_goal.target_name = object_name
        pickup_goal.group_name = "left_arm"
        pickup_goal.possible_grasps = grasps
        # Augh. I *think* that this only handles if invalidated during execution, not during planning. 
        pickup_goal.planning_options.replan = True
        pickup_goal.planning_options.replan_attempts = 5
        pickup_goal.planning_options.replan_delay = 3.0
        pickup_goal.planning_options.planning_scene_diff.is_diff = True
        pickup_goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        # commenting these out b/c not allowed by the moveit_commander interface
        pickup_goal.support_surface_name = "table"
        pickup_goal.allow_gripper_support_collision = True
        pickup_goal.allowed_planning_time = 5.0

        # TODO: this shouldn't be required, I dont' think!
        #pickup_goal.allowed_touch_objects.append(userdata.object_name)


        # THIS IS THE ACTIONLIB APPROACH
        # Fill in the goal here
        self.pickup_client.send_goal(pickup_goal)
        self.pickup_client.wait_for_result()
        pickup_result = self.pickup_client.get_result()
        print 'Pickup result: %d' % (pickup_result.error_code.val,)
        if pickup_result.error_code.val == 1:
            return True
        else:
            return False

class DropHandler():
    def __init__(self):
        self.place_client = actionlib.SimpleActionClient('place', PlaceAction)
        self.place_client.wait_for_server()

    def run_drop(self, object_name):

        place_goal = PlaceGoal()
        place_goal.group_name = "left_arm"
        place_goal.attached_object_name = object_name
        place_goal.place_eef = True
        place_goal.place_locations = self.get_place_locations()
        place_goal.allowed_planning_time = 5.0
        place_goal.planning_options.planning_scene_diff.is_diff = True
        place_goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.place_client.send_goal(place_goal)
        self.place_client.wait_for_result()
        place_result = self.place_client.get_result()
        print 'Place result: %d' % (place_result.error_code.val,)
        if place_result.error_code.val == 1:
            return True
        else:
            return False
    
    def get_place_locations(self):
        locations = []
        for xx in np.arange(0.4, 0.7, 0.1):
            for yy in np.arange(0.5, 0.9, 0.1):
                pl = PlaceLocation()
                
                pt = JointTrajectoryPoint()
                pt.positions = [0.5]
                pl.post_place_posture.points.append(pt)
                pl.post_place_posture.joint_names = ['l_gripper_motor_screw_joint']
                
                pl.place_pose.header.frame_id = "odom_combined"
                pl.place_pose.header.stamp = rospy.Time.now()
                pl.place_pose.pose.position.x = xx
                pl.place_pose.pose.position.y = yy
                pl.place_pose.pose.position.z = 1.05
                # Quaternion obtained by pointing wrist down
                #pl.place_pose.pose.orientation.w = 1.0
                pl.place_pose.pose.orientation.x = 0.761
                pl.place_pose.pose.orientation.y = -0.003
                pl.place_pose.pose.orientation.z = -0.648
                pl.place_pose.pose.orientation.w = 0.015

                pl.pre_place_approach.direction.header.frame_id = "odom_combined"
                pl.pre_place_approach.direction.vector.z = -1.0
                pl.pre_place_approach.desired_distance = 0.2
                pl.pre_place_approach.min_distance = 0.1
                
                pl.post_place_retreat.direction.header.frame_id = "odom_combined"
                pl.post_place_retreat.direction.vector.z = 1.0
                pl.post_place_retreat.desired_distance = 0.2
                pl.post_place_retreat.min_distance = 0.1
   
                locations.append(pl)
        return locations

