#! /usr/bin/env python

import rospy

import smach
import smach_ros

from actionlib_msgs.msg import GoalStatus
from clear_table.grasp_handler import GraspHandler
from clear_table.moveit_handler import PickupHandler, DropHandler
from clear_table.scene_handler import SceneHandler
from clear_table.sensor_handler import SensorHandler

class Home(smach.State):
     def __init__(self):
         smach.State.__init__(self, outcomes=['at_home'])
     def execute(self, userdata):
         return 'at_home'

class Segment(smach.State):
     def __init__(self):
         smach.State.__init__(self, outcomes=['done_segmenting', 'segmentation_failed', 'restart_segmentation'],
                              output_keys=['object_points', 'object_name'])

         self.sensors = SensorHandler()
         self.scene = SceneHandler()

     def execute(self, userdata):
         # TODO: I don't like this way of catching exceptions ... the helper functions have the try/catch
         # block, but at the state machine level we check for None and branch based on that. 
         kinect_data = self.sensors.get_kinect()
         if kinect_data is None:
             return 'segmentation_failed'

         (state, result) = self.sensors.get_segmentation(kinect_data)
         if state != GoalStatus.SUCCEEDED:
             return 'segmentation_failed'

         points = self.sensors.get_point_cloud(kinect_data, result.mask)
         if points is None:
             return 'segmentation_failed'

         userdata.object_points = points
         userdata.object_name = 'obj1'

         # need to set up the planning scene ...
         self.scene.clear_scene()
         self.scene.add_table()
         self.scene.add_object('obj1', points)
         
         print 'successful segmentation!'
         return 'done_segmenting'

class GenerateGrasps(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['no_grasps', 'grasps_found'], 
                             input_keys = ['object_points'],
                             output_keys = ['grasps'])
        self.gh = GraspHandler()
    
    def execute(self, userdata):
        grasps = self.gh.get_grasps(userdata.object_points)
        if grasps is None:
            return 'no_grasps'
        
        userdata.grasps = grasps
        return 'grasps_found'

     
class Pickup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pickup_failed', 'pickup_done'],
                             input_keys=['object_name', 'grasps'])
        self.pickup = PickupHandler()

    def execute(self, userdata):
        success = self.pickup.run_pick(userdata.object_name, userdata.grasps)
        if success:
            return 'pickup_done'
        else:
            return 'pickup_failed'

class Drop(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['drop_done'],
                             input_keys = ['object_name'])
        self.drop = DropHandler()
        self.scene = SceneHandler()

    def execute(self, userdata):
        self.drop.run_drop(userdata.object_name)
        self.scene.remove_object(userdata.object_name)
        return 'drop_done'

def main():

    rospy.init_node('clear_table')
    sm = smach.StateMachine(outcomes=['DONE'])

    with sm:
        smach.StateMachine.add('HOME', Home(),
                               transitions = {'at_home':'SEGMENT'})

        smach.StateMachine.add('SEGMENT', Segment(),
                               transitions = {'done_segmenting':'GENERATE_GRASPS',
                                              'segmentation_failed':'SEGMENT',
                                              'restart_segmentation':'SEGMENT'})
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
                                              
