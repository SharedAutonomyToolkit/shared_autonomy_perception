#! /usr/bin/env python

import random

import rospy

import actionlib
import smach
import smach_ros

from actionlib_msgs.msg import GoalStatus
from clear_table.grasp_handler import GraspHandler
from clear_table.moveit_handler import PickupHandler, DropHandler
from clear_table.scene_handler import SceneHandler
from clear_table.sensor_handler import SensorHandler
from sensor_msgs.msg import PointCloud2
from shared_autonomy_msgs.msg import TabletopGoal, TabletopAction

class Home(smach.State):
     def __init__(self):
         smach.State.__init__(self, outcomes=['at_home'])
     def execute(self, userdata):
         return 'at_home'

class SegmentORK(smach.State):
     def __init__(self):
         smach.State.__init__(self, outcomes=['done_segmenting', 'segmentation_failed', 'restart_segmentation', 'no_objects'],
                              output_keys=['object_points', 'object_name'])

         #self.sensors = SensorHandler()
         self.scene = SceneHandler()
         self.orkClient = actionlib.SimpleActionClient('ork_tabletop', TabletopAction)
         self.point_pub = rospy.Publisher('object_points', PointCloud2)

     def execute(self, userdata):
         self.orkClient.wait_for_server()
         goal = TabletopGoal()
         self.orkClient.send_goal(goal)
         self.orkClient.wait_for_result()
         state = self.orkClient.get_state()
         result = self.orkClient.get_result()
         if state != GoalStatus.SUCCEEDED:
             return 'segmentation_failed'

         num_objs = len(result.objects)
         if num_objs == 0:
             return 'no_objects'

         # need to set up the planning scene ...
         self.scene.clear_scene()

         print result.table_pose
         print result.table_dims
         self.scene.add_table(result.table_pose, result.table_dims)

         # randomly choose object to pick up
         obj_idx = random.randrange(0, num_objs)
         points = result.objects[obj_idx]
         userdata.object_points = points
         pick_object = 'obj' + str(obj_idx)
         userdata.object_name = pick_object

         self.scene.add_object(pick_object, points)

         #rospy.sleep(5.0)

         #self.point_pub.publish(result.objects[obj_idx])

         #print obj_idx
         # and, add only the other objects to the octomap
         #for idx in range(num_objs):
             #if idx != obj_idx:
             #    print 'publishing object %d' % idx
             #    self.point_pub.publish(result.objects[idx])
             #    rospy.sleep(2.0)
             #obj_name = 'obj' + str(idx)
             #obj_points = result.objects[idx]
             #self.scene.add_object(obj_name, obj_points)

         
         print 'successful segmentation!'
         return 'done_segmenting'


class SegmentGrabcut(smach.State):
     def __init__(self):
         smach.State.__init__(self, outcomes=['done_segmenting', 'segmentation_failed', 'restart_segmentation', 'no_objects'],
                              output_keys=['object_points', 'object_name'])

         self.sensors = SensorHandler()
         self.scene = SceneHandler()

     def execute(self, userdata):

         # we're willing to wait 15seconds to get the data
         kinect_data = self.sensors.get_kinect(15)
         if not kinect_data:
             return 'segmentation_failed'

         (state, result) = self.sensors.get_segmentation(kinect_data)
         if state != GoalStatus.SUCCEEDED:
             return 'segmentation_failed'

         points = self.sensors.get_point_cloud(kinect_data, result.mask)
         if not points:
             return 'segmentation_failed'

         userdata.object_points = points
         userdata.object_name = 'obj1'

         # need to set up the planning scene ...
         #self.scene.clear_scene()
         #self.scene.add_table()
         #self.scene.add_object('obj1', points)
         self.scene.update_scene(points)
         
         rospy.loginfo('... sleeping')
         #rospy.sleep(5.0)

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
        if grasps:
            userdata.grasps = grasps
            return 'grasps_found'
        else:
            return 'no_grasps'
     
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

    use_grabcut = rospy.get_param('clear_table_use_grabcut', True)

    with sm:
        smach.StateMachine.add('HOME', Home(),
                               transitions = {'at_home':'SEGMENT'})
        # TODO: I'm not sure if this is the best pattern here.
        # both of these classes are required to have the same SMACH interface.
        if use_grabcut:
            smach.StateMachine.add('SEGMENT', SegmentGrabcut(),
                                   transitions = {'done_segmenting':'GENERATE_GRASPS',
                                                  'segmentation_failed':'SEGMENT',
                                                  'restart_segmentation':'SEGMENT',
                                                  'no_objects':'DONE'})
        else:
            smach.StateMachine.add('SEGMENT', SegmentORK(),
                                   transitions = {'done_segmenting':'GENERATE_GRASPS',
                                                  'segmentation_failed':'SEGMENT',
                                                  'restart_segmentation':'SEGMENT',
                                                  'no_objects':'DONE'})

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
                                              
