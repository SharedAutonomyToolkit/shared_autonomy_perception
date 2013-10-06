#! /usr/bin/env python

import random

import rospy

import actionlib
#import simple_robot_control
import smach
import smach_ros

from shared_autonomy_msgs.msg import *

# TODO: add start state that moves trunk all the way up, and has robot look down!
class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['setup_done'])

    def execute(self, userdata):
        #torso = simple_robot_control.Torso()
        #torso.move(0.3)
        #head = simple_robot_control.Head()
        #head.lookAtPoint(1.0, 0, 0, 'base_link')
        return 'setup_done'

# Home state - (open loop?) sets arms to side, transitions to segmentation. (no data passed), OR to done. 
class Home(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['home_done', 'home_reset'])

    def execute(self, userdata):
        # these angles were found by using the interactive manipulation demo to move arms to side, and
        # reading back the joint angles. I couldn't find the service call that would get the effect for me
        home_angles_r = [-2.13, -0.02, -1.64, -2.07, -1.64, -1.68, 1.40]
        home_angles_l = [2.13, -0.02, 1.63, -2.06, 1.64, -1.68, 1.40]
        #arm_l = simple_robot_control.Arm('l')
        #arm_r = simple_robot_control.Arm('r')
        # i hqave no idea why this fails sometimes
        # sarah says that simple_robot_control - check that I start the right arm controller (rcart - the cartesian one)
        # use pr2_controller_manager to see which one is runnign when it's working/not working
        #arm_l.goToAngle(home_angles_l, 5.0)
        #arm_r.goToAngle(home_angles_r, 5.0)

        kp = raw_input("In state Home. Press 'q' to exit state machine, any other key to continue\n")
        if kp == 'q':
            return 'home_done'
        else:
            return 'home_reset'

# Segmentation - collects image data (service call to rgbd_assembler), gets segmentation (actionlib call to given segmentation algorithm). Passes points corresponding to segmented object to the next state
class Segment(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['seg_done'])

        # TODO: I'm not sure whether this belongs here, or somewhere else ...
        # If it's in the execute() function, we get:
        # " [ERROR] Got a feedback callback when we're not tracking a goal"
        self.client = actionlib.SimpleActionClient('/ben_segmentation_node', SegmentAction)
        self.client.wait_for_server()

    def segmentDoneCB(self, state, result):
        print "segmentation done:"
        print state
        print result

    def segmentFeedbackCB(self, feedback):
        print "feedback received"
        print feedback

    def execute(self, userdata):
        goal = SegmentGoal()
        self.client.send_goal(goal, done_cb=self.segmentDoneCB, feedback_cb=self.segmentFeedbackCB)

        if random.random() < 0.5:
            self.client.cancel_goal()

        self.client.wait_for_result()

        kp = raw_input("In state Segment. Press any key to continue. \n")
        return 'seg_done'

# get graps - service call to whatever the new thing is in the manipulation pipeline. passes grasp to next state, or transitions back to home if no grasps found.
class GetGrasps(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['no_grasp', 'grasp_found'])
    def execute(self, userdata):
        kp = raw_input("In state GetGrasps. Press 's' if grasp found, any other key for failure. \n")
        if kp == 's':
            return 'grasp_found'
        else:
            return 'no_grasp'

# execute pickup - service call to arm planning; passes nothing to next state. If successful, transitions to drop object; otherwise, to home
class Pickup(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['pickup_failed', 'pickup_done'])
    def execute(self, userdata):
        # Sarah says use pickup goal here - give it grasp, not graspable object
        kp = raw_input("In state Pickup. Press 's' if pickup succeeded, any other key for failure.\n")
        if kp == 's':
            return 'pickup_done'
        else:
            return 'pickup_failed'

# drop object - open-loop moves the hand to above the bin, releases object. transitions to home state. 
class Drop(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['drop_done'])
        #self.arm_r = simple_robot_control.Arm('r')
        #self.gripper_r = simple_robot_control.Gripper('r')
    def execute(self, userdata):

        drop_angles_r = [-1.59, -0.12, -2.45, -0.09, -0.68, -1.49, 1.68]
        #self.arm_r.goToAngle(drop_angles_r, 5.0)
        #self.gripper_r.openGripper()
        
        kp = raw_input("In State Pickup. Press any key to continue\n")
        return 'drop_done'


def main():
    rospy.init_node('clear_table')

    sm = smach.StateMachine(outcomes=['DONE'])
    # TODO: do I need to initialize all userdata here??


    with sm:
        smach.StateMachine.add('SETUP', Setup(),
                               transitions = {'setup_done':'HOME'})
        smach.StateMachine.add('HOME', Home(),
                               transitions = {'home_done':'DONE',
                                              'home_reset':'SEGMENT'})
        smach.StateMachine.add('SEGMENT', Segment(),
                               transitions = {'seg_done':'GETGRASPS'})
        smach.StateMachine.add('GETGRASPS', GetGrasps(),
                               transitions = {'no_grasp':'HOME',
                                              'grasp_found':'PICKUP'})
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
