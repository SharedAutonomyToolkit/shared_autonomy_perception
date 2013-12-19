#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import shared_autonomy_msgs.msg

def tabletop_client():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('ork_tabletop', shared_autonomy_msgs.msg.TabletopAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = shared_autonomy_msgs.msg.TabletopGoal()

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result() 

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('tabletop_client_py')
        result = tabletop_client()
        print "Table pose: "
        print result.table_pose
        print "Table dims: "
        print result.table_dims
        print "Num Objects: ", result.objects.__len__() 
        print "Recognized Objects: ", result.recognized_objects 
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
