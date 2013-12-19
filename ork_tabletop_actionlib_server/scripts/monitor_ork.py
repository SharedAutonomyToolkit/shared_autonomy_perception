#! /usr/bin/python

import rospy

import subprocess

if __name__ == "__main__":
    rospy.init_node('monitor_ork')

    ork_core = subprocess.check_output('rospack find object_recognition_core', shell=True)
    ork_core_path = ork_core.strip()
    
    ork_tabletop = subprocess.check_output('rospack find object_recognition_tabletop', shell=True)
    ork_tabletop_path = ork_tabletop.strip()

    cmd = ork_core_path + '/apps/detection -c ' + ork_tabletop_path + '/conf/detection.object.ros.ork'

    cmd_list = cmd.split(' ')

    my_process = subprocess.Popen(cmd_list)
    
    print("testing")

    rr = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        rr.sleep()

    my_process.terminate()
    






