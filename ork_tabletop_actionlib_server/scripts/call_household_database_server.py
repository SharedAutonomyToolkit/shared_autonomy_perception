import sys
import rospy

from manipulation_msgs.srv import *
from household_objects_database_msgs.msg import DatabaseModelPose
from geometry_msgs.msg import PoseStamped

def call_db():
    rospy.wait_for_service('/objects_database_node/database_grasp_planning')
    try:
        grasp = rospy.ServiceProxy('/objects_database_node/database_grasp_planning', GraspPlanning)
        req = GraspPlanningRequest()
        req.arm_name = 'right_arm'
        req.target.reference_frame_id = 'head_mount_kinect_rgb_optical_frame'
        model = DatabaseModelPose()
        model.model_id = 18699
        pose = PoseStamped()
        pose.header.frame_id = 'head_mount_kinect_rgb_optical_frame'
        pose.pose.position.x = 0.320944428444
        pose.pose.position.y = 0.0485505461693
        pose.pose.position.z = 0.959076166153
        pose.pose.orientation.x = 0.992253601551
        pose.pose.orientation.y = 0.000377745833248
        pose.pose.orientation.z = 0.0030180788599
        pose.pose.orientation.w = 0.124191477895
        model.pose = pose
        req.target.potential_models.append(model)
        resp = grasp(req)
        for i in range(resp.grasps.__len__()):
            print "resp: ", resp.grasps[i].grasp_pose
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    call_db()
