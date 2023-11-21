#!/usr/bin/env python3

import rospy
import actionlib
import math

from geometry_msgs.msg import PoseStamped, Point, Quaternion

import tf_conversions

from welding_robot_msgs.msg import WeldingPathAction 
from welding_robot_msgs.msg import WeldingPathActionFeedback
from welding_robot_msgs.msg import WeldingPathActionGoal
from welding_robot_msgs.msg import WeldingPathActionResult
from welding_robot_msgs.msg import WeldingPathFeedback
from welding_robot_msgs.msg import WeldingPathGoal
from welding_robot_msgs.msg import WeldingPathResult


def get_pose(x,y,z):
    pose1 = PoseStamped()
    pose1.header.frame_id = "base_link"
    p = Point(x,y,z)
    pose1.pose.position = p

    quat = tf_conversions.transformations.quaternion_from_euler(math.pi,0,0)
    q = Quaternion(*quat)
    pose1.pose.orientation = q
    return pose1


def test_server():
    ...

    client = actionlib.SimpleActionClient("ur_mover", WeldingPathAction)

    print("=== waiting for server ...")
    client.wait_for_server()
    poses = list()

    poses.append(get_pose(0.07, 0.3, 0.1))
    
    
    goal = WeldingPathGoal(poses=poses, goal_description = "This is my funny goal Description")
    client.send_goal(goal)

    client.wait_for_result()

    result = client.get_result()

    if result == 1:
        print("=== EXECUTION WAS SUCCESSFUL")
    
    elif result == -1:
        print("=== ERROR! Execution failed")

if __name__ == "__main__":
    rospy.init_node("test_action_client")
    
    test_server()


