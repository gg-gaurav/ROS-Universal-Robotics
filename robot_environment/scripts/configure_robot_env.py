#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point, PointStamped, Pose, Quaternion
import sys

import rospkg


import moveit_commander

def shutdown():
    """When node is shut down, remove the previous added objects from the planning scene."""
    scene.remove_world_object()
    scene.remove_attached_object("tool0", "welding_gun")


def initialize_planning_obstacles(scene):
    """Fill this method for adding obstacles"""

    # Configure pose of obstacle
    box_pose = PoseStamped()
    box_pose.header.frame_id = "map"
    box_pose.pose.position = Point(0, -0.295, 0.515)
    box_pose.pose.orientation = Quaternion(0,0,0,1)
    scene.add_box("camera_box", box_pose, size = (0.15, 0.19, 0.15))

    
    # add it to the planning scene
    #scene.add_XYZ()
    bottom_box_pose = PoseStamped()
    bottom_box_pose.header.frame_id = "map"
    bottom_box_pose.pose.position = Point(0, -0.295, -0.08)
    bottom_box_pose.pose.orientation = Quaternion(0,0,0,1)
    scene.add_box("bottom_box", bottom_box_pose, size = (1.5, 1.5, 0.10))

    #scene.add_box("base_box", 1, 1.75, 0.1, 0, -0.75, -0.05,  frame_id = map)




def initialize_nozzle(scene):
    """Fill this method for adding the nozzle to the planning scene.
    It's important, otherwise the nozzle can collide with the robot/ the environment."""

    # path to nozzle
    rospack = rospkg.RosPack()
    nozzle_path = rospack.get_path("robot_environment") + "/meshes/" + "welding_nozzle.stl"
    """ nozzle_path = rospack.get_path("whats_the_name_of_this_package?") + "/meshes/" + "nozzle" """

    # pose of nozzle relative to frame tool0
    nozzle_pose = PoseStamped()
    nozzle_pose.header.frame_id = "tool0"
    nozzle_pose.pose.position = Point(0,0,0.0008)
    nozzle_pose.pose.orientation = Quaternion( 0, 0.7071068, 0.7071068, 0 )

    #scene.add_mesh( "nozzle_mesh", nozzle_pose, nozzle_path, size = (0.01, 0.01, 0.01))
    scene.attach_mesh("tool0", "nozzle", nozzle_pose, filename = nozzle_path, size = (0.01, 0.01, 0.01), touch_links = ["tool0"])
    #scene.attach_XYZ()



if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()

    rospy.init_node("my_awesome_node_name")
    
    initialize_planning_obstacles(scene)
    initialize_nozzle(scene)

    rospy.on_shutdown(shutdown)
    
    while not rospy.is_shutdown():
        rospy.rostime.wallsleep(0.5)





    
    
