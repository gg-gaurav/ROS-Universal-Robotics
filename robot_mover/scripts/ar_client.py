#!/usr/bin/env python3

import rospy
from math import pi
import sys
import moveit_commander
import actionlib
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, Transform
import PyKDL as kdl
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from welding_robot_msgs.msg import WeldingPathGoal, WeldingPathAction 
import tf
import tf2_ros
import tf2_geometry_msgs 

current_ar_pose = PoseStamped()
current_ar_poses = []
tf_poses = []
listener = None

def callback(data):
    #goal = data[0].pos
    #print(data.markers[0].pose.pose)
    global current_ar_pose
    if len(data.markers) < 1:
        return
    current_ar_pose = data.markers[0].pose

def send_goal():
    global current_ar_poses
    # poses = []

    # for trans, rot in current_ar_poses:
    #     pose = PoseStamped()
    #     pose.pose.position = Point(*trans)
    #     pose.pose.orientation = Quaternion(*rot)
    #     pose.header.frame_id = "base_link"
    #     poses.append(pose)
    #     print(pose)
    
    client = actionlib.SimpleActionClient("ur_mover", WeldingPathAction)
    client.wait_for_server()

    goal = WeldingPathGoal(current_ar_poses, "test")
    client.send_goal(goal)
    #client.send_goal(current_ar_poses)

    client.wait_for_result()

    result = client.get_result()

    if result == 1:
        print("=== EXECUTION WAS SUCCESSFUL")

    elif result == -1:
        print("=== ERROR! Execution failed")
    
    current_ar_poses = []


##############################################
def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcaste

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time(0)

    output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(0))
    return output_pose_stamped.pose

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise
##############################################


def register_pose():
    global current_ar_pose, current_ar_poses
    
    while not rospy.is_shutdown():
        try:
            user_input = input("Press q/x ")
            if user_input == "q":
                #print(transform_pose(current_ar_pose, "cam_frame", "base_link"))
                trans  = tf_buffer.lookup_transform("base_link", "cam_frame", rospy.Time(0))
                trans_pose = tf2_geometry_msgs.do_transform_pose(current_ar_pose, trans)


                trans_welding_tip = tf_buffer.lookup_transform("tool0", "welding_tip", rospy.Time(0))
                trans_pose2 = tf2_geometry_msgs.do_transform_pose(trans_pose, trans_welding_tip)

                trans_pose2.pose.position.z = 0.06
                trans_pose2.pose.position.x += -0.08
                trans_pose2.pose.position.y += 0.0 
                trans_pose2.pose.position.x *= -1.0 



                # trans_br = TransformStamped()
                # trans_br.header.stamp = rospy.Time.now()
                # trans_br.child_frame_id = "test_frame"
                # trans_br.header.frame_id = "base_link"
                # trans_br.transform.rotation = trans_pose2.pose.orientation
                # trans_br.transform.translation = trans_pose2.pose.position

                # broadcaster.sendTransform(trans_br)

                


                print(trans_pose2)


                # tcp = TransformStamped()
                # tcp.child_frame_id = "cam_frame"
                # tcp.header.frame_id = "base_link"
                # tcp.transform.translation = Point(*trans)
                # tcp.transform.rotation = Quaternion(*rot)

                
                # roty_180 = TransformStamped()
                # roty_180.transform.rotation.x = pi

                #pose = listener.transformPose(roty_180, tcp)      
                # print(trans, rot, current_ar_pose, transform_pose(current_ar_pose, "cam_frame", "base_link"))       
                current_ar_poses.append(trans_pose2)
                
            elif user_input == "x":
                send_goal()

        except KeyboardInterrupt:
            print("exiting..")
            rospy.signal_shutdown("user requested shutdown")

if __name__ == "__main__":
    rospy.init_node('ar_tags', anonymous=False)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    broadcaster = tf2_ros.TransformBroadcaster()
    #listener = tf.TransformListener()
    #broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    register_pose()
    