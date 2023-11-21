#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import actionlib

from welding_robot_msgs.msg import WeldingPathAction 
from welding_robot_msgs.msg import WeldingPathFeedback
from welding_robot_msgs.msg import WeldingPathGoal
from welding_robot_msgs.msg import WeldingPathResult

from geometry_msgs.msg import PoseStamped, Point, Quaternion



class RobotMoverActionServer():

    _result = WeldingPathResult()
    _feedback = WeldingPathFeedback()

    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    def __init__(self) -> None:
        ...



        self.action_server = actionlib.SimpleActionServer("ur_mover", WeldingPathAction, execute_cb=self.action_callback, auto_start=False)
        self.action_server.start()



    def action_callback(self, goal):
        """Gets called by the action server is a client send a goal to it."""
        #print(goal.poses,len(goal.poses))
        waypoints = []
        for i in goal.poses: 
            #print(i)
            waypoints.append(i.pose)
        #print("Waypoints :",waypoints)

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)      
        print("How realistic the path is :",fraction)
        self.move_group.execute(plan, wait=True)

        # for i in range(len(goal.poses)):
        #     p = goal.poses[i].pose.position
        #     q = goal.poses[i].pose.orientation

        #     print(i,p,q,'\n')
        #     self._feedback.feedback_code = i
        #     self.action_server.publish_feedback(self._feedback)
        #     self.move_group.set_pose_target([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        #     plan = self.move_group.plan()
        #     self.move_group.go(wait=True)
        
        # self._result.result_code = 1
        # self.action_server.set_succeeded(self._result)







if __name__ == "__main__":
    
    rospy.init_node("node_name")
    moveit_commander.roscpp_initialize(sys.argv)

    server = RobotMoverActionServer()
    


    

    rospy.spin()
