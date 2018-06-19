#!/usr/bin/env python

from lis_pr2_pkg.uber_controller import Uber
import rospy
import os
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import time
from actionlib import SimpleActionClient as SAC
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import GoalStatus as gs


## action client for arm movement
class Arm:
    def __init__(self, arm):
        if arm == 'r':
            self.arm_client = SAC('r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
            self.arm = arm
        elif arm == 'l':
            self.arm_client = SAC('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
            self.arm = arm

        self.arm_client.wait_for_server()
        rospy.loginfo('Waiting for server...')

    def move(self, angles, time, blocking):
        angles = [angles]
        times = [time]
        timeout=times[-1] + 1.0

        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names =self.get_joint_names(self.arm)
        
        for (ang, t) in zip(angles, times):
            point = JointTrajectoryPoint()
            point.positions = ang
            point.time_from_start = rospy.Duration(t)
            goal.trajectory.points.append(point)
        goal.trajectory.header.stamp = rospy.Time.now()
        
        self.arm_client.send_goal(goal)
        rospy.sleep(.1)
        rospy.loginfo("command sent to client")
        status = 0

        if blocking: #XXX why isn't this perfect?
            end_time = rospy.Time.now() + rospy.Duration(timeout+ .1)
            while (
                    (not rospy.is_shutdown()) and\
                    (rospy.Time.now() < end_time) and\
                    (status < gs.SUCCEEDED) and\
                    (type(self.arm_client.action_client.last_status_msg) != type(None))):
                status = self.arm_client.action_client.last_status_msg.status_list[-1].status #XXX get to 80
                rospy.Rate(10).sleep()
            if status >gs.SUCCEEDED:
                rospy.loginfo("goal status achieved.  exiting")
            else:
                rospy.loginfo("ending due to timeout")

        result = self.arm_client.get_result()
        return result

    def  get_joint_names(self, char):
        return [char+'_shoulder_pan_joint',
                char+'_shoulder_lift_joint',
                char+'_upper_arm_roll_joint',
                char+'_elbow_flex_joint',
                char+'_forearm_roll_joint',
                char+'_wrist_flex_joint',
                char+'_wrist_roll_joint' ]

if __name__ == "__main__":
    rospy.init_node("corlor_detector")
    rospy.loginfo('Node initialized. Yeah, Baby!')
    right_arm =Arm('r')
    left_arm = Arm('l')


    for _ in range(2):
        left_arm.move([1.5,  -1, 3, 1, 0, 0, 0], time=1, blocking=True)
        right_arm.move([-1.5, -1,-3, 1, 0, 0, 0], time=1, blocking=True)
        time.sleep(1)
        left_arm.move([1.5,  0.3, 3, 0, 0, 0, 0], time=1, blocking=True)
        right_arm.move([-1.5, 0.3,-3, 0, 0, 0, 0], time=1, blocking=True)
        time.sleep(1)
    
    # uc = Uber()
    # for _ in range(2):
    #     uc.command_joint_pose('l', [1.5,  -1, 3, 1, 0, 0, 0], time=1, blocking=True)
    #     uc.command_joint_pose('r', [-1.5, -1,-3, 1, 0, 0, 0], time=1, blocking=True)
    #     time.sleep(1)
    #     uc.command_joint_pose('l', [1.5,  0.3, 3, 0, 0, 0, 0], time=1, blocking=True)
    #     uc.command_joint_pose('r', [-1.5, 0.3,-3, 0, 0, 0, 0], time=1, blocking=True)
    #     time.sleep(1)