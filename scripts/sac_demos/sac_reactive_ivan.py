#!/usr/bin/env python
import rospy 
import numpy as np
from pr2_msgs.msg import PressureState as PS
from lis_pr2_pkg.uber_controller import Uber
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal, Pr2GripperCommand
import actionlib


R_STANDARD = 2900
L_STANDARD = 2900
INITIAL_POS = 0.1
POS_CHANGE = 0.005

rospy.init_node("grip_sac", anonymous=True)

class LeftGripper:
	def __init__(self):
		self.GCA = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
		self.GCA.wait_for_server()
		rospy.loginfo("Waiting for server...")

	def grip(self, position, max_effort):
		goal = Pr2GripperCommandGoal()
		command = Pr2GripperCommand()
		command.position = position
		command.max_effort = max_effort
		goal.command = command
		self.GCA.send_goal(goal)


class LeftGrip:
	def __init__(self):
		self.uc = Uber()
		self.uc.open_gripper("l")
		self.position = INITIAL_POS
		self.gripping = False
		self.left_gripper_sac = LeftGripper()
		self.sub = rospy.Subscriber("pressure/l_gripper_motor", PS, self.callback, queue_size=1)
		rospy.loginfo("Initialized grip detector.")

	def callback(self, data):
		left = np.array(data.l_finger_tip)
		right = np.array(data.r_finger_tip)
		l_mean = np.mean(left)
		r_mean = np.mean(right)

		# Calibration of default pressures
		# rospy.loginfo(l_mean)
		# rospy.loginfo(r_mean)

		# check if there is pressure on a finger tip
		if l_mean > L_STANDARD or r_mean > R_STANDARD:
			self.gripping = True
			
		# close grippers slowly	if enough pressure
		if self.gripping:
			self.position = self.position - POS_CHANGE
			if self.position < 0:
				response = raw_input("Ready to release?")
				self.position = INITIAL_POS
		                self.uc.command_gripper("l", self.position, -1, blocking=True, timeout=2)
				self.gripping = False
			self.left_gripper_sac.grip(self.position, 15)
		rospy.sleep(0.01)

lg = LeftGrip()
rospy.spin()