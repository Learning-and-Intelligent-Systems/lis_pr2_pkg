#!/usr/bin/env python
import rospy 
import numpy as np
from pr2_msgs.msg import PressureState as PS
from lis_pr2_pkg.uber_controller import Uber

R_STANDARD = 2900
L_STANDARD = 2900
INITIAL_POS = 0.1
POS_CHANGE = 0.005

rospy.init_node("grip", anonymous=True)

class LeftGrip:
	def __init__(self):
		self.uc = Uber()
		self.uc.open_gripper("l")
		self.position = INITIAL_POS
		self.gripping = False
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
			self.uc.command_gripper("l", self.position, 15, blocking=True, timeout=2)
		rospy.sleep(0.01)

lg = LeftGrip()
rospy.spin()