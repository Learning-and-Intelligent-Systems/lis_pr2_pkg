#!/usr/bin/env python

import rospy
from pr2_msgs.msg import PressureState
import numpy as np
from lis_pr2_pkg.uber_controller import UberController

ROLL = 5
L_BASE = 2950
R_BASE = 1890
CLOSE = 1.2
STOP = 1.4

class X():
	def __init__(self):
		print "Opening gripper..."
		self.pos = 0.1
		self.uc = UberController()
		self.uc.command_gripper("r", self.pos, -1, blocking=True, timeout=2)
		print "Ready"
		self.l_avg = 0
		self.r_avg = 0
		self.closing = False
		self.sub = rospy.Subscriber("/pressure/r_gripper_motor", PressureState, self.cb, queue_size=1)

	def cb(self, data):
		left = np.array(data.l_finger_tip)
		right = np.array(data.r_finger_tip)
		self.l_avg = (1-1./ROLL)*self.l_avg + 1./ROLL*np.mean(left)
		self.r_avg = (1-1./ROLL)*self.r_avg + 1./ROLL*np.mean(right)
		if self.l_avg > L_BASE*CLOSE or self.r_avg > R_BASE*CLOSE:
			print "Closing gripper..."
			self.closing = True
		if self.l_avg > L_BASE*STOP or self.r_avg > R_BASE*STOP:
			self.closing = False
			print "Done"
			self.sub.unregister()
		if self.closing:
			self.pos = self.pos - 0.004
			if self.pos < 0:
				self.pos = 0.1
		                self.uc.command_gripper("r", self.pos, -1, blocking=True, timeout=2)
				self.closing = False
			self.uc.command_gripper("r", self.pos, 20, blocking=True, timeout=2)
		rospy.sleep(0.01)

if __name__ == "__main__":
	rospy.init_node("whee")
	X()
	rospy.spin()
