#!/usr/bin/env python
#Author: Skye Thompson
from lis_pr2_pkg.uber_controller import Uber
import rospy
import time
import numpy as np
rospy.init_node("test_dance")
rospy.loginfo("Skye Routine")
uc = Uber()

# initial
uc.close_gripper('l')
uc.close_gripper('r')
uc.command_joint_pose('l', [0]*7, time=2, blocking=True)
uc.command_joint_pose('r', [0]*7, time=2, blocking=True)
uc.look_forward()

for i in range(2):

	#Head right, Left arm up, Right arm down
	uc.command_head([-1, 0], time=1, blocking=False)
	l = [1, 0, 0, -1.5, 0, 0, 0]
	uc.command_joint_pose('l', l, time=2, blocking=True)
	r = [-1, 0, -np.pi, -1.5, 0, 0, 0]
	uc.command_joint_pose('r', r, time=2, blocking=True)
	time.sleep(2)

	#Flatten and bend left arm
	l = [1, 0, 0, 0, 0, 0, 0]
	uc.command_joint_pose('l', l, time=2, blocking=True)
	time.sleep(1.5)
	l = [1, 0, 0, -1.5, 0, 0, 0]
	uc.command_joint_pose('l', l, time=2, blocking=True)
	time.sleep(1.5)

	#Head left, Left arm down, Right arm up
	uc.command_head([1, 0], time=1, blocking=False)
	time.sleep(2)
	l = [1, 0, np.pi, -1.5, np.pi/2, 0, 0]
	uc.command_joint_pose('l', l, time=2, blocking=True)
	r = [-1, 0, 0, -1.5, 0, 0, 0]
	uc.command_joint_pose('r', r, time=2, blocking=True)
	time.sleep(2)

	#Head down, left arm "swings"
	uc.command_head([1, 1], time=1, blocking=False)
	time.sleep(1)
	l = [1, 0, np.pi, -2, np.pi/2, 0, 0]
	uc.command_joint_pose('l', l, time=2, blocking=True)
	time.sleep(.5)
	l = [1, 0, np.pi, -1, np.pi/2, 0, 0]
	uc.command_joint_pose('l', l, time=2, blocking=True)
	time.sleep(.5)
	l = [1, 0, np.pi, -1.5, np.pi/2, 0, 0]
	uc.command_joint_pose('l', l, time=2, blocking=True)
	time.sleep(1)
