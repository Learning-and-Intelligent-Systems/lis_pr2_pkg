#!/usr/bin/env python
#Author: Skye Thompson
from lis_pr2_pkg.uber_controller import Uber
import rospy
from lis_pr2_pkg.msg import uc_pr2 as uc_msg

global uc

def callback(data):
	left_arm_cmd = [data.l_shoulder_pan_joint,
					data.l_shoulder_lift_joint,
					data.l_upper_arm_roll_joint,
					data.l_elbow_flex_joint,
					data.l_forearm_roll_joint,
					data.l_wrist_flex_joint,
					data.l_wrist_roll_joint,
				   ]
	right_arm_cmd = [data.r_shoulder_pan_joint,
					data.r_shoulder_lift_joint,
					data.r_upper_arm_roll_joint,
					data.r_elbow_flex_joint,
					data.r_forearm_roll_joint,
					data.r_wrist_flex_joint,
					data.r_wrist_roll_joint,
				   ]
	global uc
	uc.command_joint_pose('l', left_arm_cmd, time=2, blocking=True)
	uc.command_joint_pose('r', right_arm_cmd, time=2, blocking=True)
	
def listener():
	rospy.init_node('pr2_ctrl_listener', anonymous=True)
	global uc
	uc = Uber()
	rospy.Subscriber("pybullet_cmds", uc_msg, callback)
	rospy.spin()

if __name__ == '__main__':
	
	listener()