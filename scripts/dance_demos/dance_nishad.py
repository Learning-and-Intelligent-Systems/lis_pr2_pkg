#!/usr/bin/env python
#Author: Nishad Gothoskar
from lis_pr2_pkg.uber_controller import Uber
import rospy
import time
rospy.init_node("test_dance")
rospy.loginfo("Nishad Routine")
uc = Uber()

training = False
if training:
	while True:
		raw_input('Set Position and hit key')
		print 'Left: %s, Right: %s'%(uc.get_joint_positions('l'), uc.get_joint_positions('r'))
		print (uc.get_head_pose())

# initialize positions
uc.close_gripper('l')
uc.close_gripper('r')
uc.open_gripper('l')
uc.open_gripper('r')
uc.move_arm_to_side('l')
uc.move_arm_to_side('r')
uc.look_forward()

left_hand_positions = [[0.09422760233549077, 0.2728012127918493, 0.1659444924115625, -0.011844405171707262, 1.2417649091338576, -0.09903999188153156, 28.048339235018787],
					   [0.08601984721714917, -0.5364338216253639, 0.1454190746362174, -0.1877408195026936, 1.2383519310037745, -0.09873542944083502, 28.04803467257809]]
right_hand_positions = [[-0.368789026081663, -0.4699518417662933, 0.05592731652309624, -0.43877323304173077, -7.511689438923872, -0.09765892758541128, 15.317959108114861],
                        [-0.17926449880359352, 0.348757824038619, 0.054965187564877116, -0.03254661689955185, -7.511400203489119, 0.007328096614910851, 15.309561886535638]]
head_motion = [(0, .4),(-.8, 0),(0, -.4),(.8, 0)]

for step in range(10):
	uc.command_joint_pose('l', left_hand_positions[step % 2], time=2, blocking=False)
	uc.command_joint_pose('r', right_hand_positions[step % 2], time=2, blocking=False)
	uc.command_head(head_motion[step % 4], 2, blocking=False)
	time.sleep(1.5)