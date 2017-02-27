#!/usr/bin/env python
#Author: Zi Wang
from lis_pr2_pkg.uber_controller import Uber
import rospy
import numpy as np
import csv

is_record = 0
rospy.init_node("test_dance")
rospy.loginfo("Start dancing")
uc = Uber()

# initialize positions
uc.close_gripper('l')
uc.close_gripper('r')
uc.move_arm_to_side("l")
uc.move_arm_to_side("r")
uc.look_forward()

posefile = '/home/demo/catkin_ws/src/lis_pr2_pkg/scripts/zi_poses.csv'

# record positions
joint_poses = []
cnt = 0
while is_record and cnt < 10:
  joints = []
  joints += uc.get_joint_positions('l')
  joints += uc.get_joint_positions('r')
  #print uc.return_cartesian_pose('l')
  #print uc.return_cartesian_pose('r')
  joints += uc.get_head_pose()
  joint_poses.append(joints)
  raw_input('hey')
  cnt += 1

if is_record:
  with open(posefile, 'wb') as file:
    writer = csv.writer(file)
    for joints in joint_poses:
      writer.writerow(joints)


# start dance
with open(posefile, 'rb') as file:
  joint_poses = csv.reader(file)
  for pose in joint_poses:
    pose = [float(item) for item in pose]
    print pose
    uc.command_joint_pose('l', pose[:7], time=2, blocking=False)
    uc.command_joint_pose('r', pose[7:14], time=2, blocking=False)
    uc.command_head(pose[14:16], time=2, blocking=False)
    uc.command_torso(np.random.random_sample(1)*0.2-0.08, timeout=2, blocking=True)
