#!/usr/bin/env python
#Author: Ivan Jutamulia
from lis_pr2_pkg.uber_controller import Uber
import rospy
import time

rospy.init_node("Dance_test")
rospy.loginfo("Ivan's dance")
uc = Uber()

# finding position parameters
training = False
if training:
  while True:
    input = raw_input('Press enter')
    if input != 'quit':
      print 'Left: %s, Right: %s'%(uc.get_joint_positions('l'), uc.get_joint_positions('r'))
      print ('Head:', uc.get_head_pose())
    else:
      break

def move_arms_to_front():
  left_initial = [-0.060724865504715675, -0.04400677423433334, 1.571935610022725, 0.008423494421986533, 7.85542236019144, -1.5718169372528075, 47.124129109964365]
  right_initial = [-0.09320742998704235, 0.001241879925519199, -1.5748812676586466, -0.0006970603951756971, -1.5713720799708144, -1.570870962157672, 59.69000916464502]
  uc.command_joint_pose('l', left_initial, time=2, blocking=True)
  uc.command_joint_pose('r', right_initial, time=2, blocking=True)

# initial positions
def initialize():
  uc.close_gripper('l')
  uc.close_gripper('r')
  move_arms_to_front()
  uc.look_forward()
  uc.lift_torso()


def dab():
  uc.down_torso()
  right = [0.21238636159080715, -0.5391504888657374, -1.0248642132099368, -1.6195231550857832, -2.198723737949643, -0.015427068597178573, 59.649110779751396]
  left = [1.816363858074799, -0.5357570622405283, 2.7779642591506395, 0.01739927852776546, 4.326865750381151, 0.009906344047846516, 50.3267337006553]
  uc.command_joint_pose('l', left, time=1, blocking=False)
  uc.command_joint_pose('r', right, time=1, blocking=False)
  uc.command_head([-0.32084131360518436, 1.410575101461817], time=1, blocking=False)


def dance():
  initialize()

  left_poses = [[-0.060393239035287705, 0.6932379806209892, 1.5716149003699853, -0.14734979102668921, 7.854843889321934, -1.571599392652312, 47.124085601044264],
                [-0.06022742580057372, -0.5331346196242902, 1.5716149003699853, -0.14459914751040248, 7.8553066660175395, -1.572469571054301, 47.124085601044264],
                [0.023756977582052752, -0.26234627076693084, 0.8793631149311865, -1.3915092846567267, 15.48545312897037, 0.022001823835535617, 51.91254682044532],
                [0.17431539470233892, -0.17005320965997078, -0.7868838858783012, -1.270336199228714, 15.289004421686267, 0.021697261394842626, 51.91224225800463]]
  right_poses = [[-0.09246127043082963, -0.5001522293145836, -1.5748812676586466, -0.1561808044210844, -1.5713720799708144, -1.571001488917977, 59.68996565572492],
                [-0.09411940277796904, 0.8361091760433823, -1.5750416224850163, -0.14141419185996496, -1.5712563857969133, -1.569870256995392, 59.68987863788472],
                [-0.02281971185096132, -0.06457297024974652, 0.6801886555810635, -1.4346509566490182, -1.334372564734368, 0.01028670318168945, 59.72338050636141],
                [0.3506745993422594, -0.19628726552338244, -0.5245571549358916, -1.243408846911378, -1.8150818572935257, 0.013767416789649434, 59.34267745548995]]
  head_poses = [[-1, 0], [1, 0], [0, -1], [0, 1]]
  
  for i in range(4):
    uc.command_joint_pose('l', left_poses[0], time=1, blocking=False)
    uc.command_joint_pose('r', right_poses[0], time=1, blocking=False)
    uc.command_head(head_poses[0], time=1, blocking=False)
    time.sleep(1.25)
    uc.command_joint_pose('l', left_poses[1], time=1, blocking=False)
    uc.command_joint_pose('r', right_poses[1], time=1, blocking=False)
    uc.command_head(head_poses[1], time=1, blocking=False)
    time.sleep(1.25)
  for i in range(4):
    uc.command_joint_pose('l', left_poses[2], time=1, blocking=False)
    uc.command_joint_pose('r', right_poses[2], time=1, blocking=False)
    uc.command_head(head_poses[2], time=1, blocking=False)
    time.sleep(1.25)
    uc.command_joint_pose('l', left_poses[3], time=1, blocking=False)
    uc.command_joint_pose('r', right_poses[3], time=1, blocking=False)
    uc.command_head(head_poses[3], time=1, blocking=False)
    time.sleep(1.25)

  uc.command_head([0,0], 1, blocking=False)
  time.sleep(0.25)
  dab()

dance()