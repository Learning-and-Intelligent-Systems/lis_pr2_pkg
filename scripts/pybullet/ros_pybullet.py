#!/usr/bin/env python
#Author: Skye Thompson
from lis_pr2_pkg.uber_controller import Uber
import rospy
import numpy as np
from pybullet_tools.utils import joint_from_name, connect
import pybullet as p
import pybullet_data
from lis_pr2_pkg.msg import uc_pr2 as uc_msg

k=1
ARM_JOINT_NAMES = {
   'left': ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
                 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint'],
    'right': ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                  'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'],
}

TORSO_JOINT_NAME = 'torso_lift_joint'

#Setup node
pub = rospy.Publisher('pybullet_cmds', uc_msg, queue_size=0)
rospy.init_node("pybullet_test")
rospy.loginfo("testing pybullet configurations")

#Setup controller
uc = Uber()
uc.command_joint_pose('l', [0]*7, time=2, blocking=True)

#Setup sim
physicsClient = connect(use_gui=True) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

pr2_start_orientation = p.getQuaternionFromEuler([0,0,0])
pr2_start_pose = [-.80*k,0,0]
pr2 = p.loadURDF("/home/demo/nishad/pddlstream/examples/pybullet/utils/models/pr2_description/pr2.urdf", pr2_start_pose, pr2_start_orientation, 
                      useFixedBase=True, globalScaling = 1 )

#The goal here is to get the arm positions of the actual robot using the uber controller
#and simulate them in pybullet

print str(uc.get_head_pose()) + ','

#Left Manipulator
left_joints = [joint_from_name(pr2, name) for name in ARM_JOINT_NAMES['left']]
l_joint_poses = uc.get_joint_positions('l')
#Right Manipulator
right_joints = [joint_from_name(pr2, name) for name in ARM_JOINT_NAMES['right']]
r_joint_poses = uc.get_joint_positions('r')
#Torso
torso_joint = 15
torso_pose = np.array([uc.get_torso_pose()])
#Head
#TODO

raw_input('Start?')
print
print

#Aligns the pybullet sim with the robot's state
for i in range(len(left_joints)):
    p.setJointMotorControl2(bodyIndex=pr2,jointIndex=left_joints[i],
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=l_joint_poses[i],
                                        targetVelocity=0,force=500,
                                        positionGain=1,velocityGain=0.1)
for i in range(len(right_joints)):
    p.setJointMotorControl2(bodyIndex=pr2,jointIndex=right_joints[i],
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=r_joint_poses[i],
                                        targetVelocity=0,force=500,
                                        positionGain=1,velocityGain=0.1)

#Tests that the robot can match the simulation position
#TODO: Move commands to different node so this doesn't slow down so much
rate = rospy.Rate(100)
while True:
    msg = uc_msg()

    p.stepSimulation()

    i = 0
    l_conf = []
    r_conf = []
    for joint in range(len(left_joints)):
        l_conf.append(p.getJointState(pr2, left_joints[joint])[0])

    for joint in range(len(right_joints)):
        r_conf.append(p.getJointState(pr2, right_joints[joint])[0])

    msg.l_shoulder_pan_joint = l_conf[0]
    msg.l_shoulder_lift_joint = l_conf[1]
    msg.l_upper_arm_roll_joint = l_conf[2]
    msg.l_elbow_flex_joint = l_conf[3]
    msg.l_forearm_roll_joint = l_conf[4]
    msg.l_wrist_flex_joint = l_conf[5]
    msg.l_wrist_roll_joint = l_conf[6]

    msg.r_shoulder_pan_joint = r_conf[0]
    msg.r_shoulder_lift_joint = r_conf[1]
    msg.r_upper_arm_roll_joint = r_conf[2]
    msg.r_elbow_flex_joint = r_conf[3]
    msg.r_forearm_roll_joint = r_conf[4]
    msg.r_wrist_flex_joint = r_conf[5]
    msg.r_wrist_roll_joint = r_conf[6]
    pub.publish(msg)
    rate.sleep()