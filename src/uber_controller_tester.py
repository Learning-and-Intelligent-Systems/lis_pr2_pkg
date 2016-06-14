#!/usr/bin/env python
#Author: Ariel Anders
import rospy
from uber_controller import Uber
import dance_demos as dd
import time


def test_gripper():
    rospy.loginfo("testing open gripper commands")
    uc.close_gripper('l')
    uc.close_gripper('r')
    rospy.loginfo("grippers should be closed")
    uc.open_gripper('l')
    uc.open_gripper('r')
    rospy.loginfo("grippers should be open")

def test_head():
    rospy.loginfo("testing head command")
    uc.look_down_center()
    raw_input("look up")
    uc.look_forward()

def test_torso():
    rospy.loginfo("testing torso command")
    uc.lift_torso()
    raw_input("move torso down")
    uc.down_torso()

def test_joint():
    rospy.loginfo("testing joint control")
    uc.move_arm_to_side("l")
    uc.move_arm_to_side("r")

def test_gripper_event(): 
    rospy.loginfo("requesting gripper event!-- right")
    print uc.wait_for_gripper_event('r')
    rospy.loginfo("requesting gripper event!-- left")
    uc.wait_for_gripper_event('l')

def test_get_state():
    print "testing gathering state information"
    raw_input("get joint angles-- left")
    print uc.get_joint_positions('l')


    raw_input("get joint angles-- right")
    print uc.get_joint_positions('r')

    raw_input("get cartesian pose-- left")
    print uc.return_cartesian_pose('l', 'base_link')

    raw_input("get cartesian pose--right")
    print uc.return_cartesian_pose('r', 'base_link')


       


rospy.init_node("ubertest")
rospy.loginfo("how to use uber controller")
uc = Uber()
uc.command_head([0,0], 3, blocking = False) 
uc.command_joint_pose('l', [0]*7, 3, blocking=False)
uc.command_joint_pose('r', [0]*7, 3, blocking=False)
raw_input()
test_gripper()
test_head() 
test_torso()
test_joint()

#test_gripper_event()

def macerena():
    for i in range(len(dd.macerena_times)):
        head_pos = dd.macerena_configs['step' + str(i+1)][2]
        uc.command_head(dd.macerena_configs['step' + str(i+1)][2],dd.macerena_times[i], blocking=False)
        uc.command_joint_pose('r', dd.macerena_configs['step' + str(i+1)][0], dd.macerena_times[i], blocking = True)
        uc.command_joint_pose('l', dd.macerena_configs['step' + str(i+1)][1], dd.macerena_times[i], blocking = True)
        raw_input()
        
def dab():
    for i in range(len(dd.dab_times)):
        uc.command_joint_pose('r', dd.dab_configs['step' + str(i+1)][0], dd.dab_times[i], blocking = False)
        uc.command_joint_pose('l', dd.dab_configs['step' + str(i+1)][1], dd.dab_times[i], blocking = False)
        uc.command_head(dd.dab_configs['step' + str(i+1)][2],dd.dab_times[i], blocking=False)
        if i == 0:
                time.sleep(3)
        elif i == 3:
                time.sleep(0.5)
        else:
                time.sleep(1)


def dance():
    response = raw_input('Enter a dance for the PR2!' + '\n')
    dances = {'dab' : dab,
              'macerena': macerena}
    if response not in dances:
        print ("That dance is currently unavailable!")
    else:
        dances[response]()
    

if __name__ == '__main__':
    print("Testing")
        
