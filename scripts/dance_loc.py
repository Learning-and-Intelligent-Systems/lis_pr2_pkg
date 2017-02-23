#!/usr/bin/env python
#Author: Loc Trinh

from lis_pr2_pkg.uber_controller import Uber
import numpy as np
import rospy

def reset():
    # reset robot to intial position #
    uc.look_forward()
    uc.down_torso()
    uc.close_gripper('l', False)
    uc.close_gripper('r')

def nod(blocking = True):
    # nod head #
    uc.command_head([0, np.pi/12.], 1, blocking = blocking)
    uc.command_head([0,0], 1, blocking = blocking)

def shrug(blocking = True):
    # shrug torso #
    uc.command_torso(0.2, blocking = blocking, timeout = 2)
    uc.command_torso(0.1, blocking = blocking, timeout = 2)


def dance():
    # dance baby dance #
    nod()
    nod()
    uc.lift_torso(False)
    nod()
    nod()
    nod()
    nod()
    uc.down_torso(False)
    uc.open_gripper('l', False)
    uc.open_gripper('r', False)
    nod()
    uc.close_gripper('l', False)
    uc.close_gripper('r', False)
    nod()
    uc.open_gripper('l', False)
    uc.open_gripper('r', False)
    nod()
    uc.close_gripper('l', False)
    uc.close_gripper('r', False)
    nod()
    uc.open_gripper('l', False)
    uc.open_gripper('r')
    uc.close_gripper('l', False)
    uc.close_gripper('r', False)
    nod()
    uc.open_gripper('l', False)
    uc.open_gripper('r')
    uc.close_gripper('l', False)
    uc.close_gripper('r', False)
    nod()

rospy.init_node("ubertest")
rospy.loginfo("Dance")
uc = Uber()
reset()
dance()
