#!/usr/bin/env python
'''
----------------------------------------------
-- move the robot

-- Jingxi Xu
----------------------------------------------
'''

from lis_pr2_pkg.uber_controller import Uber
import rospy
import numpy as np
import time 
import csv

rospy.init_node("ubertest")
rospy.loginfo("motion")
uc = Uber()

# initial
uc.look_forward()
uc.move_arm_to_side("l")
uc.move_arm_to_side("r")
