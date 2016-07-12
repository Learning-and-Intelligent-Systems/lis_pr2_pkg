#!/usr/bin/env python  
#mapping using triangles 
import rospy
import tf as tf
import angle_calculator as acalc
from move_joint import Arm
import roslib
import numpy as np
import cProfile
import re

if __name__ == '__main__':
    rospy.init_node('tf_elbow')

    listener = tf.TransformListener()
    #Arm naming is inverted in skeltracker 
    rate = rospy.Rate(10.0)
    r_arm = Arm('r_arm')
    l_arm = Arm('l_arm')
    r_arm.move([-np.pi/2, 0, 0, -np.pi/2, 0, 0, 0 ])
    l_arm.move([np.pi/2, 0, 0, -np.pi/2, 0, 0, 0 ])
    num = 1
    #r_arm.move([0] * 7)
    while not rospy.is_shutdown():
        try:

            t = listener.getLatestCommonTime('left_shoulder_' + str(num), 'right_shoulder_' + str(num))
            '''
	    Using triangles
	    '''
            (C_rval,c_rrot) = listener.lookupTransform('/left_shoulder_1', '/left_hand_1', t)
            (B_rval, b_rrot) = listener.lookupTransform("/left_shoulder_" + str(num) , "/left_elbow_" + str(num) , t)
            (A_rval, a_rrot) = listener.lookupTransform("/left_elbow_" + str(num) , "/left_hand_" + str(num) , t)
            #(C_lval,c_lrot) = listener.lookupTransform('/right_shoulder_1', '/right_hand_1', t)
            #(B_lval, b_lrot) = listener.lookupTransform("/right_shoulder_" + str(num) , "/right_elbow_" + str(num) , t)
            #(A_lval, a_lrot) = listener.lookupTransform("/right_elbow_" + str(num) , "/right_hand_" + str(num) , t)
            r_elbow_angle_t = acalc.law_of_cosines(A_rval, B_rval, C_rval) - np.pi 
            #l_elbow_angle_t = acalc.law_of_cosines(A_lval, B_lval, C_lval) - np.pi
	    #Comparison of two angle readings 
	    (_, r_elbow_angle_nt, _) = tf.transformations.euler_from_quaternion(b_rrot)
	    print "Triangle reading: ", (r_elbow_angle_t/ np.pi) * 180 
	    print "Frame reading: ", (-1 * r_elbow_angle_nt / np.pi) * 180 

        except (tf.Exception):
            print "Human not detected"
            continue
 
        rate.sleep()
