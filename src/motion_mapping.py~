#!/usr/bin/env python  
#mapping using frames
import rospy
import tf as tf
#import angle_calculator as acalc
from move_joint import Arm
import roslib
import numpy as np

if __name__ == '__main__':
    rospy.init_node('tf_elbow')
    listener = tf.TransformListener()
    #Arm naming is inverted in skeltracker 
    rate = rospy.Rate(10.0)
    r_arm = Arm('r_arm')
    l_arm = Arm('l_arm')
    r_arm.move([0] * 7)
    l_arm.move([0] * 7)
    num = 1
    r_roll_angle = 0
    r_pan_angle = 0
    r_lift_angle = 0
    l_roll_angle = 0
    l_pan_angle = 0
    l_lift_angle = 0
    pi = np.pi
    npi = -1 * np.pi
    keep_going = True
    pic = (180/pi)
    while not rospy.is_shutdown():
        try:

            t = listener.getLatestCommonTime('torso_' + str(num), 'left_shoulder_' + str(num))
            '''
	    Using triangles
	    '''
            #(C_rval,c_rrot) = listener.lookupTransform('/left_shoulder_1', '/left_hand_1', t)
            #(B_rval, b_rrot) = listener.lookupTransform("/left_shoulder_" + str(num) , "/left_elbow_" + str(num) , t)
            #(A_rval, a_rrot) = listener.lookupTransform("/left_elbow_" + str(num) , "/left_hand_" + str(num) , t)
            #(C_lval,c_lrot) = listener.lookupTransform('/right_shoulder_1', '/right_hand_1', t)
            #(B_lval, b_lrot) = listener.lookupTransform("/right_shoulder_" + str(num) , "/right_elbow_" + str(num) , t)
            #(A_lval, a_lrot) = listener.lookupTransform("/right_elbow_" + str(num) , "/right_hand_" + str(num) , t)
            #r_elbow_angle = acalc.law_of_cosines(A_rval, B_rval, C_rval) - np.pi 
            #l_elbow_angle = acalc.law_of_cosines(A_lval, B_lval, C_lval) - np.pi
	    #l_elbow_angle = -1 * b_lrot
	    '''
	    Using frames
	    '''
            ( _ , r_elbow_quaternion) = listener.lookupTransform("/left_shoulder_" + str(num) , "/left_elbow_" + str(num) , t) 
            ( _ , l_elbow_quaternion) = listener.lookupTransform("/right_shoulder_" + str(num) , "/right_elbow_" + str(num) , t)
	    ( _ , r_roll_quaternion) = listener.lookupTransform("/torso_" + str(num), "/left_shoulder_" + str(num) , t)
	    ( _ , l_roll_quaternion) = listener.lookupTransform("/torso_" + str(num), "/right_shoulder_" + str(num) , t)
       
	    (_, r_elbow_angle, _) = tf.transformations.euler_from_quaternion(r_elbow_quaternion)
	    (_, l_elbow_angle, _) = tf.transformations.euler_from_quaternion(l_elbow_quaternion)	    
	    (r_roll_val, r_pan_val, r_lift_val) = tf.transformations.euler_from_quaternion(r_roll_quaternion)
	    (l_roll_val, l_pan_val, l_lift_val) = tf.transformations.euler_from_quaternion(l_roll_quaternion)
		
	    r_pan_angle = r_pan_val - np.pi/2
	    r_roll_angle = r_roll_val - np.pi/2
	    r_lift_angle = -1 * r_lift_val
	    if r_elbow_quaternion[1] < 0.7:
		r_elbow_angle = pi - r_elbow_angle
	    '''
	    if l_elbow_quaternion[1] > -0.7:
		l_elbow_angle = npi - l_elbow_angle
	   
	    print "Elbow y-quaternion is: ", l_elbow_quaternion[1]
	    print "Elbow robot angle is: ", l_elbow_angle * (180/pi)
	    '''
	
	    #print "The pan value is: ", r_pan_val * pic 
            #r_arm_angles = [r_pan_angle] + [r_lift_angle] + [r_roll_angle] + [0] * 4
            r_arm_angles = [r_pan_angle] + [max(r_lift_angle, -0.53) ] + [r_roll_angle] + [max(round(r_elbow_angle - pi, 2), -2)] + [0]*3 
	    #print max((r_elbow_angle - pi), -2)
	    #print -1 * r_elbow_angle
            #r_arm_angles = [0] * 3 + [max(-2, -1 * elbow_map * r_elbow_angle)] + [0]*3 
	    #print r_lift_val 
            #r_arm_angles = [0] * 1 + [r_lift_val] + [0]*5 
            #r_arm_angles = [0,0, r_roll_val] + [0] * 4
            #l_arm_angles = l_pan_angle + l_lift_angle + l_roll_angle + [max(l_elbow_angle, 2)] + [0]*3
            r_arm.move(r_arm_angles)
            #l_arm.move(l_arm_angles)
	    print "Moving"  

        except (tf.Exception):
            print "Human not detected"
            continue
 
        rate.sleep()
        
          
        
