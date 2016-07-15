#!/bin/env python
import roslib; 
#roslib.load_manifest('pr2_gripper_reactive_approach')
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import rospy
import time

pi = 3.14159

macarena_configs = {'step1': ([0, 0, 0, 0, -pi, 0, 0 ],[0, 0, 0, 0, pi, 0, 0 ],[0] * 2 ), 
                    'step2': ([0]*7, [0]*7,[0] * 2), 
                    'step3': ([0, 0, -pi/3., -2*pi/3., -pi/4., 0, 0 ],[0, 0, pi/3., -pi/2., pi/4, 0, 0 ],[0] * 2 ),
                    'step4': ([-pi/2, -pi/2, 0, -pi, 0, 0, 0 ],[pi/2, -pi/2, 0, -pi, 0, 0, 0 ] , [0] * 2),
                    'step5': ([-pi/4, pi/2, -2 * pi/3, -3*pi/5, -pi/4, 0, 0 ],[pi/8, pi/4, 2*pi/3, -2*pi/4, -pi/6, 0, 0 ],[0] * 2 ),
                    'step6': ([-pi/2, pi/14, -8*pi/9, -pi, 0, 0, 0 ],[pi/2, pi/14, 8*pi/9, -7*pi/8, 0, 0, 0 ],[0] * 2 ),
                    'step7': ([-pi/2, pi/14, -8*pi/9, -pi, 0, 0, pi ],[pi/2, pi/14, 8*pi/9, -7*pi/8, 0, 0, pi ],[0] * 2 )}

macarena_times = [3, 2, 2, 2.5, 2.5, 1.5, 1]

dab_configs = { 'step1': ([-pi/2] + [0] * 6, [pi/2] + [0] * 6, [0] * 2),
                'step2': ([-pi, -0.5, 0, 1.379809650196403, 0, 0.03054326190991663, 0.08614766179720057],[0.53, -0.4, pi/2, -3*pi/4, -0.05935111121127434, -0.6313579395652136, -0.11586425422522062], [0.7340854833888151, 1]),
                'step3': ([-pi/2, 0, 0, 0, 0, 0, 0 ],[pi/2, 0, pi, -pi, 0, 0, 0 ],[0, 0]),
                'step4': ([-pi/2, -pi/2, 0, -pi, 0, -pi/6, 0 ],[pi/2, 0, pi, -pi, 0, 0, 0 ],[0, 0]),
                'step5': ([-pi, -0.5, 0, 1.379809650196403, 0, 0.03054326190991663, 0.08614766179720057],[0.53, -0.4, pi/2, -3*pi/4, -0.05935111121127434, -0.6313579395652136, -0.11586425422522062],[0.7340854833888151, 1])}

dab_crab_configs = { 'step1': ([-pi/2] + [0] * 6, [pi/2] + [0] * 6, [0] * 2),
                'step2': ([-pi, -0.5, 0, 1.379809650196403, 0, 0.03054326190991663, 0.08614766179720057],[0.53, -0.4, pi/2, -3*pi/4, -0.05935111121127434, -0.6313579395652136, -0.11586425422522062], [0.7340854833888151, 1]),
                'step3': ([pi/8, pi/6, 0, 0, -pi, 0, 0 ],[0, 0, 0, -pi/6, pi, 0, 0 ],[0, 0]),
                'step4': ([pi/8, pi/6, 0, 0, -pi, 0, 0 ],[-pi/8, 0, 0, 0, pi, 0, 0 ],[0, 0]),
                'step5': ([-pi/2, -pi/2, 0, -pi, 0, -pi/6, 0 ],[pi/2, 0, pi, -pi, 0, 0, 0 ],[0, 0]),
                'step6': ([-pi, -0.5, 0, 1.379809650196403, 0, 0.03054326190991663, 0.08614766179720057],[0.53, -0.4, pi/2, -3*pi/4, -0.05935111121127434, -0.6313579395652136, -0.11586425422522062],[0.7340854833888151, 1])}

#'right_arm'
#'left_arm'
#'head_pan_joint', 'head_tilt_joint'
dab_times = [4, 1.2, 1, 0.4, 1.5]

dab_crab_times = [4, 1.2, 1, 0.4, 2, 1.5]

class Head:
    def __init__(self):
	    self.jta = actionlib.SimpleActionClient('/head_traj_controller/joint_trajectory_action',
						    JointTrajectoryAction)
	    rospy.loginfo('Waiting for joint trajectory action')
	    self.jta.wait_for_server()
	    rospy.loginfo('Found joint trajectory action!')
	    
    def move(self, angles, time, blocking):
	    goal = JointTrajectoryGoal()
	    goal.trajectory.joint_names = ['head_pan_joint',
					   'head_tilt_joint']
	    point = JointTrajectoryPoint()
	    point.positions = angles
	    point.time_from_start = rospy.Duration(time)
	    goal.trajectory.points.append(point)
	    if blocking:
                self.jta.send_goal_and_wait(goal)
            else:
                self.jta.send_goal(goal)
	

class Arm:
	def __init__(self, arm_name):
	    #arm_name should be l_arm or r_arm
	    self.name = arm_name
	    self.jta = actionlib.SimpleActionClient('/'+arm_name+'_controller/joint_trajectory_action',
						    JointTrajectoryAction)
	    rospy.loginfo('Waiting for joint trajectory action')
	    self.jta.wait_for_server()
	    rospy.loginfo('Found joint trajectory action!')

	def move(self, angles, time, blocking):
	    goal = JointTrajectoryGoal()
	    char = self.name[0] #either 'r' or 'l'
	    goal.trajectory.joint_names = [char+'_shoulder_pan_joint',
					   char+'_shoulder_lift_joint',
					   char+'_upper_arm_roll_joint',
					   char+'_elbow_flex_joint',
					   char+'_forearm_roll_joint',
					   char+'_wrist_flex_joint',
					   char+'_wrist_roll_joint']
	    point = JointTrajectoryPoint()
	    point.positions = angles
	    point.time_from_start = rospy.Duration(time)
	    goal.trajectory.points.append(point)
	    if blocking:
                self.jta.send_goal_and_wait(goal)
            else:
                self.jta.send_goal(goal)
	   



def m_step(dance_steps, dance_times, a1,a2, h, n):
    i = n-1
    a1.move(dance_steps['step' + str(n)][0], dance_times[i], True)
    a2.move(dance_steps['step' + str(n)][1], dance_times[i], True)
    h.move(dance_steps['step' + str(n)][2], dance_times[i], False)

#These two functions are not exactly the same 
def d_step(dance_steps, dance_times, a1,a2, h, n):
    i = n-1
    a1.move(dance_steps['step' + str(n)][0], dance_times[i], False)
    a2.move(dance_steps['step' + str(n)][1], dance_times[i], False)
    h.move(dance_steps['step' + str(n)][2], dance_times[i], False)

    
def macarena():
    arm1 = Arm('r_arm')
    arm2 = Arm('l_arm')
    head = Head()
    
    for i in range(len(macerena_times)):
        m_step(macerena_configs, macerena_times, arm1, arm2, head, i + 1)

def dab():
    arm1 = Arm('r_arm')
    arm2 = Arm('l_arm')
    head = Head()
    
    for i in range(len(dab_times)):
    #for i in range(1):
        d_step(dab_configs, dab_times, arm1, arm2, head,  i + 1)
        if i == 0:
                time.sleep(4)
        elif i == 3:
                time.sleep(0.5)
        else:
                time.sleep(1.5)

        
def dab_crab():
    arm1 = Arm('r_arm')
    arm2 = Arm('l_arm')
    head = Head()
    
    for i in range(len(dab_crab_times)):
    #for i in range(1):
        d_step(dab_crab_configs, dab_crab_times, arm1, arm2, head,  i + 1)
        if i == 0:
                time.sleep(4)
        elif i == 3:
                time.sleep(0.5)
        else:
                time.sleep(1.5)

def main():
    response = raw_input('Enter a dance for the PR2!')
    dances = {'dab' : dab,
              'macarena': macarena,
	      'dab crab' : dab_crab}
    if response not in dances:
        print ("That dance is currently unavailable!")
    else:
        dances[response]()
    

if __name__ == '__main__':
	rospy.init_node('joint_position_tester')
	main()
