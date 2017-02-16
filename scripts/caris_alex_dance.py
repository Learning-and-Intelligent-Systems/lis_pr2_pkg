#!/bin/env python
import roslib; 
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import rospy
import time

pi = 3.14159

# a config is a tuple of 3 lists
# ([6 arm1 joint angles], [6 arm2 joint angles], [head_pan, head_tilt])

dab_configs = { 'step0': ([-pi/2] + [0] * 6, [pi/2] + [0] * 6, [0] * 2),
                'step1': ([-pi, -0.5, 0, 1.379809650196403, 0, 0.03054326190991663, 0.08614766179720057],[0.53, -0.4, pi/2, -3*pi/4, -0.05935111121127434, -0.6313579395652136, -0.11586425422522062], [0.7340854833888151, 1]),
                'step2': ([-pi/2, 0, 0, 0, 0, 0, 0 ],[pi/2, 0, pi, -pi, 0, 0, 0 ],[0, 0]),
                'step3': ([-pi/2, -pi/2, 0, -pi, 0, -pi/6, 0 ],[pi/2, 0, pi, -pi, 0, 0, 0 ],[0, 0]),
                'step4': ([-pi, -0.5, 0, 1.379809650196403, 0, 0.03054326190991663, 0.08614766179720057],[0.53, -0.4, pi/2, -3*pi/4, -0.05935111121127434, -0.6313579395652136, -0.11586425422522062],[0.7340854833888151, 1])}

dab_times = [4, 1.2, 1, 0.4, 1.5]

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
    #i = n-1
    a1.move(dance_steps['step' + str(n)][0], dance_times[n], True)
    a2.move(dance_steps['step' + str(n)][1], dance_times[n], True)
    h.move(dance_steps['step' + str(n)][2], dance_times[n], False)

#These two functions are not exactly the same 
def d_step(dance_steps, dance_times, a1,a2, h, n):
    #i = n-1
    a1.move(dance_steps['step' + str(n)][0], dance_times[n], False)
    a2.move(dance_steps['step' + str(n)][1], dance_times[n], False)
    h.move(dance_steps['step' + str(n)][2], dance_times[n], False)
    
def rumble():
    arm1 = Arm('r_arm')
    arm2 = Arm('l_arm')
    head = Head()
    
    for n in range(len(dab_crab_times)):
    #for i in range(1):
        # why add 1 then subtract in d_step function?
        d_step(dab_crab_configs, dab_crab_times, arm1, arm2, head,  n)
        #if i == 0:
        #        time.sleep(4)
        #elif i == 3:
        #        time.sleep(0.5)
        #else:
        #        time.sleep(1.5)

def get_dance_configs():

    done = False
    while not done:
        raw_input('When robot is in desired config press ENTER')

        r_config = uc.get_joint_positions('r')
        print 'The current right arm config is: ', r_config

        l_config = uc.get_joint_positions('l')
        print 'The current left arm config is: ', l_config

        head_config = uc.get_head_pose()
        print 'The current head config is: ', config

        answer = raw_input('Enter y if done, else hit ENTER')
        if answer == 'y':
            done = True

if __name__ == '__main__':
	rospy.init_node('joint_position_tester')
    raw_input('Are you ready to rumble??? (Hit ENTER)')
	#rumble()
    get_dance_configs()