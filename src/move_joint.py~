#!/bin/env python
import roslib; 
#roslib.load_manifest('pr2_gripper_reactive_approach')
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import rospy

class Arm:
	def __init__(self, arm_name):
	    #arm_name should be l_arm or r_arm
	    self.count = 0
	    self.name = arm_name
	    self.jta = actionlib.SimpleActionClient('/'+arm_name+'_controller/joint_trajectory_action',
						    JointTrajectoryAction)
	    rospy.loginfo('Please remain still while activating')
	    self.jta.wait_for_server(timeout = rospy.Duration(3))
	    rospy.loginfo('Start moving!')

	def move(self, angles):
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
	    point.time_from_start = rospy.Duration(0.5)
	    goal.trajectory.points.append(point)
	    self.jta.send_goal(goal)

def main():
	arm = Arm('l_arm')
	arm.move([0.5]*7)


if __name__ == '__main__':
	rospy.init_node('joint_position_tester')

