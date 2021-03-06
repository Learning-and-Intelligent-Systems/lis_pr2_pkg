#!/usr/bin/env python
"""
Feedback: you need to make your file executable and add the following
to the top of your python program:

#!/usr/bin/env python

You don't need to import all those packages and you did not use 
the uber controller pacakge to execute joint commands

Your code is over 80 charaters wide 

Did not meet naming guidelines of scripts/dance_[yourname].py

"""

#XXX remove import actionlib
import rospy
#XXX remove import time
from dance_demos import Head, Arm
#XXX removeimport roslib
#XXX remove import pdb
#XXX removefrom pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
#XXX removefrom trajectory_msgs.msg import JointTrajectoryPoint
from lis_pr2_pkg.uber_controller import Uber

rospy.init_node('hello')

'''
uc = Uber()
r_config = uc.get_joint_positions('r')
l_config = uc.get_joint_positions('l')

print r_config, l_config

'''
rarm = Arm('r_arm')
larm = Arm('l_arm')
head = Head()

moves = [

[[-0.4915737263873589, 0.04218582270807525, -2.009923911600148, -1.9388873443978452, -5.2395137105928065, -0.07339726601487306, -2.7596718652049583], [0.047634083380864944, 0.04997818533471578, 1.3451938855357057, -1.6528204187039948, 1.461121062850383, -0.10223789750885592, -6.096414044445326]] 
,


[[-0.438596397896245, -0.28790356724550686, -1.8774708250186216, -2.0644035511673637, -5.241364817375223, -0.07687797962284082, -2.759932918725556], [0.19827540711850766, -0.07835231301474187, 1.1947810584007525, -1.0887937271537704, 0.9433896346428664, -0.5796177688416054, -5.379038969843214]]
,
[
[-0.4507007640303649, -0.2650629380073041, -1.6366178758110528, -1.9946240682805034, -4.220479426871732, -0.07248357869278166, -2.7592802849240616], [0.11793889489958875, 0.3005483475701106, 1.5655214169679297, -0.9470632007092966, 1.553271472362626, -0.4070178828065144, -5.6664153871010345]
]
,
[

[-0.39938156788639073, -0.28587328909099996, -1.6212238124795437, -1.9074721000276198, -4.217992002132858, -0.07209199841188518, -2.758627651122568] , [0.10633196846961068, 0.3660248180529585, 1.6479437977220508, -1.588252681426941, 1.5526351544061698, -0.40497296356183343, -5.599977266108954]]]
time_step = 0.5

print len(moves)
for loop in range(3):
    for i in range(len(moves)):
        larm.move(moves[i][1], time_step, True)
        rarm.move(moves[i][0], time_step, True)
        
    for i in range(len(moves)-2,-1,-1):
        rarm.move(moves[i][0], time_step, True)
        larm.move(moves[i][1], time_step, True)
