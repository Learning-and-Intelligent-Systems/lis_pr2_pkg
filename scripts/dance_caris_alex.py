#!/usr/bin/env python
"""
Feedback: you need to make your file executable and add the following
to the top of your python program:

#!/usr/bin/env python
 -- you used an invalid interpreter

You don't need to import all those packages and you did not use 
the uber controller pacakge to execute joint commands

Your code is over 80 charaters wide 

Did not meet naming guidelines of scripts/dance_[yourname].py

"""




#XXX this is not a valid interpreter: #!/bin/env python
#XXX remove import roslib; 
#XXX removeimport pdb
#XXX remove from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
#XXX remove from trajectory_msgs.msg import JointTrajectoryPoint
#XXX removeimport actionlib
import rospy
#XXX removeimport time
from dance_demos import Head, Arm
from lis_pr2_pkg.uber_controller import Uber

pi = 3.14159

# a config is a tuple of 3 lists
# ([6 arm1 joint angles], [6 arm2 joint angles], [head_pan, head_tilt])

rumble_configs = { 'step0': ([-pi/2] + [0] * 6, [pi/2] + [0] * 6, [0] * 2),
                'step1': ([-pi, -0.5, 0, 1.379809650196403, 0, 0.03054326190991663, 0.08614766179720057],[0.53, -0.4, pi/2, -3*pi/4, -0.05935111121127434, -0.6313579395652136, -0.11586425422522062], [0.7340854833888151, 1]),
                'step2': ([-pi/2, 0, 0, 0, 0, 0, 0 ],[pi/2, 0, pi, -pi, 0, 0, 0 ],[0, 0]),
                'step3': ([-pi/2, -pi/2, 0, -pi, 0, -pi/6, 0 ],[pi/2, 0, pi, -pi, 0, 0, 0 ],[0, 0]),
                'step4': ([-pi, -0.5, 0, 1.379809650196403, 0, 0.03054326190991663, 0.08614766179720057],[0.53, -0.4, pi/2, -3*pi/4, -0.05935111121127434, -0.6313579395652136, -0.11586425422522062],[0.7340854833888151, 1])}
rumble_configs = {'step0':([-1.5831222505094336, -0.013139257002238076, -2.9627522898900116, -1.4650528060395591, 75.29444872984716, -0.20636052583923903, 81.76866505533432],[1.4689851313490287, -0.26852170015355603, 0.030925728608118375, -1.2626633515253871, -12.48459681210379, 0.01633910335368627, 18.792472118246753],(1.5917602939002817, 0.32316516429926995)),
        'step1':([-1.405121743043985, 0.15892681659222277, -3.0708314428633154, -0.23884488062107878, 75.31943867140981, -1.1324478901591082, 81.75800536990992],[1.4872245871675656, -0.2123506712121981, -0.03289549228709632, -0.5718175168031928, -12.4700771932792, 0.016991737155179876, 18.792515627166853],(1.4118517546047062, 0.009320058205649703)),
        'step2':([-1.41764064226489, -0.07201732348293842, 0.013753997187816225, -1.509497414434302, 75.42853827739856, -0.010004769429764804, 81.58423074303214],[1.5881219404910172, 0.004973686243145939, 3.1337916188654167, -1.530054855450763, -15.645188101821564, -0.146862855939896, 18.95045300712838],(-1.4902468190264249, -0.044401176170735766)),
'step3':([-1.3397913285666803, -0.25905669846688767, 0.014235061666925786, -0.429507907513185, 75.40962227996573, -0.0006938605284593224, 81.58396968951155],[1.5753543214180414, -0.006954197914582145, 3.1358762316082256, -0.3480019970042587, -15.648601079951648, 0.020080870482251156, 18.942664910430555],(-1.4392482982831505, 0.029112091923265398)),
'step4':([-1.3280185889019882, -0.19442617721508432, 0.1314544397433126, -1.5339636646581183, 75.15098795420978, -0.004392118736911255, 81.58714584067882],[1.5494045501853049, -0.24390457753015965, -0.12044922748505438, -1.2929204302045445, -12.672426303432259, 0.015338398191392044, 18.942969472871244],(-0.031186470944205513, -0.12482594810263452)),
'step5':([-1.3696377108151951, -0.21024542783561742, 0.19623778959674665, -0.9404037479856543, 74.96304276870741, -0.004392118736911255, 81.58714584067882],[1.559353344268143, -0.31428755355306603, 0.10917888387662278, -0.6249483678809475, -12.68272308490946, 0.015338398191392044, 18.942969472871244],(-0.06626758890929157, 0.1380206372477116)),
'step6':([-1.3378844763674695, -0.3008465904804883, 0.19623778959674665, -0.2811179283450691, 74.96304276870741, -0.0034784314148358364, 81.58666724255772],[1.560348223676427, -0.3586152932598001, 0.11334810936223993, -0.25578305385295164, -12.676128516997096, 0.015338398191392044, 18.942969472871244],(1.1159137266365478, -0.009320058205649776)),
'step7':([-1.335065651377332, -0.30152334986532386, 0.19623778959674665, -0.2767748070035633, 74.96304276870741, -0.0034784314148358364, 81.58666724255772], [1.559850783972285, -0.3595458374139493, 0.11334810936223993, -0.25115039108867865, -12.676128516997096, 0.015338398191392044, 18.942969472871244],(-0.7038014580777803, -0.010995574287564258))}

rumble_times = [0.5]*8


#These two functions are not exactly the same 
def d_step(dance_steps, dance_times, a1,a2, h, n):
    #i = n-1
    a1.move(dance_steps['step' + str(n)][0], dance_times[n], True)
    a2.move(dance_steps['step' + str(n)][1], dance_times[n], True)
    h.move(dance_steps['step' + str(n)][2], dance_times[n], True)
    
def rumble():
    arm1 = Arm('r_arm')
    arm2 = Arm('l_arm')
    head = Head()
    
    for n in range(len(rumble_times)):
    #for i in range(1):
        # why add 1 then subtract in d_step function?
        d_step(rumble_configs, rumble_times, arm1, arm2, head,  n)
        #if i == 0:
        #        time.sleep(4)
        #elif i == 3:
        #        time.sleep(0.5)
        #else:
        #        time.sleep(1.5)

def get_dance_configs():
    uc  = Uber()
    done = False
    while not done:
        raw_input('When robot is in desired config press ENTER')

        r_config = uc.get_joint_positions('r')
        print 'The current right arm config is: ', r_config

        l_config = uc.get_joint_positions('l')
        print 'The current left arm config is: ', l_config

        head_config = uc.get_head_pose()
        print 'The current head config is: ', head_config

        answer = raw_input('Enter y if done, else hit ENTER')
        if answer == 'y':
            done = True

if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    #raw_input('Are you ready to rumble??? (Hit ENTER)')
    rumble()
    #get_dance_configs()
