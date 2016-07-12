#!/usr/bin/env python
#Author: Ariel Anders
import numpy as np
import rospy
import roslib;
from Tkinter import *
roslib.load_manifest("lis_pr2_pkg")
from pr2_controllers_msgs.msg import \
        JointTrajectoryAction, JointTrajectoryGoal, JointTrajectoryActionGoal,\
        SingleJointPositionAction, SingleJointPositionGoal,\
        SingleJointPositionActionGoal,\
        PointHeadAction, PointHeadGoal, PointHeadActionGoal,\
        Pr2GripperCommandAction, Pr2GripperCommandGoal,\
        Pr2GripperCommandActionGoal
        

from pr2_gripper_sensor_msgs.msg import * #XXX fix

from pr2_mechanism_msgs.srv import SwitchController, ListControllers
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient as SAC
from sensor_msgs.msg import JointState
import tf
import joint_angle_poses as jap
from actionlib_msgs.msg import GoalStatus as gs


class UberController:
    simple_clients = {
    'torso':(
        'torso_controller/position_joint_action',  
        SingleJointPositionAction),
    'head':('head_traj_controller/joint_trajectory_action',
	JointTrajectoryAction),
    'l_gripper': (
        'l_gripper_controller/gripper_action', 
        Pr2GripperCommandAction),
    'r_gripper': (
        'r_gripper_controller/gripper_action',
        Pr2GripperCommandAction),
    'r_joint': (
        'r_arm_controller/joint_trajectory_action',
        JointTrajectoryAction),
    'l_joint': (
        'l_arm_controller/joint_trajectory_action',
        JointTrajectoryAction ),
    'l_gripper_event':(
        'l_gripper_sensor_controller/event_detector',
        PR2GripperEventDetectorAction), 
    'r_gripper_event':(
        'r_gripper_sensor_controller/event_detector',
        PR2GripperEventDetectorAction)
    }

    #XXX do subscriber callback nicely

    """ 
    ============================================================================
                    Initializing all controllers    
    ============================================================================
    """ 


    def __init__(self):
        self.clients = {}
        self.joint_positions = {}
        self.tf_listener = tf.TransformListener()

        rospy.loginfo("starting simple action clients")
        for item in self.simple_clients:
            self.clients[item]= SAC(*self.simple_clients[item]) 
            rospy.loginfo("%s client started" % item )
        
        for item in self.clients:
            res = self.clients[item].wait_for_server(rospy.Duration(2))
            if res:
                rospy.loginfo("%s done initializing" % item )
            else:
                rospy.loginfo("Failed to start %s" % item )
        
            
        self.rate = rospy.Rate(10)
        rospy.loginfo("subscribing to state messages")
        
        self.joint_positons = {}
        self.joint_velocities = {}
        self.joint_sub = rospy.Subscriber(\
                "joint_states", JointState, self.jointCB)
        
        self.gripper_events_sub= {}
        self.gripper_event = {}
        self.gripper_events_sub['l'] = rospy.Subscriber(\
                "/l_gripper_sensor_controller/event_detector_state",\
                PR2GripperEventDetectorData, self.l_gripper_eventCB)
        self.gripper_events_sub['r'] = rospy.Subscriber(\
                "/r_gripper_sensor_controller/event_detector_state",\
                PR2GripperEventDetectorData, self.r_gripper_eventCB)
        
        self.finished_registering()
        rospy.loginfo("done initializing Uber Controller!") 

    """ 
    =============================================================== #XXX make these all nice :)
                    State subscriber callbacks    
    ===============================================================
    """ 

    def finished_registering(self):
        def not_ready():
            return (self.joint_positions.get('torso_lift_joint', None) == None)

        end_time = rospy.Time.now() + rospy.Duration (1)
        test = not_ready()
        while(not rospy.is_shutdown()and rospy.Time.now() <end_time and test):
            self.rate.sleep()
            test = not_ready()
        if not_ready():
            rospy.loginfo("Warning! did not complete subscribing") 
        else:
            rospy.loginfo("finished registering") 
    
    def jointCB(self,data):
        pos = dict(zip(data.name, data.position))
        vel = dict(zip(data.name, data.velocity))
        self.joint_positions = pos
        self.joint_velocities = vel 
    
    def r_gripper_eventCB(self, data):
        self.gripper_event['r'] = data

    def l_gripper_eventCB(self, data):
        self.gripper_event['l'] = data


    """ 
    ===============================================================
                   Get State information    
    ===============================================================
    """ 
 
    def get_torso_pose(self):
        return self.joint_positions['torso_lift_joint']

    def get_gripper_pose(self, arm):
        return self.joint_positions['%s_gripper_joint'%arm]

    def get_head_pose(self):
        head = (
                self.joint_positions['head_pan_joint'],
                self.joint_positions['head_tilt_joint'] )
        return head

    def get_gripper_event(self, arm):
        #This may not work until you subscribe to the gripper event 
        msg = self.gripper_event[arm]
        event = msg.trigger_conditions_met or msg.acceleration_event
        return event

    def get_accel(self, arm):
        return self.accel[arm]
        
    def get_joint_positions(self, arm):
        pos = [self.joint_positions[p] for p in self.get_joint_names(arm)]
        return pos
    
    def get_joint_velocities(self, arm):
        vel = [self.joint_velocities[p] for p in self.get_joint_names(arm)]
        return vel

    #return the current Cartesian pose of the gripper
    def return_cartesian_pose(self, arm, frame = 'base_link'):
        end_time = rospy.Time.now() + rospy.Duration(5)
        link = arm + "_gripper_tool_frame"
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            try:
                t = self.tf_listener.getLatestCommonTime(frame, link)
                (trans, rot) = self.tf_listener.lookupTransform(frame, link, t)
                return list(trans) , list(rot)
            except (tf.Exception, tf.ExtrapolationException):
                rospy.sleep(0.5)
                current_time = rospy.get_rostime()
                rospy.logerr(\
                "waiting for a tf transform between %s and %s"%\
                (frame, link))
        rospy.logerr("return_cartesian_pose waited 10 seconds tf\
                transform!  Returning None")
        return None, None

    """ 
    ===============================================================
                Send Comm2ands for Action Clients                
    ===============================================================
    """ 
    def send_command(self, client, goal, blocking=False, timeout=None):
        if client == 'head':
            if blocking:
                self.clients[client].send_goal_and_wait(goal)
            else:
                self.clients[client].send_goal(goal)
        else:
            self.clients[client].send_goal(goal)
            rospy.sleep(.1)
            rospy.loginfo("command sent to %s client" % client)
            status = 0
            if blocking: #XXX why isn't this perfect?
                end_time = rospy.Time.now() + rospy.Duration(timeout+ .1)
                while (
                        (not rospy.is_shutdown()) and\
                        (rospy.Time.now() < end_time) and\
                        (status < gs.SUCCEEDED) and\
                        (type(self.clients[client].action_client.last_status_msg) != type(None))):
                    status = self.clients[client].action_client.last_status_msg.status_list[-1].status #XXX get to 80
                    self.rate.sleep()
                if status >gs.SUCCEEDED:
                    rospy.loginfo("goal status achieved.  exiting")
                else:
                    rospy.loginfo("ending due to timeout")

            result = self.clients[client].get_result()
            return result
        return None

    def command_torso(self, pose, blocking, timeout):
        goal= SingleJointPositionGoal(
          position = pose, 
          min_duration=rospy.Duration(timeout),
          max_velocity=1.0)
        return self.send_command('torso', goal, blocking, timeout)
    
    def command_event_detector(self, arm, trigger, magnitude, blocking,timeout):
        goal = PR2GripperEventDetectorGoal()
        goal.command.trigger_conditions =  trigger
        goal.command.acceleration_trigger_magnitude=magnitude
        client = "%s_gripper_event"% arm
        return self.send_command(client, goal, blocking, timeout)

    def command_head(self, angles, time, blocking):
	    goal = JointTrajectoryGoal()
	    goal.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
	    point = JointTrajectoryPoint()
	    point.positions = angles
	    point.time_from_start = rospy.Duration(time)
	    goal.trajectory.points.append(point)
	    return self.send_command('head', goal, blocking, timeout=time)
    
    def command_gripper(self, arm, position, max_effort, blocking, timeout):
        goal = Pr2GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        client = "%s_gripper"% arm
        return self.send_command("%s_gripper"%arm, goal, blocking, timeout)
    
    """ 
    ===============================================================
                    Joint Control Commands 
    ===============================================================
    """ 
    # angles is a list of joint angles, times is a list of times from start
    def command_joint_trajectory(self, arm, angles, times, blocking):
        print angles
        timeout=times[-1] + 1.0

        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names =self.get_joint_names(arm)
        
        for (ang, t) in zip(angles, times):
            point = JointTrajectoryPoint()
            point.positions = ang
            point.time_from_start = rospy.Duration(t)
            goal.trajectory.points.append(point)
        goal.trajectory.header.stamp = rospy.Time.now()
        return self.send_command("%s_joint"%arm, goal, blocking, timeout)
    
    # for convience
    def command_joint_pose(self, arm, angles, time, blocking):
        #XXX test if at location
        joints = self.get_joint_positions(arm)
        raw = lambda x:( (x+np.pi) % 2*np.pi) - np.pi
        disp = sum( [abs( raw(x-y)) for (x,y) in zip (joints, angles) ])
        rospy.loginfo("displacement for joint pos is %s " % disp)
        if disp < 0.05:
            rospy.loginfo("already near position. not moving")
        else:
            return self.command_joint_trajectory(arm, [angles], [time],  blocking)

    def  get_joint_names (self, char):
        return [char+'_shoulder_pan_joint',
                char+'_shoulder_lift_joint',
                char+'_upper_arm_roll_joint',
                char+'_elbow_flex_joint',
                char+'_forearm_roll_joint',
                char+'_wrist_flex_joint',
                char+'_wrist_roll_joint' ]
     
""" 
============================================================================
            Uber builds on the UberController with set default values
            for easier use.  See executable in scripts for deta
from Tkinter import *

root = Tk()

def callback(event):
    print "You Pressed it!"

frame = Frame(root, width=100, height=100)
frame.bind("<F1>", callback)
frame.pack()

root.mainloop()ils on 
            how to use
============================================================================
""" 
             
class Uber(UberController):
    timeout = 3
    valid_key=False;
    arm_ = 'r'
    arm_tog = {'r':'l',
               'l':'r'}
    head_torso = "torso"
    h_t_tog = {'torso':'head',
               'head':'torso'}
    h_t_msg = 'WS (up and down)'
    h_t_msg_tog = {'torso':'WS (up and down)',
               'head':'WS (tilt) AD (pan)'}

    yaw = False 
    def freeze(self, arm):
        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names =self.get_joint_names(arm)
        goal.trajectory.header.stamp = rospy.Time.now()
        return self.send_command("%s_joint"%arm, goal, False)

    def set_timeout(self, time):
        self.timeout = time
    
    def wait_for_gripper_event(self,arm, timeout=None):
        if timeout == None: timeout = self.timeout
        #trigger = PR2GripperEventDetectorGoal().command.FINGER_SIDE_IMPACT_OR_ACC
        trigger = PR2GripperEventDetectorGoal().command.ACC
        magnitude = 4.0
        self.command_event_detector(arm, trigger, magnitude, True, timeout=timeout)
        return self.get2_gripper_event(arm)

    def request_gripper_event(self, arm):
        trigger = PR2GripperEventDetectorGoal().command.ACC
        magnitude = 4.0
        self.command_event_detector(arm, trigger, magnitude, False, self.timeout)

    def open_gripper(self, arm, blocking=True):
        self.command_gripper(arm, 0.1, -1.0, blocking=blocking, timeout=self.timeout)
    
    def close_gripper(self,arm, blocking=True):
        self.command_gripper(arm, 0, 100, blocking=blocking, timeout=self.timeout)

    def look_down_center(self, blocking=True):
        self.command_head([0,np.pi/6.], 3, blocking=blocking)

    def look_forward(self, blocking=True):
        self.command_head([0,0], 3, blocking=blocking)

    def lift_torso(self, blocking=True):
        self.command_torso(0.2, blocking=blocking, timeout=self.timeout)
    
    def down_torso(self, blocking=True):
        self.command_torso(0.1, blocking=blocking, timeout=self.timeout)

    def psi_pose(self, arm, blocking=True):
        l = [ 0.95, 0.0, 3*np.pi/2., -np.pi/2., -np.pi*1.5, -np.pi/2.0, np.pi]
        r = [ -0.7, 0.0, -3*np.pi/2., -np.pi/2., np.pi*1.5, -np.pi/2.0, np.pi]
        if arm == "l":
            self.command_joint_pose('l', l, self.timeout, blocking=blocking)
        elif arm =="r":
            self.command_joint_pose('r', r, self.timeout, blocking=blocking)
        
    def motion_mapping(r_angle, l_angle):
	uc.command_joint_pose("r", [0]*3 + [r_angle] + [0] * 3, 1, False)
        uc.command_joint_pose("l", [0]*3 + [l_angle] + [0] * 3, 1, False)

    def move_arm_to_side(self, arm, blocking=True):
        l = [ 0.95, 0.0, np.pi/2., -np.pi/2., -np.pi*1.5, -np.pi/2.0, np.pi]
        r = [ -0.7, 0.0, -np.pi/2., -np.pi/2., np.pi*1.5, -np.pi/2.0, np.pi]
        if arm == "l":
            self.command_joint_pose('l', l, self.timeout, blocking=blocking)
        elif arm =="r":
            self.command_joint_pose('r', r, self.timeout, blocking=blocking)
if __name__=="__main__":

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

    def psi_pose():
        uc.command_joint_pose("r", [-np.pi/2, 0, 0, -np.pi/2, 0, 0, 0 ], 3, False)
        uc.command_joint_pose("l", [np.pi/2, 0, 0, -np.pi/2, 0, 0, 0 ], 3, False)
	
    def pose_manager():
	print ("To exit enter 'e'")
	pose_num = raw_input("Enter a pose number: ")
	while (pose_num != 'e'):
		poses = jap.joint_angles
		print len(poses)
		if int(pose_num) in range(len(poses) + 1):
			uc.command_joint_pose("l", poses['pose_'+ pose_num][0], 3, False)
           		uc.command_joint_pose("r", poses['pose_'+ pose_num][1], 3, False)
		else:
			print("Pose number: " + pose_num + " has not been added to pose dictionary!")
		pose_num = raw_input("Enter a pose number: ")


    def keyboard_controller():
                
        print("Welcome to Uber Controller!")
        print("---------------------------")
        print("The TK window must always be in focus!" + "\n")
        print("ARM CONTROLS:")
        print("Use 'OP' to open/close gripper")
        print("Use 'UJ' to forward/back")
        print("Use 'HK' to left/right")
        print("Use 'YI' to up/down" + "\n")
        print("BASE CONTROLS:")        
        print("Use UHJK to move and YI to Yaw" + "\n")
        print("TOGGLE CONTROLS:")
        print("Use 'T' to toggle right arm (default) and left arm")
        print("Use 'SPACE' to toggle torso (default) and head")
        raw_input("Press Enter to begin")
        print("Reading from keyboard")
        print("---------------------------")

        root = Tk()
        
        '''
        The following four keys are for obtaining joint information
        '''
        
        def KEYCODE_1(event):
            print "C!"
        
        def KEYCODE_Q(event):
            print "Now Exiting!"
            root.quit()

        def KEYCODE_Shift(event):
            print "Yaw!"
             
        def KEYCODE_U(event):
            print "Moving "  + uc.arm_ + "arm Forward"

        def KEYCODE_J(event):
            print "Moving "  + uc.arm_ + "arm Back"
            
        def KEYCODE_H(event):
            print "Moving "  + uc.arm_ + "arm Left"

        def KEYCODE_K(event):
            print "Moving "  + uc.arm_ + "arm Right"

        def KEYCODE_Up(event):
            print "Moving Forward"

        def KEYCODE_Down(event):
            print "Moving Backward"
            
        def KEYCODE_Left(event):
            print "Moving Left"

        def KEYCODE_Right(event):
            print "Moving Right"
            
        def KEYCODE_Y(event):
            print "Moving "  + uc.arm_ + "arm Up"

        def KEYCODE_W(event):
            if uc.head_torso == "torso":
                current = uc.get_torso_pose()
                fully_up = (round(current, 3) > 0.185)
                if fully_up:
                    print "Torso is all the way up!"
                else:
                    print "Lifting Torso!"
                    uc.command_torso(current + 0.005, False, 0.6)
            else:
                current = uc.get_head_pose()
                print current
                current_pan = current[0]
                current_tilt = current[1]
                fully_tilted = (round(current_tilt, 2) == -0.38 )
                if fully_tilted:
                    print "Head is fully tilted backwards"
                else:
                    print "Tilting head backwards"
                    uc.command_head( [current_pan, current_tilt - 0.1], 0.5, False)

            
        def KEYCODE_A(event):
            print uc.get_head_pose()
            if uc.head_torso == "torso":
                print "Invalid Command!"
            else:
                current = uc.get_head_pose()
                current_pan = current[0]
                current_tilt = current[1]
                fully_panned = (round(current_pan, 3) > 2.84 )
                if fully_panned:
                    print "Head is fully turned to the left"
                else:
                    print "Turning head to the left"
                    uc.command_head( [current_pan + 0.1, current_tilt], 0.5, False)



        def KEYCODE_S(event):
            
             if uc.head_torso == "torso":
                current = uc.get_torso_pose()
                fully_down = (round(current, 3) < 0.12)
                if fully_down:
                    print "Torso is all the way down!"
                else:
                    print "Lowering Torso!"
                    uc.command_torso(current - 0.005, False, 0.6)
             else:
                current = uc.get_head_pose()
                current_pan = current[0]
                current_tilt = current[1]
                fully_tilted = (round(current_tilt, 3) > 1.3 )
                if fully_tilted:
                    print "Head is fully tilted forwards"
                else:
                    print "Tilting head forwards"
                    uc.command_head( [current_pan, current_tilt + 0.1], 0.5, False)
                    
        def KEYCODE_D(event):
           print uc.get_head_pose()
           if uc.head_torso == "torso":
                print "Invalid Command!"
           else:
                current = uc.get_head_pose()
                current_pan = current[0]
                current_tilt = current[1]
                fully_panned= (round(current_pan, 3) < -2.84 )
                if fully_panned:
                    print "Head is fully turned to the right"
                else:
                    print "Turning head to the right"
                    uc.command_head( [current_pan - 0.1, current_tilt], 0.5, False)


            

        def KEYCODE_I(event):
            print "Moving "  + uc.arm_ + "arm Down"
            
        def KEYCODE_T(event):
            uc.arm_ = uc.arm_tog[uc.arm_]
            print "Controlling " + uc.arm_ + "_arm" 

               
        def KEYCODE_SPACE(event):
            uc.head_torso = uc.h_t_tog[uc.head_torso]
            uc.h_t_msg = uc.h_t_msg_tog[uc.head_torso]
            print "Controlling " + uc.head_torso + " [" + uc.h_t_msg + "]"
            
        def KEYCODE_Shift_Left(event):
            print "Turning counter-clockwise"

        def KEYCODE_Shift_Right(event):
            print "Turning clockwise"

        def KEYCODE_O(event):
            current = uc.get_gripper_pose(uc.arm_)
            print current
            fully_open = (round(current, 3) > 0.085)
            if fully_open:
                print "Gripper already fully open!"
            else:
                print "Opening " + uc.arm_ + "_gripper!"
                uc.command_gripper(uc.arm_, current + 0.01, 6, False, 0.4)


        def KEYCODE_P(event):
            current = uc.get_gripper_pose(uc.arm_)
            print current
            fully_closed = (round(current, 3) < 0.002)
            if fully_closed:
                print "Gripper already fully closed!"
            else:
                print "Closing " + uc.arm_ + "_gripper!"
                uc.command_gripper(uc.arm_, current - 0.01, 6, False, 0.4)


        
                        
        frame = Frame(root, width=100, height=100)
        frame.bind("<q>", KEYCODE_Q)
        frame.bind("<u>", KEYCODE_U)
        frame.bind("<j>", KEYCODE_J)
        frame.bind("<h>", KEYCODE_H)
        frame.bind("<z>", KEYCODE_1)
        frame.bind("<k>", KEYCODE_K)
        frame.bind("<y>", KEYCODE_Y)
        frame.bind("<i>", KEYCODE_I)
        frame.bind("<t>", KEYCODE_T)
        frame.bind("<o>", KEYCODE_O)
        frame.bind("<p>", KEYCODE_P)
        frame.bind("<w>", KEYCODE_W)
        frame.bind("<a>", KEYCODE_A)
        frame.bind("<s>", KEYCODE_S)
        frame.bind("<d>", KEYCODE_D)
        frame.bind("<Up>", KEYCODE_Up)
        frame.bind("<Down>", KEYCODE_Down)
        frame.bind("<Left>", KEYCODE_Left)
        frame.bind("<Right>", KEYCODE_Right)
        frame.bind("<Shift-Left>", KEYCODE_Shift_Left)
        frame.bind("<Shift-Right>", KEYCODE_Shift_Right)
        frame.bind("<space>", KEYCODE_SPACE)
        frame.pack()
        frame.focus_set()
        root.mainloop()


    rospy.init_node("ubertest")
    rospy.loginfo("how to use uber controller")
    uc = Uber()
    '''
    test_head() 
    test_torso()
    test_gripper()
<<<<<<< HEAD
    '''
    
=======
    test_joint()
    '''    

    pose_manager()
>>>>>>> 70b687425481dad3ddf1066061f76aa10e42012c


    #test_gripper_event()
