#!/usr/bin/env python
import rospy 
from pr2_msgs.msg import AccelerometerState as AS
import pdb
from lis_pr2_pkg.uber_controller import Uber

#XXX add description of this code

rospy.init_node("demo_accel")   #XXX use descriptive node name

# discussed message type
test_msg = AS()
uc = Uber()

class Accel():
    def __init__(self):
        self.sub= rospy.Subscriber("accelerometer/r_gripper_motor", AS, self.cb, queue_size=1) #XXX line is over 80 characters
	self.last_point = None  #XXX look at using numpy for cooler filtering 
	self.new_point = None
	self.delta = 2 #XXXa what does delta do?
	#uc.open_gripper('r')

    def cb (self, data):
	#pdb.set_trace()
	# for now just use the first sample as the acceleration
        self.last_point = self.new_point
	self.new_point = 0 # just tracking the x direction for now

	#print data.samples
	for i in range(len(data.samples)):
		self.new_point = (i*self.new_point + data.samples[i].x)/(i+1)
		#print self.new_point
 
	# if you have data in the new and old points
        if (self.last_point and self.new_point) != None:
		#print self.last_point, self.new_point
		
		# check for movement in the x direction
		if abs(self.last_point - self.new_point) > self.delta:
			print 'there is movement in the x direction!'
			# close the right gripper
			uc.close_gripper('r')
			raw_input('Press enter when you want the gripper to reopen')
			uc.open_gripper('r')

ac = Accel()
rospy.spin()
