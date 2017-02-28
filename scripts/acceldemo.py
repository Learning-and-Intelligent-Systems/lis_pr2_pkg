#!/usr/bin/env python
import rospy 
from pr2_msgs.msg import AccelerometerState as AS
import pdb

rospy.init_node("demo_accel")

# discussed message type
test_msg = AS()

class Accel():
    def __init__(self):
        self.sub= rospy.Subscriber("accelerometer/l_gripper_motor", AS, self.cb, queue_size=1)

    def cb (self, data):
        self.sub.unregister()
        #pdb.set_trace()
        points = []
        for sample in data.samples:
            pt = sample.x, sample.y, sample.z
            points.append(pt)

        print points
        raw_input("next")
        self.sub= rospy.Subscriber("accelerometer/l_gripper_motor", AS, self.cb, queue_size=1)
ac = Accel()
rospy.spin()
