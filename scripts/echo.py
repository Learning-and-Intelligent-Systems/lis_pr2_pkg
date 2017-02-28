#!/usr/bin/env python
"""
This program recieves an image from a rostopic and converts it
to a cv2 image.  Then, it immediately converts it back to a
ros message of type sensor_msgs/Image and publishes it on the
rostopic ~echo_image. 
"""
import cv2  # the open cv library packge
import rospy # standard ros with python package
from sensor_msgs.msg import Image  # the rostopic message we subscribe/publish 
from cv_bridge import CvBridge # package to convert rosmsg<->cv2 

class Echo:
    def __init__(self):
        
        # create bridge to convert to and from cv2 and rosmsg
        self.bridge = CvBridge()
         
        # publisher for the image we will "echo"
        self.pub_image = rospy.Publisher("~echo_image",\
                Image, queue_size=1)
        
        # subscribe to the rostopic carrying the image we are interested in
        # "camera/rgb/image_rect_color" is the topic name
        # Image is the message type
        # self.processImage is the callback function executed every time we
        # recieve the message
        self.sub_image = rospy.Subscriber("/head_mount_kinect/rgb/image_rect_color",\
                Image, self.processImage, queue_size=1)
        
        # report initalization success
        rospy.loginfo("Echo Initialized.")

    """
    This is the callback function that is executed every time an Image
    message is received on the rostopic we subscribed to.
    """
    def processImage(self, image_msg):
        # convert rosmsg to cv2 type
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)            

        # convert cv2 message back to rosmsg
        image_ros_msg = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")

        # publish rosmsg 
        self.pub_image.publish(image_ros_msg) 


if __name__=="__main__":
    # initalize the ros node
    rospy.init_node('Echo')

    # create Echo to start the image passthrough
    e = Echo()

    # continue running echo until node is killed from outside process
    rospy.spin()

