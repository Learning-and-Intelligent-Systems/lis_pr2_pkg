#!/usr/bin/env python
#Author: Zi Wang

from lis_pr2_pkg.uber_controller import Uber
import rospy
import os
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np


class Detector:
    def __init__(self):
        self.bridge = CvBridge()

        self.sub_image = rospy.Subscriber("/head_mount_kinect/rgb/image_rect_color", Image, self.processImage, queue_size=1)
        self.pub = rospy.Publisher("ifdetect", String, queue_size=1)
        rospy.loginfo("Detector initialized.")

    def processImage(self, image_msg):
        #self.sub_image.unregister()
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        face_cascade = cv2.CascadeClassifier('/home/demo/zi/haarcascade_frontalface_default.xml')
        eye_cascade = cv2.CascadeClassifier('/home/demo/zi/haarcascade_eye.xml')
        gray = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x,y,w,h) in faces:
            cv2.rectangle(image_cv,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = image_cv[y:y+h, x:x+w]
            eyes = eye_cascade.detectMultiScale(roi_gray)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
        if len(faces):
            rospy.loginfo("Face detected.")
            #cv2.imshow('image',image_cv)
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
            self.pub.publish("Detected %d faces." % len(faces))
        #cv2.imshow('image', image_cv)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        #cv2.imwrite('/home/demo/tmp.png',image_cv)


speak = lambda x: os.system("rosrun sound_play say.py '%s'" % x)

class Sub:
    def __init__(self):
        self.sub = rospy.Subscriber("ifdetect", String, self.cb)

    def cb(self, s):
        speak(s)
rospy.init_node("test_reactive")
rospy.loginfo('start testing reactive controller')

uc = Uber()
uc.lift_torso()
uc.move_arm_to_side('r')
uc.move_arm_to_side('l')
uc.open_gripper('r')
uc.command_head([-np.pi/3,np.pi/8],3,True)
#uc.look_forward()

det = Detector()
sub = Sub()
#while not rospy.is_shutdown():
#    uc.command_head([np.random.uniform(-np.pi/2,np.pi/2), 0], 3, True)
rospy.spin()
