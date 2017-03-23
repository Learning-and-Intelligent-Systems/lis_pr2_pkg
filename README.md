# lis_pr2_pkg


This is a python package that uses rospy with most of the default pr2_controllers. 


launch/ contains a launch file to start the teleop_joystick controller, pr2_gripper_sensor_action, and kinect.

src/ has uber_controller which is a simple python interface for interacting with the pr2 controllers.  Within this file Uber is created which has set defaults for using the pr2

scripts/ has an example file to test
