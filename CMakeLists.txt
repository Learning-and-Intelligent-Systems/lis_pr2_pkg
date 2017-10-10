cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(lis_pr2_pkg)

find_package(catkin REQUIRED COMPONENTS roscpp angles filters tf tf_conversions eigen_conversions pluginlib actionlib std_msgs geometry_msgs diagnostic_msgs trajectory_msgs pr2_controllers_msgs pr2_controller_interface pr2_controller_manager kdl_parser pr2_mechanism_model control_toolbox realtime_tools message_filters control_msgs  message_generation rospy geometry_msgs std_msgs actionlib_msgs genmsg )



catkin_python_setup()

#add_message_files(
#    DIRECTORY msg 
#    FILES 
#    )
#add_action_files(
#    DIRECTORY action
#    FILES
#    GDM.action
#    )

#add_service_files(
#    DIRECTORY srv
#    FILES
#    )

#catkin_install_python(PROGRAMS src/uber_controller.py
#      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#generate_messages(DEPENDENCIES geometry_msgs std_msgs actionlib_msgs )

catkin_package(
    CATKIN_DEPENDS  roscpp angles filters tf tf_conversions  eigen_conversions pluginlib actionlib std_msgs geometry_msgs diagnostic_msgs trajectory_msgs pr2_controllers_msgs pr2_controller_interface pr2_controller_manager kdl_parser pr2_mechanism_model control_toolbox realtime_tools message_filters control_msgs  message_runtime rospy geometry_msgs std_msgs actionlib_msgs genmsg
    LIBRARIES lis_pr2_pkg
)

