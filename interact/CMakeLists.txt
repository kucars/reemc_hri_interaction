cmake_minimum_required(VERSION 2.8.3)
project(interact)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  actionlib_msgs
)

#generate_messages(
#  DEPENDENCIES actionlib_msgs std_msgs  pal_detection_msgs test_msgAction
#)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
