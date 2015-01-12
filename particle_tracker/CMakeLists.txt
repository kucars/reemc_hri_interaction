cmake_minimum_required(VERSION 2.8.3)
project(particle_tracker)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  nao_msgs
  pal_detection_msgs
  actionlib_msgs
  pcl_conversions
  pcl_ros
)

catkin_package(
 # INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs actionlib_msgs
)

## Declare a cpp executable
add_executable( particle_tracker src/main.cpp src/tracker.cpp src/particle.cpp src/Recognizer.cpp)
target_link_libraries(particle_tracker ${catkin_LIBRARIES})
