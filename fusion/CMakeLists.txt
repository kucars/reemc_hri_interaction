cmake_minimum_required(VERSION 2.8.3)
project(fusion)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pal_detection_msgs
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  pal_interaction_msgs
  actionlib_msgs
)
add_action_files(
  DIRECTORY action
  FILES test_msg.action
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  pal_detection_msgs# Or other packages containing msgs
)

catkin_package(
 CATKIN_DEPENDS actionlib_msgs pal_detection_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(debug src/debug.cpp)
#target_link_libraries(debug ${catkin_LIBRARIES})
#add_dependencies(debug pal_detection_msgs_generate_messages_cpp)


add_executable(fusion src/fusion.cpp)
target_link_libraries(fusion ${catkin_LIBRARIES})
add_dependencies(fusion pal_detection_msgs_generate_messages_cpp)

#add_executable(client src/client.cpp)
#target_link_libraries(client ${catkin_LIBRARIES})
#add_dependencies(client pal_detection_msgs_generate_messages_cpp)
