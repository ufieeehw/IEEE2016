cmake_minimum_required(VERSION 2.8.3)
project(ieee2016_simulator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  message_runtime
)



generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ieee2016_simulator
 CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
