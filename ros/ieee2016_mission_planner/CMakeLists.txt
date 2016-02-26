cmake_minimum_required(VERSION 2.8.3)
project(ieee2016_mission_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ieee2016_mission_planner
  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

