cmake_minimum_required(VERSION 2.8.3)
project(mindwave_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS geometry_msgs mindwave_msgs rospy
  DEPENDS
)


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/turtlebot_teleop_mindwave.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)
