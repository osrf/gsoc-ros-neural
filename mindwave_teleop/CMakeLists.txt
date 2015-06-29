cmake_minimum_required(VERSION 2.8.3)
project(mindwave_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	message_generation geometry_msgs roscpp
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS geometry_msgs mindwave_msgs rospy roscpp
  DEPENDS
)

add_executable(quadcopter_teleop_mindwave src/quadcopter_teleop_mindwave.cpp)
target_link_libraries(quadcopter_teleop_mindwave ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/turtlebot_teleop_mindwave.py
 	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS quadcopter_teleop_mindwave
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)
