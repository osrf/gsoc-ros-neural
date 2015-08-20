cmake_minimum_required(VERSION 2.8.3)
project(mindwave_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	message_generation 
	geometry_msgs 
	roscpp 
	std_msgs
	actionlib
	joint_trajectory_action
	kinematics_msgs
	pr2_controllers_msgs
  control_msgs
	tf
  roslib
  robotiq_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS 
  	geometry_msgs 
  	mindwave_msgs 
  	rospy 
  	roscpp 
  	actionlib 
  	joint_trajectory_action 
  	kinematics_msgs 
  	pr2_controllers_msgs
    control_msgs
  	tf
    roslib
    robotiq_msgs
  DEPENDS
)

add_executable(quadcopter_teleop_mindwave src/quadcopter_teleop_mindwave.cpp)
target_link_libraries(quadcopter_teleop_mindwave ${catkin_LIBRARIES})

add_executable(ur10_ik_move_arm src/ur10_ik_move_arm.cpp)
target_link_libraries(ur10_ik_move_arm ${catkin_LIBRARIES})

add_executable(ur10_simple_move_arm src/ur10_simple_move_arm.cpp)
target_link_libraries(ur10_simple_move_arm ${catkin_LIBRARIES})

add_executable(reproduce_trajectory_node src/reproduce_trajectory.cpp)
target_link_libraries(reproduce_trajectory_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/turtlebot_teleop_mindwave.py
 	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/client_teleop_arm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS quadcopter_teleop_mindwave ur10_ik_move_arm ur10_simple_move_arm reproduce_trajectory_node
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)
