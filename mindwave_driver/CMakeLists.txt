cmake_minimum_required(VERSION 2.8.3)
project(mindwave_driver)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
    rospy 
)

install(
    PROGRAMS
       nodes/mindwave_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)


