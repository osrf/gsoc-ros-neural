cmake_minimum_required(VERSION 2.8.3)
project(mindwave_execute_trajectory)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
roscpp
rospy
std_msgs
actionlib 
trajectory_msgs
message_generation)

find_package(Boost REQUIRED COMPONENTS system)

add_service_files(
   FILES
   ExecTraj.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES mindwave_execute_trajectory
  CATKIN_DEPENDS actionlib trajectory_msgs
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(mindwave_execute_trajectory_node main.cpp)
add_dependencies(mindwave_execute_trajectory_node mindwave_execute_trajectory_generate_messages_cpp)

target_link_libraries(mindwave_execute_trajectory_node
   ${catkin_LIBRARIES}
 )

add_executable(${PROJECT_NAME} main.cpp)
add_dependencies(${PROJECT_NAME} mindwave_execute_trajectory_generate_messages_cpp)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

