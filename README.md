# gsoc-ros-neural
Repository for the GSOC 2015 Neural Interfaces for ROS project, sponsored by OSRF.

### Student Info ###
* Student Name: Steve Ataucuri Cruz
* Organization: Open Source Robotics Foundation
* Mentor's name: Jackie Kay, Esteve Fernandez
* Project title: Neural Interfaces for ROS/Gazebo project
* Project link: http://www.google-melange.com/gsoc/project/details/google/gsoc2015/stonescenter/5878405773918208
* Additional information: stonescenter.wordpress.com

### What is this repository for? ###
  
The main goal of this project is create a ros package, driver plugin library and virtual sensors to be used into Gazebo context which will be able to communicate with The Mindwave device to ROS framework 

#### Dependencies ####
In order to be able to run the nodes, it is necessary clone or install some packages:
* apt-get install libbluetooth-dev
* pip install pyserial
* pip install pybluez
* If you use python 2.7 install : pip install enum34

### Folders Contents? ###

* **mindwave_driver:** Driver for the Mindwave
* **mindwave_execute_trajectory:** ROS service to execute a trajectory of waypoints
* **mindwave_teleop:** ROS nodes to teleop some robots into Gazebo
* **mindwave_msgs:** ROS message for Mindwave driver
* **mindwave_gazebo:** Default world for Gazebo

### Documentation ###

* Step-by-step: please go to [Controling the turtlebot](https://github.com/osrf/gsoc-ros-neural/wiki/Steps-to-Control-a-robot-in-Gazebo)
* Step-by-step: please go to [Moving the robot arm](https://github.com/osrf/gsoc-ros-neural/wiki/Moving-an-Arm-manipulator)
* Reports : you can see additional info at [ 
Wiki](https://github.com/jacquelinekay/gsoc-ros-neural/wiki/GSoC-2015-Steve-Ataucuri)
