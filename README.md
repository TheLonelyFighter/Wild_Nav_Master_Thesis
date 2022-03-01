# Wild_Nav_Master_Thesis
Open source autonomous drone path planning package written in python using [ROS2](https://docs.ros.org/en/foxy/index.html) and [PX4](https://px4.io/) Autopilot.

# Setting up the environment

## 1. ROS2 Foxy Distro

Check out the guide [here](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html) for binary install.

## 2.  PX4 Autopilot (hang in there, this one is quite tough :))
Install dependencies using the guide [here](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html)    
Build PX4 using guide [here](https://docs.px4.io/master/en/dev_setup/building_px4.html)

## 3. ROS2-PX4 Bridge
Install only Fast-RTPS-Gen using the guide [here](https://docs.px4.io/master/en/dev_setup/fast-dds-installation.html) because Fast DDS comes pre-installed with ROS2  Foxy.  
Then follow the guide [here](https://docs.px4.io/master/en/ros/ros2_comm.html).

# Test autonomous fligh package in Gazebo
a. Source ROS2 install:
  source ~/ros2_foxy/ros2-linux/setup.bash
b. Source ROS2-PX4 comm package:
  source ~/px4_ros_com_ros2/install/setup.bash

c. Launch simulation:
  make px4_sitl_rtps gazebo
d. Launch RTPS agent used for communication between ROS2-PX4
  micrortps_agent -t UDP
e. Finally run python package

