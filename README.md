# ARIAC Robotics Challenge 2020
This repository is for creating a ROS package to complete the NIST ARIAC 2020 Challenge. The packages 
listed are part of the submission for the course ENPM663: Building a Manufacturing Robot, offered at the
University of Maryland, College Park in Spring 2021. 

## Package Information
1. rwa2_group_2 : Package that places the logical cameras in the environment to get information 
about different parts and their locations.
   
2. rwa3_group_2 : Package that completes the orders to build kits consisting of different parts.
The order may contain multiple shipments. The robot also replaces the parts that are faulty.
   
## Team Information
1. [Aman Virmani](https://github.com/AmanVirmani)
2. [Dani Lerner](https://github.com/dlerner97)
3. [Mack Tang](https://github.com/tangm7)
4. [Siddharth Telang](https://github.com/siddharthtelang)

## Dependencies
The packages have been tested on Ubuntu 18.04 with ROS 1 melodic installed. 

The following packages need to be installed.

1. moveit_core
2. moveit_ros_perception
3. moveit_ros_planning
4. moveit_ros_planning_interface
5. moveit_visual_tools
6. pcl_conversions
7. pcl_ros
8. tf2_eigen
9. tf2_geometry_msgs
10. tf2_ros

Also install the official ARIAC packages in your catkin workspace using below commands.

```bash
ca ~/catkin_ws/src/
git clone https://github.com/usnistgov/ARIAC.git
git clone https://github.com/osrf/ariac-gazebo_ros_pkgs
```
## Instructions to install packages
Clone this repository in the ```src``` directory of catkin workspace. The build the packages 
using ```catkin build```

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/AmanVirmani/ARIAC_Robotics
catkin build
```

