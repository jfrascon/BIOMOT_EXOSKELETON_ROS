#!/bin/bash
# Author: Juan Francisco Rascón Crespo: jfrascon@gmail.com
# Set environment variables and launch nodes in the BeagleBone Black
source ${CATKIN_WS}/devel/setup.bash
rm -r ${HOME}/.ros/log/*
reset
roslaunch h2r launch_ROS_nodes_in_bbb.launch
