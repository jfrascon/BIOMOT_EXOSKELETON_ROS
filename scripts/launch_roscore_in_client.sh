#!/bin/bash
# Author: Juan Francisco Rascón Crespo: jfrascon@gmail.com
# Set environment variables and launch roscore.
source ${CATKIN_WS}/devel/setup.bash
rm -r ${HOME}/.ros/log/*
roscore
