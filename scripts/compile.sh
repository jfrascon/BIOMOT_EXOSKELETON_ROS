#!/bin/bash
# Author: Juan Francisco Rascón Crespo: jfrascon@gmail.com
# Compile source code in the BeagleBone Black and in the PC's.
if [ -d "${CATKIN_WS}/build" ]; then
    rm -r ${CATKIN_WS}/build
fi

if [ -d "${CATKIN_WS}/devel" ]; then
    rm -r ${CATKIN_WS}/devel
fi

${ROS_BIN_DIR}/catkin_make --directory ${CATKIN_WS}
source ${CATKIN_WS}/devel/setup.bash
