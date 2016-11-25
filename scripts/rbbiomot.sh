#!/bin/bash
# Author: Juan Francisco Rascón Crespo: jfrascon@gmail.com
# Log specific messages.
source ${CATKIN_WS}/devel/setup.bash
rosbag record /joint_angles_topic /joint_state /forces_topic /fsrs_topic /patterns_topic /pid_topic /sensors_topic -o DataLog_
