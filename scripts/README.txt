Author: Juan Francisco Rascón Crespo: jfrascon@gmail.com

The following instructions must be set in the file $HOME/.bashrc of the each client computer and in the BeagleBone Black.
After each = sign a value must be set according to the next instructions.
PLEASE DON'T LEAVE A BLANK SPACE AFTER THE = SIGN.

# Wherever you put your catkin_ws folder, normally ==> ${HOME}/catkin_ws
export CATKIN_WS=
# Your ROS version. You can know it using the command 'rosversion -d' (without quotes) in a terminal
export ROS_VERSION=
export ROS_BIN_DIR=/opt/ros/${ROS_VERSION}/bin
# The computer's IP, i.e, the IP of a client computer or the IP of the BeagleBone Black
# For example:
# For a client computer ==> 192.168.7.1 (For USB only)
# For BeagleBone Black  ==> 192.168.7.2 (For USB only)
export ROS_IP=
# The master computer's IP. Normally the master computer is a PC, not the BeagleBone Black.
export ROS_MASTER_IP=
export ROS_MASTER_URI=http://${ROS_MASTER_IP}:11311
# Other configuration files
source /opt/ros/${ROS_VERSION}/setup.bash
source ${CATKIN_WS}/devel/setup.bash
