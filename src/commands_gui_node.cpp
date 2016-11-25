/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * ROS node that launchs the command GUI.
  */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <QApplication>

#include "ros/ros.h"

#include "COMMANDS_GUI/commands_gui.h"

//#include "/typewindow.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "commands_gui_node");
  QApplication qapp(argc, argv);
  CommandsGUI commands_gui;
  // command_gui.setModal(true);
  commands_gui.show();
  return qapp.exec();
}
