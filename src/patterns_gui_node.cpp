/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * ROS node that launchs a GUI that allows the user to select a patterns file
  * to be send to the exoskeleton joint boards.
  */

#include <QApplication>

#include "ros/ros.h"

//#include "h2r/PatternsMessage.h"
#include "PATTERNS_GUI/patterns_gui.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "patterns_gui_node");
  QApplication qapp(argc, argv);
  qRegisterMetaType<int>();
  PatternsGUI patterns_gui;
  patterns_gui.show();
  return qapp.exec();
}
