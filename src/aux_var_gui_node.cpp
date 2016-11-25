/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * ROS node that runs a Qt GUI that allows the user to enter values for
  * configuring control algorithms that are executed in Matlab in a high
  * performance computer.
  */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <QApplication>

#include "ros/ros.h"

#include "AUX_VAR_GUI/aux_var_gui.h"

//#include "/typewindow.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "aux_var_gui_node");
  QApplication qapp(argc, argv);
  AuxVarGUI aux_var_gui;
  aux_var_gui.show();
  return qapp.exec();
}
