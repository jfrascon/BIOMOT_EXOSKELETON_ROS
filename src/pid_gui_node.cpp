/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * ROS node that launchs the real time joint pid plotting GUI.
  */

#include <iostream>
#include <locale>
#include <string>

#include <QApplication>

#include "h2r/PIDMessage.h"
#include "ros/ros.h"

#include "PID_GUI/pid_gui.h"

using namespace std;

int main(int argc, char *argv[]) {
  QApplication qapp(argc, argv);

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  QApplication::setGraphicsSystem("raster");
#endif

  PID pid;

  ros::init(argc, argv, "pid_gui_node");
  ros::NodeHandle n;
  ros::Subscriber pid_sub =
      n.subscribe("pid_topic", 1, &PID::process_pid, &pid);

  PIDGUI pid_gui(0);

  // QObject::connect(&pid, SIGNAL(pid_changed(struct PIDContainer)), &pid_gui,
  // SLOT(realtimeDataSlot(struct PIDContainer)));
  QObject::connect(&pid, SIGNAL(pid_changed(struct PIDContainer)), &pid_gui,
                   SLOT(update_pid_gui(struct PIDContainer)));

  qRegisterMetaType<PIDContainer>();

  pid_gui.show();

  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  qapp.exec();
  async_spinner.stop();

  return 0;
}
