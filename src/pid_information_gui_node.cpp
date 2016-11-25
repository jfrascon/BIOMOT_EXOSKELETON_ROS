/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * ROS node that launchs the real time joint pid information GUI.
  */

#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <QApplication>

#include "h2r/PIDMessage.h"
#include "ros/ros.h"

#include "PID_INFORMATION_GUI/pid_information_gui.h"

using namespace std;

int main(int argc, char *argv[]) {
  //	cout << "main(tid): " << std::this_thread::get_id() << endl;

  QApplication qapp(argc, argv);

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  QApplication::setGraphicsSystem("raster");
#endif

  PIDInformation pid_information;

  ros::init(argc, argv, "pid_information_gui_node");
  ros::NodeHandle n;
  ros::Subscriber pid_information_sub = n.subscribe(
      "pid_topic", 1, &PIDInformation::process_pid, &pid_information);

  PIDInformationGUI pid_information_gui(0);
  QObject::connect(
      &pid_information,
      SIGNAL(pid_information_changed(struct PIDInformationContainer)),
      &pid_information_gui,
      SLOT(update_pid_information_gui(struct PIDInformationContainer)));

  qRegisterMetaType<PIDInformationContainer>();

  pid_information_gui.show();

  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  qapp.exec();
  async_spinner.stop();

  return 0;
}
