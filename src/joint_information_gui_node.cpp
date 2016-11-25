/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * ROS node that launchs the real time joint angle information GUI.
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

#include "h2r/ForcesMessage.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

#include "JOINT_INFORMATION_GUI/joint_information_gui.h"
#include "logObject.h"

using namespace std;

int main(int argc, char *argv[]) {
  //	cout << "main(tid): " << std::this_thread::get_id() << endl;

  QApplication qapp(argc, argv);

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  QApplication::setGraphicsSystem("raster");
#endif

  JointInformation joint_information;

  ros::init(argc, argv, "joint_information_gui_node");
  ros::NodeHandle n;
  ros::Subscriber joint_state_sub =
      n.subscribe("joint_state", 1, &JointInformation::process_joint_state,
                  &joint_information);
  ros::Subscriber joint_forces_sub =
      n.subscribe("forces_topic", 1, &JointInformation::process_joint_forces,
                  &joint_information);
  ros::Subscriber joint_angles_sub =
      n.subscribe("joint_angles_topic", 1,
                  &JointInformation::process_joint_angles, &joint_information);

  JointInformationGUI joint_information_gui(0);

  QObject::connect(
      &joint_information,
      SIGNAL(joint_information_changed(struct JointInformationContainer)),
      &joint_information_gui,
      SLOT(update_joint_information_gui(struct JointInformationContainer)));

  qRegisterMetaType<JointInformationContainer>();

  joint_information_gui.show();

  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  qapp.exec();
  async_spinner.stop();

  return 0;
}
