/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * ROS node that launchs the real time joint angle plotting GUI.
  */

#include <iomanip>
#include <iostream>

#include <locale>
#include <sstream>
#include <string>

#include <QApplication>

#include "JOINT_ANGLES_GUI/joint_angles_gui.h"

using namespace std;

int main(int argc, char *argv[]) {

  //	cout << "main(tid): " << std::this_thread::get_id() << endl;

  QApplication qapp(argc, argv);

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  QApplication::setGraphicsSystem("raster");
#endif

  JointAngles joint_angles;

  ros::init(argc, argv, "joint_angles_gui_node");
  ros::NodeHandle n;
  ros::Subscriber joint_angles_sub =
      n.subscribe("joint_angles_topic", 1, &JointAngles::process_joint_angles,
                  &joint_angles);

  JointAnglesGUI joint_angles_gui(0);

  QObject::connect(&joint_angles,
                   SIGNAL(joint_angles_changed(struct JointAnglesContainer)),
                   &joint_angles_gui,
                   SLOT(update_joint_angles_gui(struct JointAnglesContainer)));

  qRegisterMetaType<JointAnglesContainer>();

  joint_angles_gui.show();

  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  qapp.exec();
  async_spinner.stop();

  return 0;
}
