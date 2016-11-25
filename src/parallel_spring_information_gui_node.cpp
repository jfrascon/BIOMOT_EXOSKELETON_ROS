/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * ROS node that launchs the GUI that displays the data associated with the
  * parallel spring of each knee.
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

#include "h2r/ParallelSpringMessage.h"
#include "ros/ros.h"

#include "PARALLEL_SPRING_INFORMATION_GUI/parallel_spring_information_gui.h"

using namespace std;

int main(int argc, char *argv[]) {
  //	cout << "main(tid): " << std::this_thread::get_id() << endl;

  QApplication qapp(argc, argv);

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  QApplication::setGraphicsSystem("raster");
#endif

  ParallelSpringInformation ps_inf;

  ros::init(argc, argv, "parallel_spring_information_gui_node");
  ros::NodeHandle n;
  ros::Subscriber ps_inf_sub =
      n.subscribe("parallel_spring_topic", 1,
                  &ParallelSpringInformation::process_parallel_spring, &ps_inf);

  ParallelSpringInformationGUI ps_inf_gui(0);

  QObject::connect(&ps_inf, SIGNAL(parallel_spring_information_changed(
                                struct ParallelSpringInformationContainer)),
                   &ps_inf_gui,
                   SLOT(update_parallel_spring_information_gui(
                       struct ParallelSpringInformationContainer)));

  qRegisterMetaType<ParallelSpringInformationContainer>();

  ps_inf_gui.show();

  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  qapp.exec();
  async_spinner.stop();

  return 0;
}
