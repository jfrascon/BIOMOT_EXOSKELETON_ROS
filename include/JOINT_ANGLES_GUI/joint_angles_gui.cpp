/*
* Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
* Qt GUI used to plot in 'realtime' the three angular positions of a selected
* joint.
* The three angular positions of a joint are:
*     The fix link angle.
*     The alpha angle.
*     The lever arm angle (Which is the addition of the two others)
* This Qt GUI uses the QCustomPlot library to plot functions.
*/

#include <string>

#include <QDesktopWidget>
#include <QMessageBox>
#include <QScreen>
#include <thread>

#include "constants.h"
#include "joint_angles_gui.h"
#include "ui_joint_angles_gui.h"

using namespace std;

/** 6 joints: 2 hips, 2 knees, 2 ankles.
  * Layout: left hip angle, left knee angle, left ankle angle, right hip angle,
  * right knee angle, right ankle angle.
  * 3 angles per joint => 18 angles for storing.
  * First array: alpha angles.
  * Second array: fix link angles.
  * Third array: lever arm angles.
  * The struct joint angles container stores all these angles in three different
  * float arrays.
  */
JointAngles::JointAngles()
    : jac{{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
          {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
          {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}
{
}

JointAngles::JointAngles(const JointAngles &js) : jac(js.jac) {}

const JointAngles &JointAngles::operator=(const JointAngles &js)
{
  jac = js.jac;
}

/** The exoskeleton joint angles are transmitted through the ROS network in
 * messages of type JointAnglesMessage.
 * Each message of this type can hold 18 angles (3 angles per joint).
 * These angles are transmitted in radians but are displayed in degrees, so an
 * unity conversion in needed.
 */
void JointAngles::process_joint_angles(
    const h2r::JointAnglesMessage::ConstPtr &jam)
{
  // Process all the angles of each joint and convert them from radians to
  // degrees.
  for (int i = 5; i >= 0; i--)
  {
    jac.fix_link_angle[i] = rad2grad * jam->fix_link_angle[i];
    jac.alpha_angle[i] = rad2grad * jam->alpha_angle[i];
    jac.lever_arm_angle[i] = rad2grad * jam->lever_arm_angle[i];
  }

  emit joint_angles_changed(jac);  // Emit notification to the Qt core
  // cout << "process_joint_state(tid): " << std::this_thread::get_id() << " |
  // bid: " << jac.board_id << " | fla: " << setfill(' ') << setw(8) <<
  // jac.fix_link_angle << " deg | aa: " << setfill(' ') << setw(8) <<
  // jac.alpha_angle << " deg" << endl;
}

JointAnglesGUI::JointAnglesGUI(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::JointAnglesGUI),
      board_id(LEFT_HIP_ID),
      fix_link_angles{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      alpha_angles{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      lever_arm_angles{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
{
  //  cout << "JointAnglesGUI(tid) " << QThread::currentThreadId() << endl;
  select_board_index();
  ui->setupUi(this);
  ui->hip_rb->setChecked(true);
  setGeometry(400, 250, 542, 390);
  setup(10);
}

JointAnglesGUI::~JointAnglesGUI() { delete ui; }

void JointAnglesGUI::setup(int demoIndex)
{
  //  cout << "setup(tid): " << QThread::currentThreadId() << endl;
  switch (demoIndex)
  {
    case 10:
      setup_real_time_data_demo(ui->qcp_left_joint, ui->qcp_right_joint);
      break;
  }
  setWindowTitle(demoName);
  statusBar()->clearMessage();
  currentDemoIndex = demoIndex;
  ui->qcp_left_joint->replot();
  ui->qcp_right_joint->replot();
}
/** Plot the last point available for the selected joint.
*/
void JointAnglesGUI::setup_real_time_data_demo(QCustomPlot *qcp_left_joint,
                                               QCustomPlot *qcp_right_joint)
{
//  cout << "setup_real_time_data_demo(tid): " << QThread::currentThreadId() <<
//  endl;

//  Include this section to fully disable antialiasing for higher performance:
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  QMessageBox::critical(this, "",
                        "You're using Qt < 4.7, this program needs "
                        "functions that are available with Qt 4.7 to "
                        "work properly");
#endif
  demoName = "Realtime Joint Angles GUI";

  //  qcp_left_joint->setNotAntialiasedElements(QCP::aeAll);
  //  QFont font;
  //  font.setStyleStrategy(QFont::NoAntialias);
  //  qcp_left_joint->xAxis->setTickLabelFont(font);
  //  qcp_left_joint->yAxis->setTickLabelFont(font);
  //  qcp_left_joint->legend->setFont(font);

  // For a given joint: hip, knee, angle -> 3 angles for the left side joint
  // and 3 angles for the right side joint are displayed.

  qcp_left_joint->addGraph();  // Red line: Left alpha angle.
  qcp_left_joint->graph(0)->setPen(QPen(Qt::red));
  qcp_left_joint->graph(0)->setAntialiasedFill(false);

  qcp_left_joint->addGraph();  // Green line: Left fix link angle.
  qcp_left_joint->graph(1)->setPen(QPen(Qt::green));
  qcp_left_joint->graph(1)->setAntialiasedFill(false);

  qcp_left_joint->addGraph();  // Blue line: Left lever arm angle.
  qcp_left_joint->graph(2)->setPen(QPen(Qt::blue));
  qcp_left_joint->graph(2)->setAntialiasedFill(false);

  qcp_left_joint->addGraph();  // Red dot
  qcp_left_joint->graph(3)->setPen(QPen(Qt::red));
  qcp_left_joint->graph(3)->setLineStyle(QCPGraph::lsNone);
  qcp_left_joint->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);

  qcp_left_joint->addGraph();  // Green dot
  qcp_left_joint->graph(4)->setPen(QPen(Qt::green));
  qcp_left_joint->graph(4)->setLineStyle(QCPGraph::lsNone);
  qcp_left_joint->graph(4)->setScatterStyle(QCPScatterStyle::ssDisc);

  qcp_left_joint->addGraph();  // Blue dot
  qcp_left_joint->graph(5)->setPen(QPen(Qt::blue));
  qcp_left_joint->graph(5)->setLineStyle(QCPGraph::lsNone);
  qcp_left_joint->graph(5)->setScatterStyle(QCPScatterStyle::ssDisc);

  // In the x axis a timestamp is displayed.
  qcp_left_joint->xAxis->setTickLabelType(QCPAxis::ltDateTime);
  qcp_left_joint->xAxis->setDateTimeFormat("hh:mm:ss");
  qcp_left_joint->xAxis->setAutoTickStep(false);
  qcp_left_joint->xAxis->setTickStep(2);
  qcp_left_joint->axisRect()->setupFullAxesBox();

  //  Make left and bottom axes transfer their ranges to right and top axes.
  connect(qcp_left_joint->xAxis, SIGNAL(rangeChanged(QCPRange)),
          qcp_left_joint->xAxis2, SLOT(setRange(QCPRange)));
  connect(qcp_left_joint->yAxis, SIGNAL(rangeChanged(QCPRange)),
          qcp_left_joint->yAxis2, SLOT(setRange(QCPRange)));

  qcp_right_joint->addGraph();  // Gray line: Right alpha angle.
  qcp_right_joint->graph(0)->setPen(QPen(Qt::darkGray));
  qcp_right_joint->graph(0)->setAntialiasedFill(false);

  qcp_right_joint->addGraph();  // Black line: Right fix link angle.
  qcp_right_joint->graph(1)->setPen(QPen(Qt::black));
  qcp_right_joint->graph(1)->setAntialiasedFill(false);

  qcp_right_joint->addGraph();  // Cyan line: Right lever arm angle.
  qcp_right_joint->graph(2)->setPen(QPen(Qt::cyan));
  qcp_right_joint->graph(2)->setAntialiasedFill(false);

  qcp_right_joint->addGraph();  // Gray dot
  qcp_right_joint->graph(3)->setPen(QPen(Qt::darkGray));
  qcp_right_joint->graph(3)->setLineStyle(QCPGraph::lsNone);
  qcp_right_joint->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);

  qcp_right_joint->addGraph();  // Black dot
  qcp_right_joint->graph(4)->setPen(QPen(Qt::black));
  qcp_right_joint->graph(4)->setLineStyle(QCPGraph::lsNone);
  qcp_right_joint->graph(4)->setScatterStyle(QCPScatterStyle::ssDisc);

  qcp_right_joint->addGraph();  // Cyan dot
  qcp_right_joint->graph(5)->setPen(QPen(Qt::cyan));
  qcp_right_joint->graph(5)->setLineStyle(QCPGraph::lsNone);
  qcp_right_joint->graph(5)->setScatterStyle(QCPScatterStyle::ssDisc);

  qcp_right_joint->xAxis->setTickLabelType(QCPAxis::ltDateTime);
  qcp_right_joint->xAxis->setDateTimeFormat("hh:mm:ss");
  qcp_right_joint->xAxis->setAutoTickStep(false);
  qcp_right_joint->xAxis->setTickStep(2);
  qcp_right_joint->axisRect()->setupFullAxesBox();

  //  Make left and bottom axes transfer their ranges to right and top axes.
  connect(qcp_right_joint->xAxis, SIGNAL(rangeChanged(QCPRange)),
          qcp_right_joint->xAxis2, SLOT(setRange(QCPRange)));
  connect(qcp_right_joint->yAxis, SIGNAL(rangeChanged(QCPRange)),
          qcp_right_joint->yAxis2, SLOT(setRange(QCPRange)));

  // Setup a timer that repeatedly calls JointAnglesGUI::real_time_data_slot to
  // plot the data.
  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
  dataTimer.start(0);  // Interval 0 means to refresh as fast as possible
}

// Get the left joint to plot its data. The associate right joint id is obtained
// adding +3 to the actual left board id.
// 32 left hip  -->  board_index = 0
// 64 left knee -->  board_index = 1
// 96 left ankle --> board_index = 2
// 128 right hip  -->  board_index = 0 + 3
// 160 right knee -->  board_index = 1 + 3
// 192 right ankle --> board_index = 2 + 3
void JointAnglesGUI::select_board_index()
{
  board_index = (board_id - LEFT_HIP_ID) / LEFT_HIP_ID;
}

void JointAnglesGUI::on_hip_rb_clicked()
{
  board_id = LEFT_HIP_ID;
  select_board_index();
}
void JointAnglesGUI::on_knee_rb_clicked()
{
  board_id = LEFT_KNEE_ID;
  select_board_index();
}
void JointAnglesGUI::on_ankle_rb_clicked()
{
  board_id = LEFT_ANKLE_ID;
  select_board_index();
}

/** Plot all joint angles (joint alpha angle, joint fix link angle, joint lever
  * arm angle) when the timer is over.
  */
void JointAnglesGUI::realtimeDataSlot()
{

// Get the actual time and compare with the last registered time.
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  double key = 0;
#else
  double key = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0;
#endif

  static double lastPointKey = 0.0;

  if (key - lastPointKey > 0.02)  // at most add point every 20 ms
  {
    // cout << "realtimeDataSlot(tid): " << QThread::currentThreadId()
    // << " " << board_id << endl;

    // Add data to lines:
    ui->qcp_left_joint->graph(0)->addData(key, alpha_angles[board_index]);
    ui->qcp_left_joint->graph(1)->addData(key, fix_link_angles[board_index]);
    ui->qcp_left_joint->graph(2)->addData(key, lever_arm_angles[board_index]);

    // Remove data of lines that are outside of the visible range.
    ui->qcp_left_joint->graph(0)->removeDataBefore(key - 4);
    ui->qcp_left_joint->graph(1)->removeDataBefore(key - 4);
    ui->qcp_left_joint->graph(2)->removeDataBefore(key - 4);

    // Rescale vertical axis to fit the current data.
    ui->qcp_left_joint->graph(0)->rescaleValueAxis(true);
    ui->qcp_left_joint->graph(1)->rescaleValueAxis(true);
    ui->qcp_left_joint->graph(2)->rescaleValueAxis(true);

    // Set dots.
    ui->qcp_left_joint->graph(3)->clearData();
    ui->qcp_left_joint->graph(3)->addData(key, alpha_angles[board_index]);

    ui->qcp_left_joint->graph(4)->clearData();
    ui->qcp_left_joint->graph(4)->addData(key, fix_link_angles[board_index]);

    ui->qcp_left_joint->graph(5)->clearData();
    ui->qcp_left_joint->graph(5)->addData(key, lever_arm_angles[board_index]);

    // Rescale vertical axis to fit the current data.
    ui->qcp_left_joint->graph(3)->rescaleValueAxis(true);
    ui->qcp_left_joint->graph(4)->rescaleValueAxis(true);
    ui->qcp_left_joint->graph(5)->rescaleValueAxis(true);

    // Add data to lines.
    ui->qcp_right_joint->graph(0)->addData(key, alpha_angles[board_index + 3]);
    ui->qcp_right_joint->graph(1)->addData(key,
                                           fix_link_angles[board_index + 3]);
    ui->qcp_right_joint->graph(2)->addData(key,
                                           lever_arm_angles[board_index + 3]);

    // Remove data of lines that are outside of the visible range.
    ui->qcp_right_joint->graph(0)->removeDataBefore(key - 4);
    ui->qcp_right_joint->graph(1)->removeDataBefore(key - 4);
    ui->qcp_right_joint->graph(2)->removeDataBefore(key - 4);

    // Rescale vertical axis to fit the current data.
    ui->qcp_right_joint->graph(0)->rescaleValueAxis(true);
    ui->qcp_right_joint->graph(1)->rescaleValueAxis(true);
    ui->qcp_right_joint->graph(2)->rescaleValueAxis(true);

    // Set dots.
    ui->qcp_right_joint->graph(3)->clearData();
    ui->qcp_right_joint->graph(3)->addData(key, alpha_angles[board_index + 3]);

    ui->qcp_right_joint->graph(4)->clearData();
    ui->qcp_right_joint->graph(4)->addData(key,
                                           fix_link_angles[board_index + 3]);

    ui->qcp_right_joint->graph(5)->clearData();
    ui->qcp_right_joint->graph(5)->addData(key,
                                           lever_arm_angles[board_index + 3]);

    // Rescale vertical axis to fit the current data.
    ui->qcp_right_joint->graph(3)->rescaleValueAxis(true);
    ui->qcp_right_joint->graph(4)->rescaleValueAxis(true);
    ui->qcp_right_joint->graph(5)->rescaleValueAxis(true);

    // Back-up time.
    lastPointKey = key;
  }

  // Make key axis range scroll with the data (at a constant range size of 8):
  ui->qcp_left_joint->xAxis->setRange(key + 0.50, 4, Qt::AlignRight);
  ui->qcp_left_joint->replot();

  ui->qcp_right_joint->xAxis->setRange(key + 0.50, 4, Qt::AlignRight);
  ui->qcp_right_joint->replot();

  // Calculate frames per second and display the number in the statusbar
  static double lastFpsKey;
  static int frameCount;

  ++frameCount;

  if (key - lastFpsKey > 2)  // average fps over 2 seconds
  {
    ui->statusBar->showMessage(
        QString("%1 FPS, Total Data points: %2")
            .arg(frameCount / (key - lastFpsKey), 0, 'f', 0)
            .arg(ui->qcp_left_joint->graph(0)->data()->count() +
                 ui->qcp_left_joint->graph(1)->data()->count() +
                 ui->qcp_left_joint->graph(2)->data()->count() +
                 ui->qcp_left_joint->graph(3)->data()->count() +
                 ui->qcp_left_joint->graph(4)->data()->count() +
                 ui->qcp_left_joint->graph(5)->data()->count() +
                 ui->qcp_right_joint->graph(0)->data()->count() +
                 ui->qcp_right_joint->graph(1)->data()->count() +
                 ui->qcp_right_joint->graph(2)->data()->count() +
                 ui->qcp_right_joint->graph(3)->data()->count() +
                 ui->qcp_right_joint->graph(4)->data()->count() +
                 ui->qcp_right_joint->graph(5)->data()->count()),
        0);
    lastFpsKey = key;
    frameCount = 0;
  }
}

// Slot for receiving data (joint angles) collected from the ROS network.
// All joint angles are obtained from a ROS message within a function called
// synchronusly by the ROS core. In this function the angles are copied from the
// ROS Message to a struct of type JointAnglesContainer. Then a Qt signal is
// emitted to the Qt core. When the Qt core can handle this notification it
// executes this slot (Qt callback function) that transfers the angles from the
// JointAnglesContainer struct to the adecuate arrays belonging to the window.
// Then the window will be able to plot the angles if joint is selected with a
// checkbox.
void JointAnglesGUI::update_joint_angles_gui(struct JointAnglesContainer jac)
{
  for (int i = 5; i >= 0; i--)
  {
    fix_link_angles[i] = jac.fix_link_angle[i];
    alpha_angles[i] = jac.alpha_angle[i];
    lever_arm_angles[i] = jac.lever_arm_angle[i];
  }
  //	cout << "update_joint_angles_gui(tid): " << QThread::currentThreadId()
  //<< " board_id: " << jac.board_id << " | fla: " << setfill(' ') << setw(8) <<
  // fix_link_angles[jac.board_id] << " degrees | aa: " << setfill(' ') <<
  // setw(8) << alpha_angles[board_index] << " degrees" << endl;
}
