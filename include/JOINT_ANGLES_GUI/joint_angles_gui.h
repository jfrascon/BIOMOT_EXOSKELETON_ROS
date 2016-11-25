/*
* Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
* Qt GUI used to plot in 'realtime' the three angular positions of a selected
* joint.
* The three angular positions of a joint are:
*     - The fix link angle.
*     - The alpha angle.
*     - The lever arm angle (Which is the addition of the two others)
* This Qt GUI uses the QCustomPlot library to plot functions.
*/

#ifndef JOINT_ANGLES_GUI_H
#define JOINT_ANGLES_GUI_H

#include "qcustomplot.h"  // QCustomPlot header file.
#include <QMainWindow>
#include <QMetaType>
#include <QTimer>

#include <iostream>

#include "h2r/JointAnglesMessage.h"
#include "ros/ros.h"

using namespace std;

// Struct to hold all the latest angles for all the joint. These angles are
// transfered from a ROS message by a function that is called synchronusly by
// the ROS core.
struct JointAnglesContainer
{
  // fix link angle for:
  // left hip angle, left knee angle, left ankle angle, right hip angle, right
  // knee angle, right ankle angle.
  float fix_link_angle[6];
  // Same layout for alpha angles.
  float alpha_angle[6];
  // Same layout for lever arm angles.
  float lever_arm_angle[6];
};
// Qt macro to declare user defined types for transfering data between a signal
// and a slot. This macro doens't have to be used with primitive types: int,
// char, float ....
Q_DECLARE_METATYPE(JointAnglesContainer);

// Window to display all the angles of a joint.
// The joint must be selected in the GUI with a checkbox.
class JointAngles : public QObject
{
  Q_OBJECT

 public:
  JointAnglesContainer jac;

  JointAngles();
  JointAngles(const JointAngles &ja);
  const JointAngles &operator=(const JointAngles &ja);
  void process_joint_angles(const h2r::JointAnglesMessage::ConstPtr &jam);

  // When a ROS message is received synchronusly a function is automatically
  // called and its information is extracted. Then this signal is emitted and
  // afterwards.
 signals:
  void joint_angles_changed(struct JointAnglesContainer jac);
};

namespace Ui
{
class JointAnglesGUI;
}

class JointAnglesGUI : public QMainWindow
{
  Q_OBJECT

 public:
  explicit JointAnglesGUI(QWidget *parent = 0);
  ~JointAnglesGUI();

  void setupDemo(int demoIndex);
  void setupRealtimeDataDemo(QCustomPlot *qcp_left_joint,
                             QCustomPlot *qcp_right_joint);

 public slots:
  void update_joint_angles_gui(struct JointAnglesContainer jac);

 private slots:
  void realtimeDataSlot();
  void on_hip_rb_clicked();
  void on_knee_rb_clicked();
  void on_ankle_rb_clicked();

 private:
  void select_board_index();
  Ui::JointAnglesGUI *ui;
  QString demoName;
  QTimer dataTimer;
  // QCPItemTracer *itemDemoPhaseTracer;
  int currentDemoIndex;
  int board_id;
  int board_index;
  // int board_index_mirror;
  float fix_link_angles[6];
  float alpha_angles[6];
  float lever_arm_angles[6];
};

#endif  // JOINT_ANGLES_GUI_H
