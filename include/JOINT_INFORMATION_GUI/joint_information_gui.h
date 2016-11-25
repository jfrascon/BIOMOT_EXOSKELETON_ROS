#ifndef JOINT_MESSAGE_GUI_H
#define JOINT_MESSAGE_GUI_H

#include <QLCDNumber>
#include <QMainWindow>
#include <QMetaType>
#include <QTimer>

#include "h2r/ForcesMessage.h"
#include "h2r/JointAnglesMessage.h"
#include "h2r/SingleCommandMessage.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

struct JointInformationContainer
{
  float fix_link_angle[6];
  float alpha_angle[6];
  float lever_arm_angle[6];
  float alpha_torque[6];
  // float internal_load_cell_force;
  // float external_load_cell_force;
  // float internal_load_cell_torque;
  // float external_load_cell_torque;
  int type;
};

Q_DECLARE_METATYPE(JointInformationContainer);

class JointInformation : public QObject
{
  Q_OBJECT

 public:
  JointInformationContainer jic;

  JointInformation();
  JointInformation(const JointInformation &ji);
  const JointInformation &operator=(const JointInformation &ji);
  void process_joint_state(const sensor_msgs::JointState::ConstPtr &jsm);
  void process_joint_forces(const h2r::ForcesMessage::ConstPtr &fm);
  void process_joint_angles(const h2r::JointAnglesMessage::ConstPtr &jam);

 signals:
  void joint_information_changed(struct JointInformationContainer jsc);
};

namespace Ui
{
class JointInformationGUI;
}

class JointInformationGUI : public QMainWindow
{
  Q_OBJECT

 public:
  explicit JointInformationGUI(QWidget *parent = 0);
  ~JointInformationGUI();

 public slots:
  void update_joint_information_gui(struct JointInformationContainer);

 private slots:
  void showDatas();
  void on_lhip_rb_clicked();
  void on_lknee_rb_clicked();
  void on_lankle_rb_clicked();
  void on_rhip_rb_clicked();
  void on_rknee_rb_clicked();
  void on_rankle_rb_clicked();
  //  void on_deg_alpha_angle_button_clicked();
  //  void on_deg_fix_link_angle_button_clicked();
  //  void on_calibrate_encoders_button_clicked();

 private:
  int get_board_index(int board_id);
  //void send_reference_position(int message_id_);
  Ui::JointInformationGUI *ui;
  QTimer dataTimer;
  int board_id;
  int board_index;
  float fix_link_angle[6];
  float alpha_angle[6];
  float lever_arm_angle[6];
  float alpha_torque[6];
  ros::Publisher sender_pub;
  h2r::SingleCommandMessage single_command_msg;
  // float internal_load_cell_force[6];
  // float external_load_cell_force[6];
  // float internal_load_cell_torque[6];
  // float external_load_cell_torque[6];
};

#endif
