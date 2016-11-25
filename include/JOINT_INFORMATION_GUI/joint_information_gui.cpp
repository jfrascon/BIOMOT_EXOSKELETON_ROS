/*
* Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
* Qt GUI used to display in 'realtime' the numeric value of:
*     The fix link angle in a specific joint, in degrees and in percent over the
*     maximum fix link angle permited in that joint.
*     The alpha angle in the same joint, in degrees and in percent over the
*     maximum alpha angle permitted in that joint.
*     The lever arm angle (which is the addition of the two others) in the same
*     joint, in degrees and in percent over the maximum lever arm angle
*     permitted in that joint.
*     The torque in the same joint, in Nm and in percent over the maximum torque
*     permitted in that joint.
*     It also display the board id from which the information is gathered.
*/

#include <string>

#include <QString>
#include <QTimer>
#include <qmessagebox.h>

#include "constants.h"
#include "joint_information_gui.h"
#include "ros/ros.h"
#include "ui_joint_information_gui.h"

using namespace std;

// Initialize the struct that store the last angle and torque for each joint.
JointInformation::JointInformation()
    : jic{{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
          {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
          {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
          {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}
{
}
JointInformation::JointInformation(const JointInformation &ji) : jic(ji.jic) {}
const JointInformation &JointInformation::operator=(const JointInformation &ji)
{
  jic = ji.jic;
}

/** In a JointState ROS message there is information of a fix link angle and
 * torque. No alpha angle information is available.
  */
void JointInformation::process_joint_state(
    const sensor_msgs::JointState::ConstPtr &jsm)
{
  for (int i = 5; i >= 0; i--)
  {
    jic.fix_link_angle[i] = rad2grad * jsm->position[i];
    jic.alpha_torque[i] = jsm->effort[i];
  }
  // Indicate that the data in the joint information container has been gathered
  // from a joint
  // state message, so the alpha angle can't be updated in the GUI.
  jic.type = 0;
  // cout << "PJS  " << jic.board_id << "/0  |  fla = " <<  setfill(' ') <<
  // setw(7) << jic.fix_link_angle << endl;
  // Emit a notification to the Qt core that the message's data has already been
  // processed.
  emit joint_information_changed(jic);
}

/** The JointAnglesMessage ROS message only has data about angles in a specific
 * joint.
 */
void JointInformation::process_joint_angles(
    const h2r::JointAnglesMessage::ConstPtr &jam)
{
  for (int i = 5; i >= 0; i--)
  {
    jic.fix_link_angle[i] = rad2grad * jam->fix_link_angle[i];
    jic.alpha_angle[i] = rad2grad * jam->alpha_angle[i];
    jic.lever_arm_angle[i] = rad2grad * jam->lever_arm_angle[i];
  }
  // Indicate that the data in the joint information container has been gathered
  // from a joint angles message, so the alpha torque can't be updated in the
  // GUI.
  jic.type = 1;
  // cout << "PJA  " << jic.board_id << "/2  |  fla = " <<  setfill(' ') <<
  // setw(7) << jic.fix_link_angle << "  |  aa = " << setfill(' ') << setw(7) <<
  // jic.alpha_angle << endl;
  // Emit a notification to the Qt core that the message's data has already been
  // processed.
  emit joint_information_changed(jic);
}

JointInformationGUI::JointInformationGUI(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::JointInformationGUI),
      board_id(LEFT_HIP_ID),
      board_index((board_id - LEFT_HIP_ID) / LEFT_HIP_ID),
      fix_link_angle{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      alpha_angle{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      alpha_torque{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      lever_arm_angle{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
{

  ros::NodeHandle n;
  sender_pub = n.advertise<h2r::SingleCommandMessage>("tx_topic", 1);

  single_command_msg.number_elements = 1;
  single_command_msg.message_id.assign(6, 0);
  single_command_msg.type = TOR_PID_CONTROLLER_ID;
  single_command_msg.data.assign(6, 0.0f);

  ui->setupUi(this);
  ui->lhip_rb->setChecked(true);
  // ui->label_9->setVisible(false);
  // ui->label_10->setVisible(false);
  // QTimer *timer = new QTimer(this);

  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(showDatas()));
  dataTimer.start(100); /*Runs a timer every 100 milliseconds*/
}

int JointInformationGUI::get_board_index(int board_id)
{
  return ((board_id - LEFT_HIP_ID) / LEFT_HIP_ID);
}

void JointInformationGUI::on_lhip_rb_clicked()
{
  board_id = LEFT_HIP_ID;
  board_index = get_board_index(board_id);
}

void JointInformationGUI::on_lknee_rb_clicked()
{
  // select_board(LEFT_KNEE_ID);
  board_id = LEFT_KNEE_ID;
  board_index = get_board_index(board_id);
}

void JointInformationGUI::on_lankle_rb_clicked()
{
  // select_board(LEFT_ANKLE_ID);
  board_id = LEFT_ANKLE_ID;
  board_index = get_board_index(board_id);
}

void JointInformationGUI::on_rhip_rb_clicked()
{
  board_id = RIGHT_HIP_ID;
  board_index = get_board_index(board_id);
}

void JointInformationGUI::on_rknee_rb_clicked()
{
  board_id = RIGHT_KNEE_ID;
  board_index = get_board_index(board_id);
}

void JointInformationGUI::on_rankle_rb_clicked()
{
  board_id = RIGHT_ANKLE_ID;
  board_index = get_board_index(board_id);
}

void JointInformationGUI::showDatas() /*Updates data's output*/
{
  QString text =
      QString::number(angle_deg2percent * fix_link_angle[board_index], 'f', 2);
  ui->lcd_fix_link_angle_percent->display(text);

  text = QString::number(fix_link_angle[board_index], 'f', 2);
  ui->lcd_fix_link_angle_degrees->display(text);

  text = QString::number(angle_deg2percent * alpha_angle[board_index], 'f', 2);
  ui->lcd_alpha_angle_percent->display(text);

  text = QString::number(alpha_angle[board_index], 'f', 2);
  ui->lcd_alpha_angle_degrees->display(text);

  text =
      QString::number(angle_deg2percent * lever_arm_angle[board_index], 'f', 2);
  ui->lcd_lever_arm_angle_percent->display(text);

  text = QString::number(lever_arm_angle[board_index], 'f', 2);
  ui->lcd_lever_arm_angle_degrees->display(text);

  text = QString::number(torque_Nm2percent * alpha_torque[board_index], 'f', 2);
  ui->lcd_alpha_torque_Nm_percent->display(text);

  text = QString::number(alpha_torque[board_index], 'f', 2);
  ui->lcd_alpha_torque_Nm->display(text);

  text = QString::number(board_id);
  ui->lcdId->display(text);
}

JointInformationGUI::~JointInformationGUI() { delete ui; }

void JointInformationGUI::update_joint_information_gui(
    struct JointInformationContainer jic)
{
  switch (jic.type)
  {
    case 0:
    {
      for (int i = 5; i >= 0; i--)
      {
        fix_link_angle[i] = jic.fix_link_angle[i];
        alpha_torque[i] = jic.alpha_torque[i];
      }

      // cout << "UJIG " << jic.board_id << "/0  |  fla = " <<  setfill(' ') <<
      // setw(7) << jic.fix_link_angle << endl;
      break;
    }
    case 1:
    {
      for (int i = 5; i >= 0; i--)
      {
        fix_link_angle[i] = jic.fix_link_angle[i];
        alpha_angle[i] = jic.alpha_angle[i];
        lever_arm_angle[i] = jic.lever_arm_angle[i];
      }

      // cout << "UJIG " << jic.board_id << "/2  |  fla = " <<  setfill(' ') <<
      // setw(7) << jic.fix_link_angle << "  |  aa = " << setfill(' ') <<
      // setw(7)
      // << jic.alpha_angle << endl;
      break;
    }
  }
}
