/*
* Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
* Qt Window with 4 auxiliary variables used to configure different control
* algorithms running in a Matlab instance located in a High Performance
* Computer.
*/

#include <cfloat>

#include <QLineEdit>
#include <qmessagebox.h>

#include "aux_var_gui.h"
#include "constants.h"
#include "ui_aux_var_gui.h"

AuxVarGUI::AuxVarGUI(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::AuxVarGUI), board_id(LEFT_HIP_ID)
{
  ros::NodeHandle n;
  sender_pub = n.advertise<h2r::SingleCommandMessage>("tx_topic", 1);

  ui->setupUi(this);
  // Default joint to which the variable is actuating in the control algorithm.
  ui->lhip_rb->setChecked(true);
  // The variable is only actuating in one joint.
  single_command_msg.number_elements = 1;
  single_command_msg.can_id.assign(6, 0);
  single_command_msg.can_id[0] = board_id + AUX_VAR_MESSAGE_ID;
  single_command_msg.type = AuxVarGUI::AUX_VAR_TYPE_0;
  single_command_msg.data.assign(6, 0.0f);
  single_command_msg.data[0] = 1.5f;
}

AuxVarGUI::~AuxVarGUI() { delete ui; }

// Select the board clicked.
void AuxVarGUI::on_lhip_rb_clicked() { board_id = LEFT_HIP_ID; }

void AuxVarGUI::on_lknee_rb_clicked() { board_id = LEFT_KNEE_ID; }

void AuxVarGUI::on_lankle_rb_clicked() { board_id = LEFT_ANKLE_ID; }

void AuxVarGUI::on_rhip_rb_clicked() { board_id = RIGHT_HIP_ID; }

void AuxVarGUI::on_rknee_rb_clicked() { board_id = RIGHT_KNEE_ID; }

void AuxVarGUI::on_rankle_rb_clicked() { board_id = RIGHT_ANKLE_ID; }

void AuxVarGUI::on_v0_send_button_clicked()
{
  process_var(ui->v0_line_edit, AuxVarGUI::AUX_VAR_TYPE_0);
}

void AuxVarGUI::on_v1_send_button_clicked()
{
  process_var(ui->v1_line_edit, AuxVarGUI::AUX_VAR_TYPE_1);
}

void AuxVarGUI::on_v2_send_button_clicked()
{
  process_var(ui->v2_line_edit, AuxVarGUI::AUX_VAR_TYPE_2);
}

void AuxVarGUI::on_v3_send_button_clicked()
{
  process_var(ui->v3_line_edit, AuxVarGUI::AUX_VAR_TYPE_3);
}

/* Send the value through the ROS network to the TX node, which is connected
 * to the CAN bus. The variable type is also sent in the ROS message and later
 * in the CAN packet, so the algorithm running in Matlab knows how to manage
 * that value.
 */
void AuxVarGUI::process_var(QWidget *line_edit, int type)
{
  bool conversion_success = true;
  single_command_msg.number_elements = 1;
  single_command_msg.can_id[0] = board_id + AUX_VAR_MESSAGE_ID;
  single_command_msg.type = type;
  single_command_msg.data[0] =
      ((QLineEdit *)line_edit)->text().toFloat(&conversion_success);
  sender_pub.publish(single_command_msg);
}
