/*
* Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
* Qt GUI used to send commands to the electronic boards that manage the joints:
*   Kp, Ki, Kd constants for the PID algorithms running in the joints.
*   Torque setpoints in Nm and position setpoints in degrees for PID algorithms.
*   Calibration of magnetic encoders and optical encoders -> set zero position.
*   Choose the type of algorithm running in each joint.
*   Introduce precompression of springs in each joint.
*   Introduce threshold levels for FSRs in each foot.
*   Disable controllers anytime if something happens suddenly.
*/

#include <cfloat>

#include <QLineEdit>
#include <qmessagebox.h>

#include "commands_gui.h"
#include "constants.h"
#include "ui_commands_gui.h"

#include <climits>

CommandsGUI::CommandsGUI(QWidget *parent)
    : QDialog(parent),
      ui(new Ui::CommandsGUI),
      board_id(LEFT_HIP_ID),
      ros_nh(new NodeHandle()),
      single_cmd_pub(ros_nh->advertise<SingleCommandMessage>("tx_topic", 1)),
      fsrs_thresholds_pub(ros_nh->advertise<FeetFSRSThresholdsMessage>(
          "fsrs_thesholds_topic", 1))

{
  ui->setupUi(this);
  ui->controller_cmb->setEditable(true);
  ui->controller_cmb->lineEdit()->setReadOnly(true);
  ui->controller_cmb->lineEdit()->setAlignment(Qt::AlignCenter);

  for (int i = 0; i < ui->controller_cmb->count(); i++)
    ui->controller_cmb->setItemData(i, Qt::AlignCenter, Qt::TextAlignmentRole);

  ui->lhip_rb->setChecked(true);
  // Default controller: Position controller based on fix link angle position.
  single_command_msg.type = POS_PID_CONTROLLER_ID;
  // Number of CAN packets to be sent by the TX node. Only one packet is sent
  // to the specified joint board.
  single_command_msg.number_elements = 1;
  single_command_msg.can_id.assign(6, 0);
  // can_id = boar_id + message_id
  single_command_msg.can_id[0] = board_id + SETPOINT_MESSAGE_ID;
  single_command_msg.data.assign(6, 0.0f);
  // Default setpoint: 1.5 (150% of actuation). With this setpoint
  // a joint is programmed to get disabled.
  single_command_msg.data[0] = 1.5;

  // Default thresholds for feet preasure sensors.
  fsrs_thresholds_msg.left_toe_threshold = INT_MAX;
  fsrs_thresholds_msg.left_heel_threshold = INT_MAX;
  fsrs_thresholds_msg.right_toe_threshold = INT_MAX;
  fsrs_thresholds_msg.right_heel_threshold = INT_MAX;
}

CommandsGUI::~CommandsGUI()
{
  delete ros_nh;
  delete ui;
}

void CommandsGUI::on_lhip_rb_clicked() { board_id = LEFT_HIP_ID; }

void CommandsGUI::on_lknee_rb_clicked() { board_id = LEFT_KNEE_ID; }

void CommandsGUI::on_lankle_rb_clicked() { board_id = LEFT_ANKLE_ID; }

void CommandsGUI::on_rhip_rb_clicked() { board_id = RIGHT_HIP_ID; }

void CommandsGUI::on_rknee_rb_clicked() { board_id = RIGHT_KNEE_ID; }

void CommandsGUI::on_rankle_rb_clicked() { board_id = RIGHT_ANKLE_ID; }

/** A patient wears the exoskeleton and place the body in a comfortable
* reference position. Then the system user click the calibrate encoder
* button and all the encoders are set to zero position, I mean, initial
* position or reference position.
 */
void CommandsGUI::on_calibrate_encoders_button_clicked()
{
  // Optical and magnetic encoders in all joints have to be calibrated.
  // So a calibrate can frame for each joint has to be sent. Total: 6 can
  // frames.
  single_command_msg.number_elements = 6;

  single_command_msg.can_id[0] = LEFT_HIP_ID + ENCODERS_CALIBRATION_MESSAGE_ID;
  single_command_msg.can_id[1] = LEFT_KNEE_ID + ENCODERS_CALIBRATION_MESSAGE_ID;
  single_command_msg.can_id[2] =
      LEFT_ANKLE_ID + ENCODERS_CALIBRATION_MESSAGE_ID;
  single_command_msg.can_id[3] = RIGHT_HIP_ID + ENCODERS_CALIBRATION_MESSAGE_ID;
  single_command_msg.can_id[4] =
      RIGHT_KNEE_ID + ENCODERS_CALIBRATION_MESSAGE_ID;
  single_command_msg.can_id[5] =
      RIGHT_ANKLE_ID + ENCODERS_CALIBRATION_MESSAGE_ID;

  single_cmd_pub.publish(single_command_msg);
}

// The encoder sensor of the selected joint in the GUI is only the one
// calibrated.
void CommandsGUI::on_cal_fix_link_angle_button_clicked()
{
  send_reference_position(REF_FIX_LINK_ANGLE_MESSAGE_ID);
}

// The magnetic position sensor of the selected joint in the GUI is the only one
// calibrated.
void CommandsGUI::on_cal_alpha_angle_button_clicked()
{
  send_reference_position(REF_ALPHA_ANGLE_MESSAGE_ID);
}

/** This function only sends one position sensor calibration message to the
  * specified joint in the GUI. This message is sent to the TX node, then
  * the TX node send this data through the CAN bus.
 */
void CommandsGUI::send_reference_position(int message_id_)
{
  single_command_msg.number_elements = 1;
  single_command_msg.can_id[0] = board_id + message_id_;
  single_cmd_pub.publish(single_command_msg);
}

/** This function sends thresholds for the feet preasure sensors
  * (FSR - foot switch resistors) located in the exoeskeleton.
  * One fsr is located in the heel and another is located in the forefoot.
  * A threshold represent a preasure value. Any preasure value given
  * by an FSR above this threshold means that the FSR is being pressed.
  * Any preasure value given by an FSR below this threshold means that the FSR
  * is being realsed. Analizing the preasure value given by the heel FSR and
  * the forefoot FSR the system can determine in which part of the gait cycle
  * the patient is.
  * Thresholds depend on each patient wearing the exoeskeleton.
  * Thresholds are introduced without units, values between 0 and
  * 4095 (because of 12-bit adc: 4096 values from 0 to 4095).
  * The GUI nodes are located in a normal PC.
  * The node that manages the gait cycle is located in the BeagleBone
  * Black, so the thresholds must be sent in a ROS message.
 */
void CommandsGUI::on_send_thresholds_button_clicked()
{
  bool conversion_success = true;
  fsrs_thresholds_msg.left_toe_threshold =
      ui->left_toe_line_edit->text().toFloat(&conversion_success);
  fsrs_thresholds_msg.left_heel_threshold =
      ui->left_heel_line_edit->text().toFloat(&conversion_success);
  fsrs_thresholds_msg.right_toe_threshold =
      ui->right_toe_line_edit->text().toFloat(&conversion_success);
  fsrs_thresholds_msg.right_heel_threshold =
      ui->right_heel_line_edit->text().toFloat(&conversion_success);

  // Any value above 4095 is considered 4095 in the node that manages the gait
  // cycle.
  if (!conversion_success || fsrs_thresholds_msg.left_toe_threshold < 0 ||
      fsrs_thresholds_msg.left_heel_threshold < 0 ||
      fsrs_thresholds_msg.right_toe_threshold < 0 ||
      fsrs_thresholds_msg.right_heel_threshold < 0)
    QMessageBox::critical(
        this, tr("ERROR"),
        tr("Invalid threshold(s) found. They must be positive integers"));
  else
    single_cmd_pub.publish(fsrs_thresholds_msg);
}

// Store the controller type to be sent to the TX node.
void CommandsGUI::on_controller_cmb_currentIndexChanged(int controller_id)
{
  single_command_msg.type = controller_id;
}

/** Get the value of the Kp, Ki and Kd constants
  */
void CommandsGUI::on_kp_button_clicked()
{
  single_command_msg.can_id[0] = board_id + KP_MESSAGE_ID;
  char error_message[100];
  sprintf(error_message, "Kp out of range\nRange from %f to %f", Kmin, Kmax);
  process_k(ui->kp_line_edit, error_message);
}

void CommandsGUI::on_ki_button_clicked()
{
  single_command_msg.can_id[0] = board_id + KI_MESSAGE_ID;
  char error_message[100];
  sprintf(error_message, "Ki out of range\nRange from %f to %f", Kmin, Kmax);
  process_k(ui->ki_line_edit, error_message);
}

void CommandsGUI::on_kd_button_clicked()
{
  single_command_msg.can_id[0] = board_id + KD_MESSAGE_ID;
  char error_message[100];
  sprintf(error_message, "Kd out of range\nRange from %f to %f", Kmin, Kmax);
  process_k(ui->kd_line_edit, error_message);
}

/** Check correct value for PID constants: Kd, Ki, Kd.
  * All of them between Kmin and Kmax.
  */
void CommandsGUI::process_k(QWidget *line_edit, char *error_message)
{

  bool conversion_success = true;
  // Number of CAN packets to be sent later by the TX node into the CAN bus.
  single_command_msg.number_elements = 1;
  // Data to be sent to the TX node. Then this node will send this data through
  // the CAN bus to the joint board who is identificated with the id stored in
  // single_command_msg.message_id[0]
  single_command_msg.data[0] =
      ((QLineEdit *)line_edit)->text().toFloat(&conversion_success);

  if (!conversion_success || single_command_msg.data[0] < Kmin ||
      single_command_msg.data[0] > Kmax)
    QMessageBox::critical(this, tr("ERROR"), tr(error_message));
  else
  {
    // Send a ROS message to the TX node.
    // The TX node will send the data to the corresponding joint board through
    // the CAN bus.
    single_cmd_pub.publish(single_command_msg);
  }
}

/** Check the value of the introduced setpoint and normalized it
  * using the maximum value permited for a specific type of control algorithm.
  * Joint boards expect setpoint between -1.0 (which represents -100% of
  * actuation) and +0.99 (99% of actuation)
  * Joint boards use fixed point arithmetic in format Q1.15, which means
  * integer values between -32768 and 32767 represent float values between -1.0f
  * (-32768/32768) and 0.9999.. (32767/32768).
  * Note: 2^15 = 32768.
  */
void CommandsGUI::on_setpoint_button_clicked()
{
  single_command_msg.can_id[0] = board_id + SETPOINT_MESSAGE_ID;
  float lower_limit = 0.0f;
  float upper_limit = 0.0f;
  float normalization_factor = 0.0f;
  char text[100] = {'\0'};

  // Get maximum and minimum value for a setpoint in a specific control
  // algorithm.
  switch (single_command_msg.type)
  {
    // Alpha angle position controller.
    case POS_PID_CONTROLLER_ID:
    {
      lower_limit = lower_angle_setpoint * max_angle_deg;
      upper_limit = upper_angle_setpoint * max_angle_deg;
      normalization_factor = +max_angle_deg;
      sprintf(text,
              "Setpoint out of range\nRange from %f degrees to %f degrees",
              lower_limit, upper_limit);
      break;
    }
    // Lever arm angle position controller.
    case LVL_PID_CONTROLLER_ID:
    {
      lower_limit = lower_lvl_setpoint * max_lvl_deg;
      upper_limit = upper_lvl_setpoint * max_lvl_deg;
      normalization_factor = +max_lvl_deg;
      sprintf(text, "Setpoint out of range\nRange from %f rpm to %f rpm",
              lower_limit, upper_limit);
      break;
    }
    // Torque controller.
    case TOR_PID_CONTROLLER_ID:
    {
      lower_limit = lower_tor_setpoint * max_tor_Nm;
      upper_limit = upper_tor_setpoint * max_tor_Nm;
      normalization_factor = +max_tor_Nm;
      sprintf(text, "Setpoint out of range\nRange from %f Nm to %f Nm",
              lower_limit, upper_limit);
      break;
    }
    // Tacit learning controller (Complex controller running in a HPC with
    // Matlab)
    case TCL_PID_CONTROLLER_ID:
    {
      lower_limit = lower_angle_setpoint * max_angle_deg;
      upper_limit = upper_angle_setpoint * max_angle_deg;
      normalization_factor = +max_angle_deg;
      sprintf(text,
              "Setpoint out of range\nRange from %f degrees to %f degrees",
              lower_limit, upper_limit);
      break;
    }
  }
  // Check if setpoint is in the correct range. Otherwise norify the error to
  // the user.
  bool conversion_success = true;
  single_command_msg.number_elements = 1;
  single_command_msg.data[0] =
      ui->setpoint_line_edit->text().toFloat(&conversion_success);

  if (!conversion_success || single_command_msg.data[0] < lower_limit ||
      single_command_msg.data[0] > upper_limit)
    QMessageBox::critical(this, tr("ERROR"), tr(text));
  else
  {
    // setpoint from -1.0 to 0.9999389648  => format Q1.15 (fixed point
    // arithmetic).
    single_command_msg.data[0] /= normalization_factor;
    // single_command_msg.data[0] / normalization_factor;
    single_cmd_pub.publish(single_command_msg);
  }
}

/** Change the precompression in a specific joint spring.
*/
void CommandsGUI::on_precomp_button_clicked()
{
  single_command_msg.can_id[0] = board_id + PRECOMPRESSION_MESSAGE_ID;
  bool conversion_success = true;
  single_command_msg.number_elements = 1;
  single_command_msg.data[0] =
      ui->precomp_line_edit->text().toFloat(&conversion_success);
  char text[100] = {'\0'};
  float lower_limit = lower_precompression_limit;
  float upper_limit = upper_precompression_limit;

  if (!conversion_success || single_command_msg.data[0] < lower_limit ||
      single_command_msg.data[0] > upper_limit)
  {
    sprintf(text, "Precompression out of range\nRange from %f mm to %f mm",
            lower_limit, upper_limit);
    QMessageBox::critical(this, tr("ERROR"), tr(text));
  }
  else
  {
    single_command_msg.data[0] /= 1000.00f;  // In m, not in mm
    // single_command_msg.data[0] / 1000.00f;
    single_cmd_pub.publish(single_command_msg);
  }
}

/** Disable all controllers in the exoskeleton joints with a single click.
  * Every single setpoint is between -1.0f (wich represent 100% of actuation)
  * and 0.9999389648 (99.99% of actuation). So if we deliberately set
  * a setpoint to 1.5f (150% of actuation) the PID controllers in a joint gets
  * disabled.
  * This behavour is programmed inside the firmware joint.
  */
void CommandsGUI::on_disable_button_clicked()
{
  single_command_msg.can_id[0] = board_id + SETPOINT_MESSAGE_ID;
  single_command_msg.number_elements = 1;
  single_command_msg.data[0] = 1.5f;
  single_cmd_pub.publish(single_command_msg);
}
