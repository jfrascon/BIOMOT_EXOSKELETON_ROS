/* Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com */

#ifndef COMMAND_GUI_H
#define COMMAND_GUI_H

#include <QDialog>

#include "h2r/FeetFSRSThresholdsMessage.h"
#include "h2r/SingleCommandMessage.h"
#include "ros/ros.h"

using namespace ros;
using namespace h2r;

namespace Ui
{
class CommandsGUI;
}

class CommandsGUI : public QDialog
{
  Q_OBJECT

 public:
  explicit CommandsGUI(QWidget* parent = 0);
  ~CommandsGUI();

 private slots:
  void on_kp_button_clicked();
  void on_ki_button_clicked();
  void on_kd_button_clicked();
  void on_setpoint_button_clicked();
  void on_precomp_button_clicked();
  void on_controller_cmb_currentIndexChanged(int controller_id);
  void on_lhip_rb_clicked();
  void on_lknee_rb_clicked();
  void on_lankle_rb_clicked();
  void on_rhip_rb_clicked();
  void on_rknee_rb_clicked();
  void on_rankle_rb_clicked();
  void on_disable_button_clicked();
  void on_cal_fix_link_angle_button_clicked();
  void on_cal_alpha_angle_button_clicked();
  void on_calibrate_encoders_button_clicked();
  void on_send_thresholds_button_clicked();

 private:
  Ui::CommandsGUI* ui;
  NodeHandle* ros_nh;
  /** Public data to the TX node. The TX node sends data through the CAN bus to
    * the exoskeleton.
    */
  Publisher single_cmd_pub;
  /** Public preasure thresholds to the node that manages the exoskeleton gait
    * cycle.
    */
  Publisher fsrs_thresholds_pub;

  SingleCommandMessage single_command_msg;
  FeetFSRSThresholdsMessage fsrs_thresholds_msg;
  int board_id;

  void process_k(QWidget* line_edit, char* error_message);
  void send_reference_position(int message_id_);
  // void select_board(int board_id);
};

#endif
