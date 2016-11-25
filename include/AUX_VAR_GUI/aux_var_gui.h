/* Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
 */
#ifndef COMMAND_GUI_H
#define COMMAND_GUI_H

#include <QMainWindow>

#include "h2r/SingleCommandMessage.h"
#include "ros/ros.h"

namespace Ui
{
class AuxVarGUI;
}

class AuxVarGUI : public QMainWindow
{
  Q_OBJECT

 public:
  explicit AuxVarGUI(QWidget* parent = 0);
  ~AuxVarGUI();

 private slots:
  void on_v0_send_button_clicked();
  void on_v1_send_button_clicked();
  void on_v2_send_button_clicked();
  void on_v3_send_button_clicked();
  void on_lhip_rb_clicked();
  void on_lknee_rb_clicked();
  void on_lankle_rb_clicked();
  void on_rhip_rb_clicked();
  void on_rknee_rb_clicked();
  void on_rankle_rb_clicked();
  void process_var(QWidget* line_edit, int type);

 private:
  Ui::AuxVarGUI* ui;
  ros::Publisher sender_pub;
  h2r::SingleCommandMessage single_command_msg;
  // Board id selected in the GUI. The data entered
  // in the GUI is sent to this joint.
  int board_id;

  // Variable type. The user decides which variable is associated
  // with which algorithm
  enum AUX_VAR_TYPE
  {
    AUX_VAR_TYPE_0 = 0,
    AUX_VAR_TYPE_1 = 1,
    AUX_VAR_TYPE_2 = 2,
    AUX_VAR_TYPE_3 = 3
  };
};

#endif
