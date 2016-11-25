/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * Qt GUI used to display in 'realtime' the numeric value of:
  *   The setpoint for a specified joint, in natural units and in percent with
  *   respect its maximum.
  *   The measured output for a specified joint, in natural units and in percent
  *   with respect its maximum.
  *   The control output for a specified joint, in natural units and in percent
  *   with respect its maximum.
  * It also display the board id from which the information is gathered.
  */

#include <QLCDNumber>
#include <QString>
#include <qmessagebox.h>
#include <string>

#include "h2r/PIDMessage.h"
#include "logObject.h"
#include "ros/ros.h"

#include "constants.h"
#include "pid_information_gui.h"
#include "ui_pid_information_gui.h"

using namespace std;

PIDInformation::PIDInformation()
    : pic{{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
          {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
          {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}
{
}
PIDInformation::PIDInformation(const PIDInformation& pid) : pic(pid.pic) {}
const PIDInformation& PIDInformation::operator=(const PIDInformation& pid)
{
  pic = pid.pic;
}

void PIDInformation::process_pid(const h2r::PIDMessage pid_message)
{
  for (int i = 5; i >= 0; i--)
  {
    pic.setpoint[i] = pid_message.setpoint[i];
    pic.measured_output[i] = pid_message.measured_output[i];
    pic.control_output[i] = pid_message.control_output[i];
  }

  emit pid_information_changed(pic);
}

PIDInformationGUI::PIDInformationGUI(QWidget* parent)
    : QMainWindow(parent),
      ui(new Ui::PIDInformationGUI),
      board_id(LEFT_HIP_ID),
      board_index((board_id - LEFT_HIP_ID) / LEFT_HIP_ID),
      setpoint{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      control_output{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      measured_output{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
{
  ui->setupUi(this);
  ui->lhip_rb->setChecked(true);
  // QTimer *timer = new QTimer(this);
  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(showDatas()));
  dataTimer.start(
      100); /*Runs a timer every 100 milliseconds to update the LCD numbers*/
}

void PIDInformationGUI::select_board_index()
{
  board_index = (board_id - LEFT_HIP_ID) / LEFT_HIP_ID;
}

void PIDInformationGUI::on_lhip_rb_clicked()
{
  // select_board(LEFT_HIP_ID);
  board_id = LEFT_HIP_ID;
  select_board_index();
}

void PIDInformationGUI::on_lknee_rb_clicked()
{
  // select_board(LEFT_KNEE_ID);
  board_id = LEFT_KNEE_ID;
  select_board_index();
}

void PIDInformationGUI::on_lankle_rb_clicked()
{
  // select_board(LEFT_ANKLE_ID);
  board_id = LEFT_ANKLE_ID;
  select_board_index();
}

void PIDInformationGUI::on_rhip_rb_clicked()
{
  // select_board(RIGHT_HIP_ID);
  board_id = RIGHT_HIP_ID;
  select_board_index();
}

void PIDInformationGUI::on_rknee_rb_clicked()
{
  // select_board(RIGHT_KNEE_ID);
  board_id = RIGHT_KNEE_ID;
  select_board_index();
}

void PIDInformationGUI::on_rankle_rb_clicked()
{
  // select_board(RIGHT_ANKLE_ID);
  board_id = RIGHT_ANKLE_ID;
  select_board_index();
}

void PIDInformationGUI::showDatas() /*Updates data's output*/
{
  QString text = QString::number(setpoint[board_index], 'f', 2);
  ui->lcd_setpoint->display(text);

  text = QString::number(control_output[board_index], 'f', 2);
  ui->lcd_control_output->display(text);

  text = QString::number(measured_output[board_index], 'f', 2);
  ui->lcd_measured_output->display(text);

  text = QString::number(board_id);
  ui->lcdId->display(text);
}

PIDInformationGUI::~PIDInformationGUI() { delete ui; }

void PIDInformationGUI::update_pid_information_gui(
    struct PIDInformationContainer pic)
{
  for (int i = 5; i >= 0; i--)
  {
    setpoint[i] = pic.setpoint[i];
    control_output[i] = pic.control_output[i];
    measured_output[i] = pic.measured_output[i];
  }
}
