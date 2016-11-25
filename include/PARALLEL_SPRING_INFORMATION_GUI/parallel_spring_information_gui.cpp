/*
* Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
* Qt GUI used to observe the behaviour of parallel spring in each knee.
* In this GUI the user can observe the fix link angle of each knee, the number
* of revolutions accomplished by the motor with comprises the spring and if the
* spring is fully elongated. The spring elongation can be detected with a
* contact magnetic sensor.
*/

#include <QLCDNumber>
#include <QLabel>
#include <QString>
//#include <qmessagebox.h>
//#include <string>

#include "logObject.h"
#include "ros/ros.h"

#include "constants.h"
#include "parallel_spring_information_gui.h"
#include "ui_parallel_spring_information_gui.h"

using namespace std;

ParallelSpringInformation::ParallelSpringInformation()
    : ps_inf_cont{0.0f, 0.0f, 0, 0, 0, 0}
{
}

ParallelSpringInformation::ParallelSpringInformation(
    const ParallelSpringInformation& ps_inf)
    : ps_inf_cont(ps_inf.ps_inf_cont)
{
}
const ParallelSpringInformation& ParallelSpringInformation::operator=(
    const ParallelSpringInformation& ps_inf)
{
  ps_inf_cont = ps_inf.ps_inf_cont;
}

void ParallelSpringInformation::process_parallel_spring(
    const h2r::ParallelSpringMessage::ConstPtr& ps_message)
{

  ps_inf_cont.right_knee_fix_link_angle = ps_message->right_knee_fix_link_angle;
  ps_inf_cont.left_knee_fix_link_angle = ps_message->left_knee_fix_link_angle;
  ps_inf_cont.right_number_revolutions = ps_message->right_number_revolutions;
  ps_inf_cont.left_number_revolutions = ps_message->left_number_revolutions;
  ps_inf_cont.right_limit = ps_message->right_limit;
  ps_inf_cont.left_limit = ps_message->left_limit;

  emit parallel_spring_information_changed(ps_inf_cont);
}

// ----- ----- ----- ----- -----

ParallelSpringInformationGUI::ParallelSpringInformationGUI(QWidget* parent)
    : QMainWindow(parent),
      ui(new Ui::ParallelSpringInformationGUI),
      right_knee_fix_link_angle(0.0f),
      left_knee_fix_link_angle(0.0f),
      right_number_revolutions(0),
      left_number_revolutions(0),
      right_limit(0),
      left_limit(0)
{
  ui->setupUi(this);
  // QTimer *timer = new QTimer(this);
  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(showDatas()));
  dataTimer.start(
      100); /*Runs a timer every 100 milliseconds to update the LCD numbers*/
}

void ParallelSpringInformationGUI::showDatas() /*Updates data's output*/
{
  QString text = QString::number(left_knee_fix_link_angle, 'f', 2);
  ui->lcd_lfla->display(text);

  text = QString::number(right_knee_fix_link_angle, 'f', 2);
  ui->lcd_rfla->display(text);

  text = QString::number(left_number_revolutions);
  ui->lcd_lpsr->display(text);

  text = QString::number(right_number_revolutions);
  ui->lcd_rpsr->display(text);

  if (left_limit)
    ui->led1->setStyleSheet("background-color: rgb(255, 0, 0);");
  else
    ui->led1->setStyleSheet("background-color: rgb(120, 0, 0);");

  if (right_limit)
    ui->led2->setStyleSheet("background-color: rgb(255, 0, 0);");
  else
    ui->led2->setStyleSheet("background-color: rgb(120, 0, 0);");
}

ParallelSpringInformationGUI::~ParallelSpringInformationGUI() { delete ui; }

void ParallelSpringInformationGUI::update_parallel_spring_information_gui(
    struct ParallelSpringInformationContainer ps_inf_cont)
{
  left_knee_fix_link_angle = rad2grad * ps_inf_cont.left_knee_fix_link_angle;
  right_knee_fix_link_angle = rad2grad * ps_inf_cont.right_knee_fix_link_angle;
  left_number_revolutions = ps_inf_cont.left_number_revolutions;
  right_number_revolutions = ps_inf_cont.right_number_revolutions;
  left_limit = ps_inf_cont.left_limit;
  right_limit = ps_inf_cont.right_limit;
}
