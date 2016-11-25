/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * GUI for enabling/disabling pattern sending to the exoskeleton.
  * A walking pattern is a set of angular positions that each joint has to
  * follow within a frequency
  * to reproduce a human movement.
  * The pattern for each joint is located in a txt file or csv (comma separted
  * values). All the patterns together. The first pattern is for left hip. Then,
  * second pattern for left knee, then left ankle, then right hip, then right
  * knee and finally right ankle.
  * Comma separated values for each joint in a single line.
  * There is a constant that limits the number of values for a pattern
  * (num_values_pattern inconstant.h)
  * The GUI uses a second thread of execution for reading the pattern file that
  * is chosen by the user and with this thread avoid freezing the GUI during
  * the reading.
  */
#include <iostream>

#include <QFileDialog>
#include <qmessagebox.h>

#include "h2r/PatternsMessage.h"
#include "ros/ros.h"

#include "constants.h"
#include "patterns_gui.h"
#include "ui_patterns_gui.h"
#include "worker_thread_patterns_gui.h"

using namespace std;

PatternsGUI::PatternsGUI(QWidget* parent)
    : QDialog(parent),
      ui(new Ui::PatternsGUI()),
      setpoints_number{0, 0, 0, 0, 0, 0},
      enabled_patterns{false, false, false, false, false, false},
      active_threads_number(0),
      patterns_pub(ros::NodeHandle().advertise<h2r::PatternsMessage>(
          "patterns_topic", 1))
{
  ui->setupUi(this);

  ui->controller_cmb->setEditable(true);
  ui->controller_cmb->lineEdit()->setReadOnly(true);
  ui->controller_cmb->lineEdit()->setAlignment(Qt::AlignCenter);

  for (int i = 0; i < ui->controller_cmb->count(); i++)
    ui->controller_cmb->setItemData(i, Qt::AlignCenter, Qt::TextAlignmentRole);

  ui->sample_period_cmb->setEditable(true);
  ui->sample_period_cmb->lineEdit()->setReadOnly(true);
  ui->sample_period_cmb->lineEdit()->setAlignment(Qt::AlignCenter);

  for (int i = 0; i < ui->sample_period_cmb->count(); i++)
    ui->sample_period_cmb->setItemData(i, Qt::AlignCenter,
                                       Qt::TextAlignmentRole);

  patterns_msg.pattern_0.clear();
  patterns_msg.pattern_1.clear();
  patterns_msg.pattern_2.clear();
  patterns_msg.pattern_3.clear();
  patterns_msg.pattern_4.clear();
  patterns_msg.pattern_5.clear();

  pattern_0.clear();
  pattern_1.clear();
  pattern_2.clear();
  pattern_3.clear();
  pattern_4.clear();
  pattern_5.clear();

  pattern_check_boxes_array[0] = ui->left_hip_pattern_cb;
  pattern_check_boxes_array[1] = ui->left_knee_pattern_cb;
  pattern_check_boxes_array[2] = ui->left_ankle_pattern_cb;
  pattern_check_boxes_array[3] = ui->right_hip_pattern_cb;
  pattern_check_boxes_array[4] = ui->right_knee_pattern_cb;
  pattern_check_boxes_array[5] = ui->right_ankle_pattern_cb;

  pattern_line_editors_array[0] = ui->left_hip_pattern_line_edit;
  pattern_line_editors_array[1] = ui->left_knee_pattern_line_edit;
  pattern_line_editors_array[2] = ui->left_ankle_pattern_line_edit;
  pattern_line_editors_array[3] = ui->right_hip_pattern_line_edit;
  pattern_line_editors_array[4] = ui->right_knee_pattern_line_edit;
  pattern_line_editors_array[5] = ui->right_ankle_pattern_line_edit;

  select_buttons_array[0] = ui->left_hip_pattern_file_select_button;
  select_buttons_array[1] = ui->left_knee_pattern_file_select_button;
  select_buttons_array[2] = ui->left_ankle_pattern_file_select_button;
  select_buttons_array[3] = ui->right_hip_pattern_file_select_button;
  select_buttons_array[4] = ui->right_knee_pattern_file_select_button;
  select_buttons_array[5] = ui->right_ankle_pattern_file_select_button;

  patterns[0] = &pattern_0;
  patterns[1] = &pattern_1;
  patterns[2] = &pattern_2;
  patterns[3] = &pattern_3;
  patterns[4] = &pattern_4;
  patterns[5] = &pattern_5;

  patterns_in_msg[0] = &patterns_msg.pattern_0;
  patterns_in_msg[1] = &patterns_msg.pattern_1;
  patterns_in_msg[2] = &patterns_msg.pattern_2;
  patterns_in_msg[3] = &patterns_msg.pattern_3;
  patterns_in_msg[4] = &patterns_msg.pattern_4;
  patterns_in_msg[5] = &patterns_msg.pattern_5;

  bool conversion_success_sample_period = false;
  patterns_msg.sample_period = ui->sample_period_cmb->currentText().toInt(
      &conversion_success_sample_period);
  patterns_msg.controller_id = POS_PID_CONTROLLER_ID;

  for (int i = 0; i < 6; i++)
  {
    pattern_check_boxes_array[i]->setChecked(false);
    disable_or_enable_pattern(i, false);
  }
}

PatternsGUI::~PatternsGUI() { delete ui; }

void PatternsGUI::disable_or_enable_pattern(int pattern_index, bool status)
{
  pattern_line_editors_array[pattern_index]->setEnabled(status);
  select_buttons_array[pattern_index]->setEnabled(status);
  enabled_patterns[pattern_index] = status;
}

void PatternsGUI::on_left_hip_pattern_cb_clicked()
{
  if (ui->left_hip_pattern_cb->isChecked())
    disable_or_enable_pattern(LEFT_HIP_ID, true);
  else
    disable_or_enable_pattern(LEFT_HIP_ID, false);
}

void PatternsGUI::on_left_knee_pattern_cb_clicked()
{
  if (ui->left_knee_pattern_cb->isChecked())
    disable_or_enable_pattern(LEFT_KNEE_ID, true);
  else
    disable_or_enable_pattern(LEFT_KNEE_ID, false);
}

void PatternsGUI::on_left_ankle_pattern_cb_clicked()
{
  if (ui->left_ankle_pattern_cb->isChecked())
    disable_or_enable_pattern(LEFT_ANKLE_ID, true);
  else
    disable_or_enable_pattern(LEFT_ANKLE_ID, false);
}

void PatternsGUI::on_right_hip_pattern_cb_clicked()
{
  if (ui->right_hip_pattern_cb->isChecked())
    disable_or_enable_pattern(RIGHT_HIP_ID, true);
  else
    disable_or_enable_pattern(RIGHT_HIP_ID, false);
}

void PatternsGUI::on_right_knee_pattern_cb_clicked()
{
  if (ui->right_knee_pattern_cb->isChecked())
    disable_or_enable_pattern(RIGHT_KNEE_ID, true);
  else
    disable_or_enable_pattern(RIGHT_KNEE_ID, false);
}

void PatternsGUI::on_right_ankle_pattern_cb_clicked()
{
  if (ui->right_ankle_pattern_cb->isChecked())
    disable_or_enable_pattern(RIGHT_ANKLE_ID, true);
  else
    disable_or_enable_pattern(RIGHT_ANKLE_ID, false);
}

void PatternsGUI::on_left_hip_pattern_file_select_button_clicked()
{
  open_file_chooser(LEFT_HIP_ID);
}

void PatternsGUI::on_left_knee_pattern_file_select_button_clicked()
{
  open_file_chooser(LEFT_KNEE_ID);
}

void PatternsGUI::on_left_ankle_pattern_file_select_button_clicked()
{
  open_file_chooser(LEFT_ANKLE_ID);
}

void PatternsGUI::on_right_hip_pattern_file_select_button_clicked()
{
  open_file_chooser(RIGHT_HIP_ID);
}

void PatternsGUI::on_right_knee_pattern_file_select_button_clicked()
{
  open_file_chooser(RIGHT_KNEE_ID);
}

void PatternsGUI::on_right_ankle_pattern_file_select_button_clicked()
{
  open_file_chooser(RIGHT_ANKLE_ID);
}

void PatternsGUI::open_file_chooser(int pattern_index)
{
  QString filename = QFileDialog::getOpenFileName(
      this, tr("Open Setpoints File"), "~/", tr("TXT Files (*.txt)"));

  if (!filename.isNull())
  {
    pattern_line_editors_array[pattern_index]->setText(filename);
    select_buttons_array[pattern_index]->setEnabled(false);
    start_pattern_reading(pattern_index);
  }
}

void PatternsGUI::start_pattern_reading(int pattern_index)
{
  std::vector<float>* pattern = patterns[pattern_index];
  QLineEdit* pattern_file = pattern_line_editors_array[pattern_index];
  WorkerThread* worker_thread =
      new WorkerThread(pattern_file->text(), pattern_index, *pattern,
                       file_reader_threads_mutex, active_threads_number);
  connect(worker_thread, SIGNAL(pattern_loaded(int, int)), this,
          SLOT(register_enabled_pattern_setpoints(int, int)));
  connect(worker_thread, SIGNAL(finished()), worker_thread,
          SLOT(deleteLater()));
  worker_thread->start();  // This invokes WorkerThread::run in a new thread
}

void PatternsGUI::register_enabled_pattern_setpoints(
    int pattern_index, int setpoints_number_in_pattern)
{
  select_buttons_array[pattern_index]->setEnabled(true);
  setpoints_number[pattern_index] = setpoints_number_in_pattern;
  check_setpoints_number_in_pattern(0, setpoints_number_in_pattern);
}

bool PatternsGUI::check_setpoints_number_in_pattern(
    int initial_pattern_index, int setpoints_number_in_pattern)
{
  bool different_number_of_setpoints = false;

  ostringstream error_oss;
  error_oss << "Number of setpoints in the files differs." << endl << endl;

  for (int i = initial_pattern_index; i < 6; i++)
  {
    if (enabled_patterns[i])
    {
      if (setpoints_number[i])
      {
        if (setpoints_number[i] != setpoints_number_in_pattern)
          different_number_of_setpoints = true;

        error_oss << "Number of setpoints in pattern " << i << ": "
                  << setfill(' ') << setw(3) << setpoints_number[i] << endl;
      }
      else
      {
        pattern_check_boxes_array[i]->setChecked(false);
        disable_or_enable_pattern(i, false);
      }
    }
  }

  if (different_number_of_setpoints)
    QMessageBox::critical(this, tr("ERROR"), tr(error_oss.str().c_str()));

  return different_number_of_setpoints;
}

void PatternsGUI::on_send_button_clicked()
{
  int i;
  int disabled_patterns_number = 0;
  int setpoints_number_in_pattern;
  bool threads_reading = false;

  std::unique_lock<std::mutex> mlock(file_reader_threads_mutex);
  if (active_threads_number)
    threads_reading = true;
  mlock.unlock();

  if (threads_reading)
  {
    QMessageBox::critical(this, tr("ERROR"),
                          tr("Setpoints file reading in progress.\nPlease wait "
                             "a few seconds before trying to send a message"));
    return;
  }

  for (i = 0; i < 6; i++)
  {
    if (enabled_patterns[i])
    {
      if (setpoints_number[i])
      {
        setpoints_number_in_pattern = setpoints_number[i];
        break;
      }
      else
      {
        pattern_check_boxes_array[i]->setChecked(false);
        disable_or_enable_pattern(i, false);
      }
    }
    disabled_patterns_number++;
  }

  if (disabled_patterns_number == 6)
    return;

  if (check_setpoints_number_in_pattern(i, setpoints_number_in_pattern))
    return;  // Nothing to send beacuse of an error in number of setpoints in
             // file

  for (int i = 0; i < 6; i++)
  {
    patterns_in_msg[i]->clear();
    if (enabled_patterns[i])
    {
      *patterns_in_msg[i] = *patterns[i];
    }
  }

  patterns_msg.controller_id = ui->controller_cmb->currentIndex();
  bool conversion_success_sample_period;
  patterns_msg.sample_period = ui->sample_period_cmb->currentText().toInt(
      &conversion_success_sample_period);
  patterns_pub.publish(patterns_msg);
}

void PatternsGUI::on_stop_button_clicked()
{

  patterns_msg.controller_id = ui->controller_cmb->currentIndex();
  float out_of_range_value;

  switch (patterns_msg.controller_id)
  {
    case POS_PID_CONTROLLER_ID:
    {
      out_of_range_value = 1.5f * max_angle_deg;
      break;
    }
    case LVL_PID_CONTROLLER_ID:
    {
      out_of_range_value = 1.5f * max_lvl_deg;
      break;
    }
    case TOR_PID_CONTROLLER_ID:
    {
      out_of_range_value = 1.5f * max_tor_Nm;
      break;
    }
    case TCL_PID_CONTROLLER_ID:
    {
      out_of_range_value = 1.5f * max_angle_deg;
      break;
    }
    default:
    {
      return;
    }
  }

  for (int i = 0; i < 6; i++)
  {
    patterns_in_msg[i]->clear();
    patterns_in_msg[i]->push_back(out_of_range_value);
  }

  bool conversion_success_sample_period;
  patterns_msg.sample_period = ui->sample_period_cmb->currentText().toInt(
      &conversion_success_sample_period);
  patterns_pub.publish(patterns_msg);
}
