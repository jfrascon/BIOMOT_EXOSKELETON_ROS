#ifndef PATTERNS_GUI_H
#define PATTERNS_GUI_H

/** Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  */
#include <sstream>

#include <QCheckBox>
#include <QDialog>
#include <QLineEdit>
#include <QMetaType>
#include <QPushButton>
#include <QRadioButton>
#include <QWidget>

#include <mutex>

#include "h2r/PatternsMessage.h"
#include "ros/ros.h"

using namespace std;

namespace Ui
{
class PatternsGUI;
}

class PatternsGUI : public QDialog
{
  Q_OBJECT

 public:
  explicit PatternsGUI(QWidget* parent = 0);
  ~PatternsGUI();

 public slots:
  void register_enabled_pattern_setpoints(int pattern_index,
                                          int setpoints_number_in_pattern);

 private slots:
  void on_send_button_clicked();
  void on_stop_button_clicked();

  void on_left_hip_pattern_cb_clicked();
  void on_left_knee_pattern_cb_clicked();
  void on_left_ankle_pattern_cb_clicked();
  void on_right_hip_pattern_cb_clicked();
  void on_right_knee_pattern_cb_clicked();
  void on_right_ankle_pattern_cb_clicked();

  // void on_sample_period_cmb_currentIndexChanged(int controller_id);

  void on_left_hip_pattern_file_select_button_clicked();
  void on_left_knee_pattern_file_select_button_clicked();
  void on_left_ankle_pattern_file_select_button_clicked();
  void on_right_hip_pattern_file_select_button_clicked();
  void on_right_knee_pattern_file_select_button_clicked();
  void on_right_ankle_pattern_file_select_button_clicked();

 private:
  Ui::PatternsGUI* ui;
  QPushButton* select_buttons_array[6];
  QLineEdit* pattern_line_editors_array[6];
  QCheckBox* pattern_check_boxes_array[6];
  int setpoints_number[6];
  bool enabled_patterns[6];

  ros::Publisher patterns_pub;
  h2r::PatternsMessage patterns_msg;

  std::vector<float> pattern_0;
  std::vector<float> pattern_1;
  std::vector<float> pattern_2;
  std::vector<float> pattern_3;
  std::vector<float> pattern_4;
  std::vector<float> pattern_5;
  std::vector<float>* patterns[6];
  std::vector<float>* patterns_in_msg[6];

  std::mutex file_reader_threads_mutex;
  int active_threads_number;

  void disable_or_enable_pattern(int pattern_index, bool status);
  void start_pattern_reading(int pattern_index);
  void open_file_chooser(int pattern_index);
  bool check_setpoints_number_in_pattern(int initial_pattern_index,
                                         int reference_setpoints_number);
};

#endif
