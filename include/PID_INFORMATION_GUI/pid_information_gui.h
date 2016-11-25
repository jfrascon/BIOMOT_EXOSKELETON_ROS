/** Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com */

#ifndef PID_INFORMATION_GUI_H
#define PID_INFORMATION_GUI_H

#include "h2r/PIDMessage.h"
#include <QMainWindow>
#include <QMetaType>
#include <QTimer>

struct PIDInformationContainer
{
  // int board_id;
  float setpoint[6];
  float control_output[6];
  float measured_output[6];
};

Q_DECLARE_METATYPE(PIDInformationContainer);

class PIDInformation : public QObject
{
  Q_OBJECT

 public:
  PIDInformationContainer pic;

  PIDInformation();
  PIDInformation(const PIDInformation& pid);
  const PIDInformation& operator=(const PIDInformation& pid);
  void process_pid(const h2r::PIDMessage pid_message);

 signals:
  void pid_information_changed(struct PIDInformationContainer pid_container);
};

namespace Ui
{
class PIDInformationGUI;
}

class PIDInformationGUI : public QMainWindow
{
  Q_OBJECT

 public:
  explicit PIDInformationGUI(QWidget* parent = 0);
  ~PIDInformationGUI();

 public slots:
  void update_pid_information_gui(struct PIDInformationContainer);

 private slots:
  void showDatas();
  void on_lhip_rb_clicked();
  void on_lknee_rb_clicked();
  void on_lankle_rb_clicked();
  void on_rhip_rb_clicked();
  void on_rknee_rb_clicked();
  void on_rankle_rb_clicked();

 private:
  void select_board_index();
  Ui::PIDInformationGUI* ui;
  int board_id;
  int board_index;
  QTimer dataTimer;
  float setpoint[6];
  float measured_output[6];
  float control_output[6];
};

#endif
