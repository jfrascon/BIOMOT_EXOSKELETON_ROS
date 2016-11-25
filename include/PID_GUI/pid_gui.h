/** Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com */

#ifndef PID_GUI_H
#define PID_GUI_H

#include "qcustomplot.h"  // the header file of QCustomPlot. Don't forget to add it to your project, if you use an IDE, so it gets compiled.
#include <QMainWindow>
#include <QTimer>

#include "h2r/PIDMessage.h"
#include "ros/ros.h"

struct PIDContainer
{
  // int board_id;
  float setpoint[6];
  float measured_output[6];
  float control_output[6];
};

Q_DECLARE_METATYPE(PIDContainer);

class PID : public QObject
{
  Q_OBJECT

 public:
  PIDContainer pid_container;

  PID();
  PID(const PID &pid);
  const PID &operator=(const PID &pid);
  void process_pid(const h2r::PIDMessage::ConstPtr &pid_message);

 signals:
  void pid_changed(struct PIDContainer pid_container);
};

namespace Ui
{
class PIDGUI;
}

class PIDGUI : public QMainWindow
{
  Q_OBJECT
 public:
  explicit PIDGUI(QWidget *parent = 0);
  ~PIDGUI();

  void setupDemo(int demoIndex);
  void setupRealtimeDataDemo(QCustomPlot *qcp_mo_ss, QCustomPlot *qcp_co);

 public slots:
  void update_pid_gui(struct PIDContainer pid_container);

 private slots:
  void realtimeDataSlot();
  void on_lhip_rb_clicked();
  void on_lknee_rb_clicked();
  void on_lankle_rb_clicked();
  void on_rhip_rb_clicked();
  void on_rknee_rb_clicked();
  void on_rankle_rb_clicked();

 private:
  void select_board_index();
  Ui::PIDGUI *ui;
  QString demoName;
  QTimer dataTimer;
  QCPItemTracer *itemDemoPhaseTracer;
  int currentDemoIndex;
  int board_id;
  int board_index;
  float setpoint[6];
  float control_output[6];
  float measured_output[6];
};

#endif
