#ifndef PID_INFORMATION_GUI_H
#define PID_INFORMATION_GUI_H

/*
* Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
* Qt GUI used to observe the behaviour of parallel spring in each knee.
* In this GUI the user can observe the fix link angle of each knee, the number
* of revolutions accomplished by the motor with comprises the spring and if the
* spring is fully elongated. The spring elongation can be detected with a
* contact magnetic sensor.
*/

#include <QMainWindow>
#include <QMetaType>
#include <QTimer>

#include "h2r/ParallelSpringMessage.h"

struct ParallelSpringInformationContainer
{
  float right_knee_fix_link_angle;
  float left_knee_fix_link_angle;
  int right_number_revolutions;
  int left_number_revolutions;
  int right_limit;
  int left_limit;
};

Q_DECLARE_METATYPE(ParallelSpringInformationContainer);

class ParallelSpringInformation : public QObject
{
  Q_OBJECT

 public:
  ParallelSpringInformationContainer ps_inf_cont;

  ParallelSpringInformation();
  ParallelSpringInformation(const ParallelSpringInformation& ps_inf);
  const ParallelSpringInformation& operator=(
      const ParallelSpringInformation& ps_inf);
  void process_parallel_spring(
      const h2r::ParallelSpringMessage::ConstPtr& ps_message);

 signals:
  void parallel_spring_information_changed(
      struct ParallelSpringInformationContainer);
};

namespace Ui
{
class ParallelSpringInformationGUI;
}

class ParallelSpringInformationGUI : public QMainWindow
{
  Q_OBJECT

 public:
  explicit ParallelSpringInformationGUI(QWidget* parent = 0);
  ~ParallelSpringInformationGUI();

 public slots:
  void update_parallel_spring_information_gui(
      struct ParallelSpringInformationContainer);

 private slots:
  void showDatas();

 private:
  Ui::ParallelSpringInformationGUI* ui;
  QTimer dataTimer;

  float right_knee_fix_link_angle;
  float left_knee_fix_link_angle;
  int right_number_revolutions;
  int left_number_revolutions;
  int right_limit;
  int left_limit;
};

#endif
