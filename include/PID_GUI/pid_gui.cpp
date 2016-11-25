/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * Qt GUI used to plot in 'realtime' the PID parameters of the selected joint.
  * These parameters are:
  *     The setpoint for that joint.
  *     The measured output for that joint for the applied controller.
  *     The control output applied by the controller.
  * This Qt GUI uses the QCustomPlot library to plot functions.
  */

#include <string>

#include <QDesktopWidget>
#include <QMessageBox>
#include <QMetaEnum>
#include <QScreen>

#include "constants.h"
#include "pid_gui.h"
#include "ui_pid_gui.h"

#include <thread>

using namespace std;

PID::PID()
    : pid_container{{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}
{
}
PID::PID(const PID &pid) : pid_container(pid.pid_container) {}
const PID &PID::operator=(const PID &pid) { pid_container = pid.pid_container; }

void PID::process_pid(const h2r::PIDMessage::ConstPtr &pid_message)
{
  //  pc.board_id = std::stoi(pid_message->header.frame_id);
  for (int i = 5; i >= 0; i--)
  {
    pid_container.setpoint[i] = pid_message->setpoint[i];
    pid_container.measured_output[i] = pid_message->measured_output[i];
    pid_container.control_output[i] = pid_message->control_output[i];
  }
  emit pid_changed(pid_container);
}

PIDGUI::PIDGUI(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::PIDGUI),
      board_id(LEFT_HIP_ID),
      board_index((board_id - LEFT_HIP_ID) / LEFT_HIP_ID),
      setpoint{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      control_output{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      measured_output{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
{
  ui->setupUi(this);
  ui->lhip_rb->setChecked(true);
  setGeometry(400, 250, 542, 390);
  setupDemo(10);
}

PIDGUI::~PIDGUI() { delete ui; }

void PIDGUI::setupDemo(int demoIndex)
{
  switch (demoIndex)
  {
    case 10:
      setupRealtimeDataDemo(ui->qcp_mo_ss, ui->qcp_co);
      break;
  }
  setWindowTitle(demoName);
  statusBar()->clearMessage();
  currentDemoIndex = demoIndex;
  ui->qcp_mo_ss->replot();
  ui->qcp_co->replot();
}

void PIDGUI::setupRealtimeDataDemo(QCustomPlot *qcp_mo_ss, QCustomPlot *qcp_co)
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  QMessageBox::critical(this, "",
                        "You're using Qt < 4.7, this program needs "
                        "functions that are available with Qt 4.7 to "
                        "work properly");
#endif
  demoName = "Realtime PID GUI";

  // include this section to fully disable antialiasing for higher performance:
  /*
  qcp_mo_ss->setNotAntialiasedElements(QCP::aeAll);
  QFont font;
  font.setStyleStrategy(QFont::NoAntialias);
  qcp_mo_ss->xAxis->setTickLabelFont(font);
  qcp_mo_ss->yAxis->setTickLabelFont(font);
  qcp_mo_ss->legend->setFont(font);
  */

  // qcp_mo_ss->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
  // qcp_mo_ss->graph(0)->setAntialiasedFill(false);
  // qcp_mo_ss->graph(0)->setChannelFillGraph(qcp_mo_ss->graph(1));

  qcp_mo_ss->addGraph();  // red line
  qcp_mo_ss->graph(0)->setPen(QPen(Qt::red));

  qcp_mo_ss->addGraph();  // green line
  qcp_mo_ss->graph(1)->setPen(QPen(Qt::green));

  qcp_mo_ss->addGraph();  // upper blue line
  qcp_mo_ss->graph(2)->setPen(QPen(Qt::blue));

  qcp_mo_ss->addGraph();  // lower blue line
  qcp_mo_ss->graph(3)->setPen(QPen(Qt::blue));

  qcp_mo_ss->addGraph();  // red dot
  qcp_mo_ss->graph(4)->setPen(QPen(Qt::red));
  qcp_mo_ss->graph(4)->setLineStyle(QCPGraph::lsNone);
  qcp_mo_ss->graph(4)->setScatterStyle(QCPScatterStyle::ssDisc);

  qcp_mo_ss->addGraph();  // green dot
  qcp_mo_ss->graph(5)->setPen(QPen(Qt::green));
  qcp_mo_ss->graph(5)->setLineStyle(QCPGraph::lsNone);
  qcp_mo_ss->graph(5)->setScatterStyle(QCPScatterStyle::ssDisc);

  qcp_mo_ss->addGraph();  // blue upper dot
  qcp_mo_ss->graph(6)->setPen(QPen(Qt::blue));
  qcp_mo_ss->graph(6)->setLineStyle(QCPGraph::lsNone);
  qcp_mo_ss->graph(6)->setScatterStyle(QCPScatterStyle::ssDisc);

  qcp_mo_ss->addGraph();  // blue lower dot
  qcp_mo_ss->graph(7)->setPen(QPen(Qt::blue));
  qcp_mo_ss->graph(7)->setLineStyle(QCPGraph::lsNone);
  qcp_mo_ss->graph(7)->setScatterStyle(QCPScatterStyle::ssDisc);

  qcp_mo_ss->xAxis->setTickLabelType(QCPAxis::ltDateTime);
  qcp_mo_ss->xAxis->setDateTimeFormat("hh:mm:ss");
  qcp_mo_ss->xAxis->setAutoTickStep(false);
  qcp_mo_ss->xAxis->setTickStep(2);

  qcp_mo_ss->axisRect()->setupFullAxesBox();

  // qcp_mo_ss->yAxis->setRange(-100, 100);

  // make left and bottom axes transfer their ranges to right and top axes:
  connect(qcp_mo_ss->xAxis, SIGNAL(rangeChanged(QCPRange)), qcp_mo_ss->xAxis2,
          SLOT(setRange(QCPRange)));
  connect(qcp_mo_ss->yAxis, SIGNAL(rangeChanged(QCPRange)), qcp_mo_ss->yAxis2,
          SLOT(setRange(QCPRange)));

  qcp_co->addGraph();  // black line
  qcp_co->graph(0)->setPen(QPen(Qt::black));

  qcp_co->addGraph();  // black dot
  qcp_co->graph(1)->setPen(QPen(Qt::black));
  qcp_co->graph(1)->setLineStyle(QCPGraph::lsNone);
  qcp_co->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);

  qcp_co->xAxis->setTickLabelType(QCPAxis::ltDateTime);
  qcp_co->xAxis->setDateTimeFormat("hh:mm:ss");
  qcp_co->xAxis->setAutoTickStep(false);
  qcp_co->xAxis->setTickStep(2);

  qcp_co->axisRect()->setupFullAxesBox();

  // qcp_co->yAxis->setRange(-100, 100);

  // make left and bottom axes transfer their ranges to right and top axes:
  connect(qcp_co->xAxis, SIGNAL(rangeChanged(QCPRange)), qcp_co->xAxis2,
          SLOT(setRange(QCPRange)));
  connect(qcp_co->yAxis, SIGNAL(rangeChanged(QCPRange)), qcp_co->yAxis2,
          SLOT(setRange(QCPRange)));

  // setup a timer that repeatedly calls PIDGUI::realtimeDataSlot:
  connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
  dataTimer.start(0);  // Interval 0 means to refresh as fast as possible
}

void PIDGUI::select_board_index()
{
  board_index = (board_id - LEFT_HIP_ID) / LEFT_HIP_ID;
}

void PIDGUI::on_lhip_rb_clicked()
{
  board_id = LEFT_HIP_ID;
  select_board_index();
}
void PIDGUI::on_lknee_rb_clicked()
{
  board_id = LEFT_KNEE_ID;
  select_board_index();
}
void PIDGUI::on_lankle_rb_clicked()
{
  board_id = LEFT_ANKLE_ID;
  select_board_index();
}
void PIDGUI::on_rhip_rb_clicked()
{
  board_id = RIGHT_HIP_ID;
  select_board_index();
}
void PIDGUI::on_rknee_rb_clicked()
{
  board_id = RIGHT_KNEE_ID;
  select_board_index();
}
void PIDGUI::on_rankle_rb_clicked()
{
  board_id = RIGHT_ANKLE_ID;
  select_board_index();
}

void PIDGUI::realtimeDataSlot()
{
// calculate two new data points:
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  double key = 0;
#else
  double key = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0;
#endif
  static double lastPointKey = 0;
  if (key - lastPointKey > 0.02)  // at most add point every 20 ms
  {
    // Lines
    ui->qcp_mo_ss->graph(0)->addData(key, measured_output[board_index]);
    ui->qcp_mo_ss->graph(1)->addData(key, setpoint[board_index]);
    ui->qcp_mo_ss->graph(2)->addData(key,
                                     setpoint[board_index] + pid_tolerance);
    ui->qcp_mo_ss->graph(3)->addData(key,
                                     setpoint[board_index] - pid_tolerance);

    // Dots
    ui->qcp_mo_ss->graph(4)->clearData();
    ui->qcp_mo_ss->graph(4)->addData(key, measured_output[board_index]);
    ui->qcp_mo_ss->graph(5)->clearData();
    ui->qcp_mo_ss->graph(5)->addData(key, setpoint[board_index]);
    ui->qcp_mo_ss->graph(6)->clearData();
    ui->qcp_mo_ss->graph(6)->addData(key,
                                     setpoint[board_index] + pid_tolerance);
    ui->qcp_mo_ss->graph(7)->clearData();
    ui->qcp_mo_ss->graph(7)->addData(key,
                                     setpoint[board_index] - pid_tolerance);

    // remove data of lines that's outside visible range:
    ui->qcp_mo_ss->graph(0)->removeDataBefore(key - 4);
    ui->qcp_mo_ss->graph(1)->removeDataBefore(key - 4);
    ui->qcp_mo_ss->graph(2)->removeDataBefore(key - 4);
    ui->qcp_mo_ss->graph(3)->removeDataBefore(key - 4);

    // rescale value (vertical) axis to fit the current data:
    ui->qcp_mo_ss->graph(0)->rescaleValueAxis(true);
    ui->qcp_mo_ss->graph(1)->rescaleValueAxis(false);
    ui->qcp_mo_ss->graph(2)->rescaleValueAxis(true);
    ui->qcp_mo_ss->graph(3)->rescaleValueAxis(true);

    ui->qcp_co->graph(0)->addData(key, control_output[board_index]);
    ui->qcp_co->graph(1)->clearData();
    ui->qcp_co->graph(1)->addData(key, control_output[board_index]);
    ui->qcp_co->graph(0)->removeDataBefore(key - 4);
    ui->qcp_co->graph(0)->rescaleValueAxis(true);

    lastPointKey = key;
  }
  // make key axis range scroll with the data (at a constant range size of 8):
  ui->qcp_mo_ss->xAxis->setRange(key + 0.50, 4, Qt::AlignRight);
  // ui->qcp_mo_ss->yAxis->setRange(-100, 100);
  ui->qcp_co->xAxis->setRange(key + 0.50, 4, Qt::AlignRight);
  // ui->qcp_co->yAxis->setRange(-100, 100);

  ui->qcp_mo_ss->replot();
  ui->qcp_co->replot();

  // calculate frames per second:
  static double lastFpsKey;
  static int frameCount;
  ++frameCount;
  if (key - lastFpsKey > 2)  // average fps over 2 seconds
  {
    ui->statusBar->showMessage(
        QString("%1 FPS, Total Data points: %2")
            .arg(frameCount / (key - lastFpsKey), 0, 'f', 0)
            .arg(ui->qcp_mo_ss->graph(0)->data()->count() +
                 ui->qcp_mo_ss->graph(1)->data()->count() +
                 ui->qcp_mo_ss->graph(2)->data()->count() +
                 ui->qcp_mo_ss->graph(3)->data()->count() +
                 ui->qcp_mo_ss->graph(4)->data()->count() +
                 ui->qcp_mo_ss->graph(5)->data()->count() +
                 ui->qcp_mo_ss->graph(6)->data()->count() +
                 ui->qcp_mo_ss->graph(7)->data()->count() +
                 ui->qcp_co->graph(0)->data()->count() +
                 ui->qcp_co->graph(1)->data()->count()),
        0);
    lastFpsKey = key;
    frameCount = 0;
  }

  // cout << std::this_thread::get_id() << " realtimeDataSlot" << endl;
}

void PIDGUI::update_pid_gui(struct PIDContainer pid_container)
{
  for (int i = 5; i >= 0; i--)
  {
    setpoint[i] = pid_container.setpoint[i];
    measured_output[i] = pid_container.measured_output[i];
    control_output[i] = pid_container.control_output[i];
  }
  // cout << std::this_thread::get_id() << " PIDGUI::update_pid_gui(...)" <<
  // endl;
}

//#include "moc_pid_gui.cpp"
