/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  */

#include <iostream>
#include <string>

#include <ctime>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <unistd.h>

#include "constants.h"

#include "h2r/FSRSMessage.h"
#include "h2r/PatternsMessage.h"
#include "ros/ros.h"

#include "CANbusInterface.h"

#define FRAMEDIM 6

using namespace H2ros;
using namespace std;

class PatternsContainer {

public:
  std::vector<float> patterns[6];
  std::vector<float> *tx_patterns[6];
  can_frame can_frames[6];
  can_frame *tx_can_frames[6];
  can_frame tx_can_frames_block[6];

  int enabled_patterns_number;
  int setpoint_index;
  int setpoints_number;
  float normalization_factor;
  ros::Rate *rate;
  bool transmit;
  bool transmit_once;
  CANbusInterface cbi;

  PatternsContainer()
      : enabled_patterns_number(0), setpoint_index(0), setpoints_number(0),
        normalization_factor(1.0f), rate(new ros::Rate(1)), transmit(false),
        transmit_once(false), cbi("can0") {
    for (int i = 0; i < 6; i++) {
      can_frames[i].can_id = SETPOINT_MESSAGE_ID + i;
      can_frames[i].can_dlc = FRAMEDIM;
      can_frames[i].data[5] = (uint8_t)(0);
    }
  }

  PatternsContainer(const PatternsContainer &pc) = delete;

  ~PatternsContainer() {
    if (rate != 0)
      delete rate;

    rate = 0;
  }

  PatternsContainer &operator=(const PatternsContainer &pc) = delete;

  void receive_patterns(const h2r::PatternsMessage::ConstPtr &msg) {
    enabled_patterns_number = 0;
    setpoint_index = 0;
    setpoints_number = 0;
    transmit = true;
    transmit_once = false;

    const std::vector<float> *msg_patterns[6] = {
        &(msg->pattern_0), &(msg->pattern_1), &(msg->pattern_2),
        &(msg->pattern_3), &(msg->pattern_4), &(msg->pattern_5)};

    for (int i = 0; i < 6; i++) {
      if (msg_patterns[i]->size()) {
        patterns[i] = *msg_patterns[i];
        tx_patterns[enabled_patterns_number] = &patterns[i];
        can_frames[i].data[4] = (uint8_t)(msg->controller_id);
        tx_can_frames[enabled_patterns_number] = &can_frames[i];
        enabled_patterns_number++;
      }
    }

    setpoints_number = tx_patterns[0]->size();

    // The only pattern with one setpoint is the one wich is used to disable the
    // pid in each board.
    if (setpoints_number == 1)
      transmit_once = true;

    switch (msg->controller_id) {
    case POS_PID_CONTROLLER_ID: {
      normalization_factor = max_angle_deg;
      break;
    }
    case LVL_PID_CONTROLLER_ID: {
      normalization_factor = max_lvl_deg;
      break;
    }
    case TOR_PID_CONTROLLER_ID: {
      normalization_factor = max_tor_Nm;
      break;
    }
    case TCL_PID_CONTROLLER_ID: {
      normalization_factor = max_angle_deg;
      break;
    }
    default: { normalization_factor = 1.0f; }
    }

    delete rate;
    rate = new ros::Rate(1000.00f / msg->sample_period);
    // rate->reset();
  }

  void begin_patterns(const h2r::FSRSMessage::ConstPtr &fsrs_msg) {
    setpoint_index = 0;
  }
};

// void checkArguments(const int argc, char** argv);

// void signalHandler(int signum)
//{
//    if(rate != NULL)
//        delete rate;
//    rate = NULL;
//}

int main(int argc, char **argv) {

  ros::init(argc, argv, "patterns_tx_node_to_driver");
  ros::NodeHandle n;
  PatternsContainer pc;
  ros::Subscriber pattern_sub = n.subscribe(
      "patterns_topic", 1, &PatternsContainer::receive_patterns, &pc);
  ros::Subscriber heel_strike_sub = n.subscribe(
      "fsrs_begin_patters_topic", 1, &PatternsContainer::begin_patterns, &pc);

  // signal(SIGINT, signalHandler); // Capture Ctrl+C
  // signal(SIGTERM, signalHandler);

  int pattern_index;
  float setpoint;

  while (ros::ok()) {
    if (pc.transmit) {
      for (pattern_index = 0; pattern_index < pc.enabled_patterns_number;
           pattern_index++) {
        setpoint = (pc.tx_patterns[pattern_index]->at(pc.setpoint_index)) /
                   pc.normalization_factor;
        memcpy(&(pc.tx_can_frames[pattern_index]->data), &setpoint,
               sizeof(float));
        // memcpy(pc.tx_can_frames_block + pattern_index,
        // pc.tx_can_frames[pattern_index], sizeof(can_frame));
        pc.cbi.canWrite(*(pc.tx_can_frames[pattern_index]));
        usleep(500);
      }
      // pc.cbi.canWrite(pc.tx_can_frames_block, pc.enabled_patterns_number *
      // sizeof(can_frame));
      pc.setpoint_index = ++pc.setpoint_index % pc.setpoints_number;
    }
    if (pc.transmit_once)
      pc.transmit = false;
    ros::spinOnce();
    pc.rate->sleep();
  }

  exit(EXIT_SUCCESS);
}
