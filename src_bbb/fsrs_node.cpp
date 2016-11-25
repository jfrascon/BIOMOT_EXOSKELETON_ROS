/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  */

#include <iostream>
#include <thread>

#include "constants.h"
#include "h2r/FSRSMessage.h"
#include "h2r/FeetFSRSThresholdsMessage.h"
#include "h2r/SegmentationMessage.h"

#include "ros/ros.h"

#include "ReadAIN.h"
#include "interruptFunctions.h"

#include "CANbusInterface.h"
#include <cstring>

#include <climits>

using namespace std;
using namespace ros;
using namespace h2r;
using namespace H2ros;

class FSRSThresholds {
public:
  explicit FSRSThresholds() {
    this->left_toe_threshold = INT_MAX;
    this->left_heel_threshold = INT_MAX;
    this->right_toe_threshold = INT_MAX;
    this->right_heel_threshold = INT_MAX;
  }

  void set_thresholds(const FeetFSRSThresholdsMessage::ConstPtr &msg) {
    this->left_toe_threshold = msg->left_toe_threshold;
    this->left_heel_threshold = msg->left_heel_threshold;
    this->right_toe_threshold = msg->right_toe_threshold;
    this->right_heel_threshold = msg->right_heel_threshold;
  }

  int get_left_toe_threshold() { return this->left_toe_threshold; }
  int get_left_heel_threshold() { return this->left_heel_threshold; }
  int get_right_toe_threshold() { return this->right_toe_threshold; }
  int get_right_heel_threshold() { return this->right_heel_threshold; }

private:
  int left_toe_threshold;
  int left_heel_threshold;
  int right_toe_threshold;
  int right_heel_threshold;
};

int main(int argc, char **argv) {

  init(argc, argv, "fsrs_node");

  NodeHandle n;

  // Publisher fsrs_pub = n.advertise<FSRSMessage>("fsrs_topic", 1);

  FSRSThresholds fsrs_thresholds;
  Subscriber fsrs_thresholds_sub =
      n.subscribe("fsrs_thesholds_topic", 1, &FSRSThresholds::set_thresholds,
                  &fsrs_thresholds);

  int left_toe_value = 0;
  int left_heel_value = 0;
  int right_toe_value = 0;
  int right_heel_value = 0;

  int left_heel_strike = 0;
  int left_flat_foot = 0;
  int left_toe_strike = 0;
  int left_toe_off = 0;

  int right_heel_strike = 0;
  int right_flat_foot = 0;
  int right_toe_strike = 0;
  int right_toe_off = 0;

  Publisher segmentation_pub =
      n.advertise<SegmentationMessage>("segmentation_topic", 1);

  SegmentationMessage segmentation_msg;
  segmentation_msg.left_segmentation = -1;
  segmentation_msg.right_segmentation = -1;

  int left_segmentation = -1;
  int right_segmentation = -1;
  bool send_segmentation_msg = false;

  // chatterCallback);
  // ros::Publisher fsrs_begin_pattern_pub =
  // n.advertise<h2r::FSRSMessage>("fsrs_begin_patters_topic", 1);

  Rate rate(100);
  // FSRSMessage fsrs_message;
  // fsrs_message.fsrs.assign(4, 0);
  // fsrs_message.begin_pattern.assign(2, false);

  // int fsr;
  // bool left_foot_on_air_flag = true;
  // bool left_foot_on_ground_flag;

  // bool right_foot_on_air_flag = true;
  // bool right_foot_on_ground_flag;

  struct StartTimeContainer start_time_container;
  start_time_container.writing_or_wrinting_available = false;
  start_time_container.number_readers = 0;
  gettimeofday(&start_time_container.start_time, NULL);
  timeval current_time;

  CANbusInterface canbusInterface_("can0");

  can_frame fsrs_can_packet;
  fsrs_can_packet.can_id = 0x211;
  fsrs_can_packet.can_dlc = 6;
  long long int fsrs_data = 0;
  long long int temp = 0;

  can_frame segmentation_can_packet;
  segmentation_can_packet.can_id = 0x212;
  segmentation_can_packet.can_dlc = 6;
  int feet_segmentation = 0;

  // Thread waiting for a trigger pulse on gpio7 port.
  thread trigger_waiter(resettingId, &(start_time_container));

  while (ok()) {
    get_current_time(&start_time_container, &current_time);

    segmentation_msg.header.stamp.sec = current_time.tv_sec;
    segmentation_msg.header.stamp.nsec = 1000 * current_time.tv_usec;

    left_heel_value = Read_AnalogData(0); // Left heel
    left_toe_value = Read_AnalogData(1);
    right_heel_value = Read_AnalogData(2); // Right heel
    right_toe_value = Read_AnalogData(3);

    if (left_heel_value > fsrs_thresholds.get_left_heel_threshold() &&
        left_toe_value < fsrs_thresholds.get_left_toe_threshold())
      left_segmentation = 1;
    else if (left_heel_value > fsrs_thresholds.get_left_heel_threshold() &&
             left_toe_value > fsrs_thresholds.get_left_toe_threshold())
      left_segmentation = 3;
    else if (left_heel_value < fsrs_thresholds.get_left_heel_threshold() &&
             left_toe_value > fsrs_thresholds.get_left_toe_threshold())
      left_segmentation = 4;
    else if (left_heel_value < fsrs_thresholds.get_left_heel_threshold() &&
             left_toe_value < fsrs_thresholds.get_left_toe_threshold())
      left_segmentation = 2;

    if (right_heel_value > fsrs_thresholds.get_right_heel_threshold() &&
        right_toe_value < fsrs_thresholds.get_right_toe_threshold())
      segmentation_msg.right_segmentation = 1;
    else if (right_heel_value > fsrs_thresholds.get_right_heel_threshold() &&
             right_toe_value > fsrs_thresholds.get_right_toe_threshold())
      segmentation_msg.right_segmentation = 3;
    else if (right_heel_value < fsrs_thresholds.get_right_heel_threshold() &&
             right_toe_value > fsrs_thresholds.get_right_toe_threshold())
      segmentation_msg.right_segmentation = 4;
    else if (right_heel_value < fsrs_thresholds.get_right_heel_threshold() &&
             right_toe_value < fsrs_thresholds.get_right_toe_threshold())
      segmentation_msg.right_segmentation = 2;

    send_segmentation_msg = false;

    if (segmentation_msg.left_segmentation != left_segmentation) {
      segmentation_msg.left_segmentation = left_segmentation;
      send_segmentation_msg = true;
    }

    if (segmentation_msg.right_segmentation != right_segmentation) {
      segmentation_msg.right_segmentation = right_segmentation;
      send_segmentation_msg = true;
    }

    fsrs_data = (left_heel_value & 0x00000FFF);
    fsrs_data |= ((left_toe_value & 0x00000FFF) << 12);
    temp = right_heel_value & 0x00000FFF;
    temp <<= 24;
    fsrs_data |= temp;
    temp = right_toe_value & 0x00000FFF;
    temp <<= 36;
    fsrs_data |= temp;
    memcpy(fsrs_can_packet.data, &fsrs_data, 6 * sizeof(char));

    feet_segmentation = left_segmentation;
    feet_segmentation <<= 8;
    feet_segmentation |= right_segmentation;
    memcpy(segmentation_can_packet.data, &feet_segmentation, 2 * sizeof(char));

    if (send_segmentation_msg) {
      canbusInterface_.canWrite(segmentation_can_packet);
      segmentation_pub.publish(segmentation_msg);
    }

    canbusInterface_.canWrite(fsrs_can_packet);

    // left_foot_on_ground_flag = fsrs_message.fsrs[0] > heel_strike_threshold;
    // right_foot_on_ground_flag = fsrs_message.fsrs[1] > heel_strike_threshold;

    // El talón izquierdo pasa de estar levantado a hacer un heel strike
    // Es el momento de empezar de nuevo el patrón de marcha del segmento
    // izquierdo del exo.
    // if(left_foot_on_air_flag && left_foot_on_ground_flag)
    //{
    //    fsrs_message.begin_pattern[0] = true;
    //    fsrs_begin_pattern_pub.publish(fsrs_message);

    //    left_foot_on_air_flag = false;
    //    fsrs_message.begin_pattern[0] = false;
    //}
    // El talón derecho pasa de estar levantado a hacer un heel strike
    // Es el momento de empezar de nuevo el patrón de marcha del segmento
    // derecho del exo.
    // if(right_foot_on_air_flag && right_foot_on_ground_flag)
    //{
    //    fsrs_message.begin_pattern[1] = true;
    //    fsrs_begin_pattern_pub.publish(fsrs_message);

    //    right_foot_on_air_flag = false;
    //    fsrs_message.begin_pattern[1] = false;
    //}

    // Comprobar si el talón izquierdo/derecho se ha levantado del suelo.
    // if(!left_foot_on_air_flag)
    //    left_foot_on_air_flag = fsrs_message.fsrs[0] < heel_up_threshold;

    // if(!right_foot_on_air_flag)
    //    right_foot_on_air_flag = fsrs_message.fsrs[1] < heel_up_threshold;

    spinOnce();
    rate.sleep();
  }

  trigger_waiter.join();
  exit(EXIT_SUCCESS);
}
