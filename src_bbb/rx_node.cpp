/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  */

#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdlib.h>
#include <string>
#include <sys/time.h>

using namespace std;

#include "h2r/JointAnglesMessage.h"
#include "h2r/PIDMessage.h"
#include "h2r/SegmentationMessage.h"
#include "ros/console.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

#include "h2r/FSRSMessage.h"
#include "h2r/ForcesMessage.h"
#include "h2r/ParallelSpringMessage.h"
#include "h2r/SensorsMessage.h"
//#include "h2r/torques_message.h"

#include "CANbusReader.h"
#include "DataFromCANbus.h"
#include "interruptFunctions.h"

#include "constants.h"

//#include "CANbusInterface.h"
//#include <linux/can.h>

//#define DEBUG 1
//#define DEBUG_HEX 1

using namespace H2ros;

void printUsage();
void printAuthors();
void printHeader();
void checkArguments(const int argc, char **argv, int &freq_reducing_factor);

void proccess_can_packets(DataFromCANbus *dataFromCANbus,
                          struct StartTimeContainer *start_time_container);
void proccess_joint_state_can_packets(
    DataFromCANbus *dataFromCANbus,
    struct StartTimeContainer *start_time_container);
void proccess_pid_can_packets(DataFromCANbus *dataFromCANbus,
                              struct StartTimeContainer *start_time_container);
void proccess_sensor_can_packets(
    DataFromCANbus *dataFromCANbus,
    struct StartTimeContainer *start_time_container);

int main(int argc, char **argv) {

  int freq_reducing_factor = 1;

  // bool reduce_freq = false;
  // int angles_torque_msgs_counter = 0;
  // int pid_msgs_counter = 0;
  // int sensors_msgs_counter = 0;

  printHeader();
  printAuthors();
  checkArguments(argc, argv, freq_reducing_factor);

  timespec current_time;
  StartTimeContainer start_time_container;
  start_time_container.writing_or_wrinting_available = false;
  start_time_container.number_readers = 0;
  gettimeofday(&start_time_container.start_time, NULL);

  std::thread trigger_waiter(resettingId, &start_time_container);

  // Object that holds a queue of CAN messages received from CAN bus.
  DataFromCANbus dataFromCANbus("can0");
  // Object that reads CAN messages from CAN bus and store them in
  // dataFromCANbus's queue
  CANbusReader can_bus_reader(dataFromCANbus);
  // CAN messages reader thread. (Information producer). There is only one
  // information producer
  // in the program.
  std::thread can_bus_reader_thread(std::ref(can_bus_reader));

  ros::init(argc, argv, "rx_node_from_driver");
  proccess_can_packets(&dataFromCANbus, &start_time_container);

  trigger_waiter.join();
  can_bus_reader_thread.join();

  exit(EXIT_SUCCESS);
}

void proccess_can_packets(DataFromCANbus *dataFromCANbus,
                          struct StartTimeContainer *start_time_container) {
  // The implemented queue allows multiple threads to access to received CAN
  // data. (Information consumer)
  // Read notes in ../lib/Concurrency/Queue.h
  dataFromCANbus->queue.subscribe();

  DataFromCANbus::FrameType newData;

  int fix_link_angle_digital = 0;
  int alpha_angle_digital = 0;
  int lever_arm_angle_digital = 0;
  int torque_Nm_digital = 0;
  int velocity_digital = 0;
  int setpoint_digital = 0;
  int measured_output_digital = 0;
  int control_output_digital = 0;

  int error;
  int temperature_digital;
  int current_digital;
  int internal_load_cell_force_digital;
  int16_t temp;

  ros::NodeHandle n;

  // The last argument (1) means that the program is interested in transmitting
  // the last available message on that topic only.
  ros::Publisher joint_state_pub =
      n.advertise<sensor_msgs::JointState>("joint_state", 1);

  ros::Publisher joint_angles_pub =
      n.advertise<h2r::JointAnglesMessage>("joint_angles_topic", 1);

  ros::Publisher pid_pub = n.advertise<h2r::PIDMessage>("pid_topic", 1);

  ros::Publisher sensors_pub =
      n.advertise<h2r::SensorsMessage>("sensors_topic", 1);

  ros::Publisher forces_pub =
      n.advertise<h2r::ForcesMessage>("forces_topic", 1);

  ros::Publisher parallel_spring_pub =
      n.advertise<h2r::ParallelSpringMessage>("parallel_spring_topic", 1);

  ros::Publisher segmentation_pub =
      n.advertise<h2r::SegmentationMessage>("segmentation_topic", 1);

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name = {"left_hip",  "left_knee",  "left_ankle",
                          "right_hip", "right_knee", "right_ankle"};
  joint_state_msg.position.assign(6, 0);
  joint_state_msg.velocity.clear();
  joint_state_msg.effort.assign(6, 0);

  h2r::JointAnglesMessage joint_angles_msg;
  joint_angles_msg.fix_link_angle.assign(6, 0);
  joint_angles_msg.lever_arm_angle.assign(6, 0);
  joint_angles_msg.alpha_angle.assign(6, 0);

  h2r::PIDMessage pid_msg;
  pid_msg.setpoint.assign(6, 0);
  pid_msg.control_output.assign(6, 0);
  pid_msg.measured_output.assign(6, 0);

  h2r::SensorsMessage sensors_msg;
  sensors_msg.internal_load_cell_force_digital.assign(6, 0);
  sensors_msg.external_load_cell_force_digital.assign(6, 0);
  sensors_msg.error.assign(6, 0);

  h2r::ForcesMessage forces_msg;
  forces_msg.internal_load_cell_force.assign(6, 0);
  forces_msg.external_load_cell_force.assign(6, 0);

  h2r::ParallelSpringMessage parallel_spring_msg;
  parallel_spring_msg.right_knee_fix_link_angle = 0.0f;
  parallel_spring_msg.left_knee_fix_link_angle = 0.0f;
  parallel_spring_msg.right_number_revolutions = 0;
  parallel_spring_msg.left_number_revolutions = 0;
  parallel_spring_msg.right_limit = 0;
  parallel_spring_msg.left_limit = 0;

  //  h2r::SegmentationMessage segmentation_msg;
  //  segmentation_msg.left_segmentation = 0;
  //  segmentation_msg.right_segmentation = 0;
  //  int left_segmentation = -1;
  //  int last_left_segmentation = -1;
  //  int right_segmentation = -1;
  //  int last_right_segmentation = -1;

  // int last_left_segmentation_0_1 = -1;
  // int last_left_segmentation_0_2 = -1;
  // int last_right_segmentation_0_1 = -1;
  // int last_right_segmentation_0_2 = -1;

  bool continue_forever = true;
  timeval current_time;
  int can_id = 0;
  int board_id = 0;
  int board_index = 0;
  int num = RIGHT_ANKLE_ID - RIGHT_HIP_ID;
  // ankle_laa_aa_id - hip_laa_aa_id ó ankle_mo_ss_id - hip_mo_ss_id
  int den = ankle_torque_fla_id - hip_torque_fla_id;

  unsigned int last_can_id = 0;

  int right_knee_fix_link_angle = 0;
  int right_number_rev = 0;
  int right_limit = 0;

  int left_knee_fix_link_angle = 0;
  int left_number_rev = 0;
  int left_limit = 0;

  while (continue_forever) {
    // All consumer threads synchronize here!
    newData = dataFromCANbus->queue.pop();
    // All consumer threads and timer thread synchronize here!
    get_current_time(start_time_container, &current_time);
    can_id = static_cast<unsigned int>(newData.data.can_id);

    if (can_id < last_can_id) {
      joint_state_pub.publish(joint_state_msg);
      parallel_spring_pub.publish(parallel_spring_msg);
      joint_angles_pub.publish(joint_angles_msg);
      pid_pub.publish(pid_msg);
      // sensors_pub.publish();
      // forces_pub.publish();

      joint_state_msg.header.stamp.sec = current_time.tv_sec;
      joint_state_msg.header.stamp.nsec = 1000 * current_time.tv_usec;

      parallel_spring_msg.header.stamp.sec = current_time.tv_sec;
      parallel_spring_msg.header.stamp.nsec = joint_state_msg.header.stamp.nsec;

      joint_angles_msg.header.stamp.sec = current_time.tv_sec;
      joint_angles_msg.header.stamp.nsec = joint_state_msg.header.stamp.nsec;

      pid_msg.header.stamp.sec = current_time.tv_sec;
      pid_msg.header.stamp.nsec = joint_state_msg.header.stamp.nsec;
    }

    if (can_id >= lower_can_id && can_id <= upper_can_id)
      last_can_id = can_id;

#ifdef DEBUG_HEX
    cout.precision(3);
    cout.setf(ios::fixed, ios::floatfield);
    cout << "Can id: " << can_id << endl;
#endif

    switch (can_id) {
    // System setpoint and measured output message for all joints.
    case hip_mo_ss_id:
    case knee_mo_ss_id:
    case ankle_mo_ss_id: {
      board_id = static_cast<unsigned int>(
          (num * (static_cast<float>((can_id - knee_mo_ss_id)) / den)) +
          RIGHT_KNEE_ID);
      board_index = (board_id - LEFT_HIP_ID) /
                    LEFT_HIP_ID; // integer division, no castind needed

#ifdef DEBUG_HEX
      cout << showbase      // show the 0x prefix
           << internal      // fill between the prefix and the number
           << setfill('0'); // fill with 0s
      cout << hex << uppercase << setw(4)
           << static_cast<int>(newData.data.data[7]) << " " << hex << uppercase
           << setw(4) << static_cast<int>(newData.data.data[6]) << " " << hex
           << uppercase << setw(4) << static_cast<int>(newData.data.data[5])
           << " " << hex << uppercase << setw(4)
           << static_cast<int>(newData.data.data[4]) << " " << hex << uppercase
           << setw(4) << static_cast<int>(newData.data.data[3]) << " " << hex
           << uppercase << setw(4) << static_cast<int>(newData.data.data[2])
           << " " << hex << uppercase << setw(4)
           << static_cast<int>(newData.data.data[1]) << " " << hex << uppercase
           << setw(4) << static_cast<int>(newData.data.data[0]) << nouppercase
           << dec << endl;
#endif

      for (int i = 0; i < 8; i += 4, board_index -= 3) {
        // pid_msgs_counter++;
        // if(pid_msgs_counter != freq_reducing_factor)
        //    continue;
        // pid_msgs_counter = 0;

        setpoint_digital = static_cast<int>(newData.data.data[i + 1]);
        setpoint_digital <<= 8;
        setpoint_digital += static_cast<int>(newData.data.data[i]);

        // Setpoint and measured output in Q1.15 format (i.e signed integer in
        // 16 bit).
        // With 16 bits ==> 65536 values (2^16) from -32768 (-2^15) to +32767
        // (2^15 - 1)
        // A 16-bit value is stored into a 32-bit variable. So negative
        // numbers turn into positive due to the upper 16 zeros of the container
        // variable.
        // In order to mantein the sign we have to check the bit 15 and fill
        // with 0xF if neccesary.
        if (setpoint_digital >> 15)
          setpoint_digital |= 0xFFFF0000;

        measured_output_digital = static_cast<int>(newData.data.data[i + 3]);
        measured_output_digital <<= 8;
        measured_output_digital += static_cast<int>(newData.data.data[i + 2]);

        if (measured_output_digital >> 15)
          measured_output_digital |= 0xFFFF0000;

        // -32768 (-2^15,     0x8000) is mapped to -100.00f (100.0f/32768 *
        // -32768)
        // +32767 (+2^15 - 1, 0x7FFF) is mapped to +99.997f (100.0f/32768 *
        // +32767)
        pid_msg.setpoint[board_index] = pid_dig2percent * setpoint_digital;
        pid_msg.measured_output[board_index] =
            pid_dig2percent * measured_output_digital;

#ifdef DEBUG
        cout.precision(3);
        cout.setf(ios::fixed, ios::floatfield);
        cout << "Board index: " << board_index
             << "  SS: " << pid_msg.setpoint[board_index]
             << " MO: " << pid_msg.measured_output[board_index] << endl;
#endif
      }
      break;
    }
    // Alpha angle and lever arm angle message for all joints.
    case hip_laa_aa_id:
    case knee_laa_aa_id:
    case ankle_laa_aa_id: {

      board_id = static_cast<unsigned int>(
          (num * (static_cast<float>((can_id - knee_laa_aa_id)) / den)) +
          RIGHT_KNEE_ID);
      board_index = (board_id - LEFT_HIP_ID) /
                    LEFT_HIP_ID; // integer division, no castind needed

#ifdef DEBUG_HEX

      cout << showbase      // show the 0x prefix
           << internal      // fill between the prefix and the number
           << setfill('0'); // fill with 0s
      cout << hex << uppercase << setw(4)
           << static_cast<int>(newData.data.data[7]) << " " << hex << uppercase
           << setw(4) << static_cast<int>(newData.data.data[6]) << " " << hex
           << uppercase << setw(4) << static_cast<int>(newData.data.data[5])
           << " " << hex << uppercase << setw(4)
           << static_cast<int>(newData.data.data[4]) << " " << hex << uppercase
           << setw(4) << static_cast<int>(newData.data.data[3]) << " " << hex
           << uppercase << setw(4) << static_cast<int>(newData.data.data[2])
           << " " << hex << uppercase << setw(4)
           << static_cast<int>(newData.data.data[1]) << " " << hex << uppercase
           << setw(4) << static_cast<int>(newData.data.data[0]) << nouppercase
           << dec << endl;
#endif

      for (int i = 0; i < 8; i += 4, board_index -= 3) {
        alpha_angle_digital = static_cast<int>(newData.data.data[i + 1]);
        alpha_angle_digital <<= 8;
        alpha_angle_digital += static_cast<int>(newData.data.data[i]);

        // alpha angle is coded in 16 bit.
        // With 16 bits ==> 65536 values (2^16) from -32768 (⁻2^15) to 32767
        // (2^15-1).
        // Permitted values for an alpha angle: (-8191, 8192)
        // Sign is coded in bit 15. If bit 15 is 1, there is negative value,
        // so it's neccesary to add 0xF's.
        if (alpha_angle_digital >> 15)
          alpha_angle_digital |= 0XFFFF0000;

        lever_arm_angle_digital = static_cast<int>(newData.data.data[i + 3]);
        lever_arm_angle_digital <<= 8;
        lever_arm_angle_digital += static_cast<int>(newData.data.data[i + 2]);

        // lever arm angle is coded in 16 bit.
        // Permitted values for a lever arm angle: (-3999, 4000)
        // Sign is coded in bit 15. If bit 15 is 1, there is negative value,
        // so it's neccesary to add 0xF's.
        if (lever_arm_angle_digital >> 15)
          lever_arm_angle_digital |= 0XFFFF0000;

        // -8191 is mapped to -pi (well, almost -pi) (2*pi/16384 * -8191)
        // +8192 is mapped to +pi                    (2*pi/16384 * +8192)
        joint_angles_msg.alpha_angle[board_index] =
            alpha_angle_dig2rad * alpha_angle_digital;

        // -3999 is mapped to -pi (well, almost -pi) (2*pi/8000 * -3999)
        // +4000 is mapped to +pi                    (2*pi/8000 * +4000)
        joint_angles_msg.lever_arm_angle[board_index] =
            lever_arm_angle_dig2rad * lever_arm_angle_digital;

#ifdef DEBUG
        cout.precision(3);
        cout.setf(ios::fixed, ios::floatfield);
        cout << "Board index: " << board_index
             << "  AA: " << rad2grad * joint_angles_msg.alpha_angle[board_index]
             << " deg  LAA: "
             << rad2grad * joint_angles_msg.lever_arm_angle[board_index]
             << " deg" << endl;
#endif
      }
      // std::this_thread::get_id()
      break;
    }
    // Fix link angle and torque for all joints.
    case hip_torque_fla_id:
    case knee_torque_fla_id:
    case ankle_torque_fla_id: {
      board_id = static_cast<unsigned int>(
          (num * (static_cast<float>((can_id - knee_torque_fla_id)) / den)) +
          RIGHT_KNEE_ID);
      board_index = (board_id - LEFT_HIP_ID) /
                    LEFT_HIP_ID; // integer division, no castind needed

#ifdef DEBUG_HEX
      cout << showbase      // show the 0x prefix
           << internal      // fill between the prefix and the number
           << setfill('0'); // fill with 0s
      cout << hex << uppercase << setw(4)
           << static_cast<int>(newData.data.data[7]) << " " << hex << uppercase
           << setw(4) << static_cast<int>(newData.data.data[6]) << " " << hex
           << uppercase << setw(4) << static_cast<int>(newData.data.data[5])
           << " " << hex << uppercase << setw(4)
           << static_cast<int>(newData.data.data[4]) << " " << hex << uppercase
           << setw(4) << static_cast<int>(newData.data.data[3]) << " " << hex
           << uppercase << setw(4) << static_cast<int>(newData.data.data[2])
           << " " << hex << uppercase << setw(4)
           << static_cast<int>(newData.data.data[1]) << " " << hex << uppercase
           << setw(4) << static_cast<int>(newData.data.data[0]) << nouppercase
           << dec << endl;
#endif

      for (int i = 0; i < 8; i += 4, board_index -= 3) {
        fix_link_angle_digital = static_cast<int>(newData.data.data[i + 1]);
        fix_link_angle_digital <<= 8;
        fix_link_angle_digital += static_cast<int>(newData.data.data[i]);

        // fix link angle is coded in 16 bit.
        // With 16 bits ==> 65536 values (2^16) from -32768 (⁻2^15) to 32767
        // (2^15-1).
        // Permitted values for a fix link angle: (-3999, 4000)
        // Sign is coded in bit 15. If bit 15 is 1, there is negative value,
        // so
        // it's neccesary to add 0xF's.
        if (fix_link_angle_digital >> 15)
          fix_link_angle_digital |= 0XFFFF0000;

        // -3999 is mapped to -pi (well, almost -pi) (2*pi/8000 * -3999)
        // +4000 is mapped to +pi                    (2*pi/8000 * +4000)
        joint_state_msg.position[board_index] =
            fix_link_angle_dig2rad * fix_link_angle_digital;
        joint_angles_msg.fix_link_angle[board_index] =
            joint_state_msg.position[board_index];

        torque_Nm_digital = static_cast<int>(newData.data.data[i + 3]);
        torque_Nm_digital <<= 8;
        torque_Nm_digital += static_cast<int>(newData.data.data[i + 2]);

        // Torque due to alpha angle is coded in Q1.15 format (i.e signed
        // integer in 16 bit).
        // With 16 bits ==> 65536 values (2^16) from -32768 (-2^15) to 32767
        // (2^15 - 1).
        // Permitted values for a torque: (-32768, 32767)
        // Sign is coded in bit 15. If bit 15 is 1, there is negative value,
        // so it's neccesary to add 0xF's.
        if (torque_Nm_digital >> 15)
          torque_Nm_digital |= 0XFFFF0000;

        // -32768 (-2^15,     0x8000) is mapped to -MAX_TORQUE
        // (MAX_TORQUE/32768 * -32768)
        // +32767 (+2^15 - 1, 0x7FFF) is mapped to +MAX_TORQUE
        // (MAX_TORQUE/32768 * +32767)
        joint_state_msg.effort[board_index] = torque_dig2Nm * torque_Nm_digital;

#ifdef DEBUG
        cout.precision(3);
        cout.setf(ios::fixed, ios::floatfield);
        cout << "Board index: " << board_index
             << "  FLA: " << rad2grad * joint_state_msg.position[board_index]
             << " deg  Torque: " << joint_state_msg.effort[board_index] << " Nm"
             << endl;
#endif
      }
      // std::this_thread::get_id()
      break;
    }
    case co_id: {
      board_index = (RIGHT_ANKLE_ID - LEFT_HIP_ID) / LEFT_HIP_ID;
//        pid_msg.header.stamp.sec = current_time.tv_sec;
//      pid_msg.header.stamp.nsec = 1000 * current_time.tv_usec;

#ifdef DEBUG_HEX
      cout << showbase      // show the 0x prefix
           << internal      // fill between the prefix and the number
           << setfill('0'); // fill with 0s
      cout << hex << uppercase << setw(4)
           << static_cast<int>(newData.data.data[7]) << " " << hex << uppercase
           << setw(4) << static_cast<int>(newData.data.data[6]) << " " << hex
           << uppercase << setw(4) << static_cast<int>(newData.data.data[5])
           << " " << hex << uppercase << setw(4)
           << static_cast<int>(newData.data.data[4]) << " " << hex << uppercase
           << setw(4) << static_cast<int>(newData.data.data[3]) << " " << hex
           << uppercase << setw(4) << static_cast<int>(newData.data.data[2])
           << " " << hex << uppercase << setw(4)
           << static_cast<int>(newData.data.data[1]) << " " << hex << uppercase
           << setw(4) << static_cast<int>(newData.data.data[0]) << nouppercase
           << dec << endl;
#endif

      for (int i = 0; i < 8; i += 4, board_index -= 1) {
        // pid_msgs_counter++;
        // if(pid_msgs_counter != freq_reducing_factor)
        //    continue;
        // pid_msgs_counter = 0;

        control_output_digital = static_cast<int>(newData.data.data[i + 1]);
        control_output_digital <<= 8;
        control_output_digital += static_cast<int>(newData.data.data[i]);

        // Control output in Q1.15 format (i.e signed integer in 16 bit).
        // With 16 bits ==> 65536 values (2^16) from -32768 (-2^15) to +32767
        // (2^15 - 1)
        // Permitted values for a torque: (-32768, 32767)
        // Sign is coded in bit 15. If bit 15 is 1, there is negative value,
        // so it's neccesary to add 0xF's.
        if (control_output_digital >> 15)
          control_output_digital |= 0xFFFF0000;

        // -32768 (-2^15,     0x8000) is mapped to -100.00f (100.0f/32768 *
        // -32768)
        // +32767 (+2^15 - 1, 0x7FFF) is mapped to +99.997f (100.0f/32768 *
        // +32767)
        pid_msg.control_output[board_index] =
            pid_dig2percent * control_output_digital;

#ifdef DEBUG
        cout.precision(3);
        cout.setf(ios::fixed, ios::floatfield);
        cout << "Board index: " << board_index
             << "  CO:  " << pid_msg.control_output[board_index] << endl;
#endif

        control_output_digital = static_cast<int>(newData.data.data[i + 3]);
        control_output_digital <<= 8;
        control_output_digital += static_cast<int>(newData.data.data[i + 2]);

        if (control_output_digital >> 15)
          control_output_digital |= 0xFFFF0000;

        pid_msg.control_output[board_index - 3] =
            pid_dig2percent * control_output_digital;

#ifdef DEBUG
        cout.precision(3);
        cout.setf(ios::fixed, ios::floatfield);
        cout << "Board index: " << board_index - 3
             << "  CO: " << pid_msg.control_output[board_index - 3] << endl;
#endif
      }
      break;
    }
    case knee_parallel_spring: {

      right_knee_fix_link_angle = 0;
      right_knee_fix_link_angle = static_cast<int>(newData.data.data[1]);
      right_knee_fix_link_angle <<= 8;
      right_knee_fix_link_angle |= static_cast<int>(newData.data.data[0]);

      if (right_knee_fix_link_angle >> 15)
        right_knee_fix_link_angle |= 0XFFFF0000;

      parallel_spring_msg.right_knee_fix_link_angle =
          fix_link_angle_dig2rad * right_knee_fix_link_angle;

      left_knee_fix_link_angle = 0;
      left_knee_fix_link_angle = static_cast<int>(newData.data.data[3]);
      left_knee_fix_link_angle <<= 8;
      left_knee_fix_link_angle |= static_cast<int>(newData.data.data[2]);

      if (left_knee_fix_link_angle >> 15)
        left_knee_fix_link_angle |= 0XFFFF0000;

      parallel_spring_msg.left_knee_fix_link_angle =
          fix_link_angle_dig2rad * left_knee_fix_link_angle;

      right_number_rev = 0;
      right_number_rev = static_cast<int>(newData.data.data[5]);
      right_number_rev &= 0x07;
      right_number_rev <<= 8;
      right_number_rev |= static_cast<int>(newData.data.data[4]);

      parallel_spring_msg.right_number_revolutions = right_number_rev;

      left_number_rev = 0;
      left_number_rev = static_cast<int>(newData.data.data[7]);
      left_number_rev &= 0x07;
      left_number_rev <<= 8;
      left_number_rev |= static_cast<int>(newData.data.data[6]);

      parallel_spring_msg.left_number_revolutions = left_number_rev;

      right_limit = 0;
      right_limit = static_cast<int>(newData.data.data[5]);
      right_limit >>= 3;

      parallel_spring_msg.right_limit = right_limit;

      left_limit = 0;
      left_limit = static_cast<int>(newData.data.data[7]);
      left_limit >>= 3;

      parallel_spring_msg.left_limit = left_limit;

      break;
    }

    /*case left_segmentation_id:
    {
      left_segmentation = static_cast<int>(newData.data.data[0]);

      if (left_segmentation != last_left_segmentation)
      {
        segmentation_msg.left_segmentation = left_segmentation;
        last_left_segmentation = left_segmentation;
        segmentation_msg.header.stamp.sec = current_time.tv_sec;
        segmentation_msg.header.stamp.nsec = 1000 * current_time.tv_usec;
        segmentation_pub.publish(segmentation_msg);
      }
      break;
    }

    case right_segmentation_id:
    {
      right_segmentation = static_cast<int>(newData.data.data[0]);

      if (right_segmentation != last_right_segmentation)
      {
        segmentation_msg.right_segmentation = right_segmentation;
        last_right_segmentation = right_segmentation;
        segmentation_msg.header.stamp.sec = current_time.tv_sec;
        segmentation_msg.header.stamp.nsec = 1000 * current_time.tv_usec;
        segmentation_pub.publish(segmentation_msg);
      }
      break;
    }*/

    default: {}
      /*case timestamp_id:
      {
        pid_msg.header.stamp.sec = current_time.tv_sec;
        pid_msg.header.stamp.nsec = 1000 * current_time.tv_usec;
        pid_msg.header.frame_id = std::to_string(1);

        board_id = RIGHT_HIP_ID;
        control_output_digital = (newData.data.data[i+1] << 8 +
      newData.data.data[i]);

        pid_pub.publish(pid_msg);

        break;
      }
      */
    }
  }
}

void printUsage() {
  cout << "\n BIOMOT:\n";
  cout << "Option                Argument          Action / Notes\n";
  cout << "------                --------          --------------\n";
  cout << "-help, -h                               Print the command-line "
          "options for BIOMOT.\n";
  cout << "-reduceFrequency,-rf  ReductionFactor   Specify the frequency "
          "reduction factor .\n";
}

void printAuthors() {
  time_t now = std::time(0);
  tm *gmtm = std::gmtime(&now);
  cout << "Copyright (C) " << gmtm->tm_year + 1900 << endl;
  cout << "Juan Francisco Rascón Crespo, Monica Reggiani, Marco Matteo "
          "Bassa\n\n";
}

void printHeader() {
  cout << "rx_node_from_driver: Connects MACCEPA's driver with ROS\n";
}

void checkArguments(const int argc, char **argv, int &freq_reducing_factor) {
  if (argc < 2) {
    printUsage();
    exit(EXIT_FAILURE);
  }

  string option = "";

  for (int i = 1; i <= (argc - 1); i++) {
    option = argv[i];

    // print the usage options
    if ((option == "-help") || (option == "-h")) {
      printUsage();
      exit(EXIT_SUCCESS);
    }

    if ((option == "-reduceFrequency") || (option == "-rf")) {
      if (argv[i + 1] <= 0) {
        cout << "No frequency reduction factor specified!" << endl;
        // printUsage();
        // exit(EXIT_FAILURE);
        freq_reducing_factor = 1;
      } else
        freq_reducing_factor = atoi(argv[i + 1]);
    }
  }
}
