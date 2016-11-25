/** Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * In this file appears all the constants used in the ROS nodes and in the
  * joint boards firmware.
  */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

#define BIOMOT
//#define H2R

#define PID_MANUAL
//#define PID_DSP

const float max_angle_deg = 360.00f;           // In degrees
const float max_angle_rad = 6.28318530717959f; // In radians
const float angle_deg2percent =
    100.00f / max_angle_deg; // 100.00f/MAX_POS_DEG) = 0.27777777777778;
const float upper_angle_setpoint = +0.50f;
const float lower_angle_setpoint = -0.50f;

const float max_lvl_deg = 360.0f; // IN degrees
const float upper_lvl_setpoint = +0.50f;
const float lower_lvl_setpoint = -0.50f;

const float max_tor_Nm = 25.00f; // In Nm
const float torque_Nm2percent =
    100.00f / max_tor_Nm; // 100.00f/max_tor_Nm = 4.00f
const float upper_tor_setpoint = +1.00f;
const float lower_tor_setpoint = -1.00f;

const float pid_tolerance = 0.1; // 0.1%

const float Kmin = -100.00f;
const float Kmax = +100.00f;

const float dist_elc = 0.130f; // In m
const float max_fix_link_angle_digital = 8000.0f;
const float max_alpha_angle_digital = 16384.0f;
const float max_lever_arm_angle_digital = 8000.0f;
const float max_torque_digital = 32768.0f;
const float max_pid_digital = 32768.0f;

const float fix_link_angle_dig2rad = max_angle_rad / max_fix_link_angle_digital;
const float alpha_angle_dig2rad = max_angle_rad / max_alpha_angle_digital;
const float lever_arm_angle_dig2rad =
    max_angle_rad / max_lever_arm_angle_digital;
const float pid_dig2percent = 100.00f / max_pid_digital;
const float torque_dig2Nm = max_tor_Nm / max_torque_digital;

const unsigned int num_values_pattern = 15000;

const float upper_precompression_limit = 21.00f; // In mm
const float slope_elc = 0.221697f;
const float offset_elc = -113.398f;

// GAP: allow negative values for precompression, so no need to change sign into
// the firmware for TL
// const float lower_precompression_limit = 0.00f // In mm
const float lower_precompression_limit = -21.00f;
// GAP

const unsigned int heel_strike_threshold = 1450;
const unsigned int heel_up_threshold = 800;

const unsigned int timestamp_id = 11;
const unsigned int co_id = 12;

const int hip_mo_ss_id = 13;
const int hip_laa_aa_id = 14;
const int hip_torque_fla_id = 15;

const int knee_mo_ss_id = 16;
const int knee_laa_aa_id = 17;
const int knee_torque_fla_id = 18;

const int ankle_mo_ss_id = 19;
const int ankle_laa_aa_id = 20;
const int ankle_torque_fla_id = 21;

const int knee_parallel_spring = 22;

// const int left_segmentation_id = 242;
// const int right_segmentation_id = 245;
// const int left_segmentation_0_1_id = 242;
// const int left_segmentation_0_2_id = 243;
// const int right_segmentation_0_1_id = 245;
// const int right_segmentation_0_2_id = 246;

// ***** ***** ***** ***** ***** ***** ***** *****
const int lower_can_id = timestamp_id;         // <===
const int upper_can_id = knee_parallel_spring; // <===

const float rad2grad = 57.29577951308233f;

const float SECS_BETWEEN_TRIGGERS = 5.00f;

enum message_id {
  KP_MESSAGE_ID = 1,
  KI_MESSAGE_ID = 2,
  KD_MESSAGE_ID = 3,
  SETPOINT_MESSAGE_ID = 4,
  PRECOMPRESSION_MESSAGE_ID = 5,
  EVENT_MESSAGE_ID = 6,
  REF_ALPHA_ANGLE_MESSAGE_ID = 7,
  REF_FIX_LINK_ANGLE_MESSAGE_ID = 8,
  AUX_VAR_MESSAGE_ID = 9,
  ENCODERS_CALIBRATION_MESSAGE_ID = 10,
  PID_MESSAGE_ID = 29,
  JOINT_STATE_MESSAGE_ID = 30,
  SENSORS_MESSAGE_ID = 31
};

enum pid_id {
  POS_PID_CONTROLLER_ID = 0,
  LVL_PID_CONTROLLER_ID = 1,
  TOR_PID_CONTROLLER_ID = 2,
  TCL_PID_CONTROLLER_ID = 3,
  PID_CONTROLLER_ID_ERROR = 4
};

enum board_id {
  LEFT_HIP_ID = 32,
  LEFT_KNEE_ID = 64,
  LEFT_ANKLE_ID = 96,
  RIGHT_HIP_ID = 128,
  RIGHT_KNEE_ID = 160,
  RIGHT_ANKLE_ID = 192
};

#endif
