/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  */

#include <iostream>
#include <string>

#include <ctime>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "h2r/SingleCommandMessage.h"
#include "ros/ros.h"

#include "CANbusInterface.h"

#define FRAMEDIM 6
//#define DEBUG_HEX 1

using namespace std;
using namespace ros;
using namespace h2r;
using namespace H2ros;

void printUsage();
void printAuthors();
void printHeader();
void checkArguments(const int argc, char **argv);
CANbusInterface canbusInterface_("can0");

void chatterCallback(const SingleCommandMessage::ConstPtr &msg) {
  can_frame sendedFrame;
  sendedFrame.can_dlc = FRAMEDIM;
  sendedFrame.data[4] = (uint8_t)(msg->type);
  sendedFrame.data[5] = (uint8_t)(0);

  for (int i = 0; i < msg->number_elements; i++) {
    memcpy(&(sendedFrame.can_id), &(msg->can_id[i]), sizeof(int));
    memcpy(sendedFrame.data, &(msg->data[i]), sizeof(float));

// sendedFrame.can_id = msg->can_id;
// float *f_buf = (float*)sendedFrame.data;
// *f_buf = msg->data;
// *sendedFrame.data = *f_buf;

#ifdef DEBUG_HEX
    cout << showbase      // show the 0x prefix
         << internal      // fill between the prefix and the number
         << setfill('0'); // fill with 0s
    cout << hex << uppercase << setw(4) << static_cast<int>(sendedFrame.can_id)
         << "   " << hex << uppercase << setw(4)
         << static_cast<int>(sendedFrame.data[5]) << " " << hex << uppercase
         << setw(4) << static_cast<int>(sendedFrame.data[4]) << " " << hex
         << uppercase << setw(4) << static_cast<int>(sendedFrame.data[3]) << " "
         << hex << uppercase << setw(4) << static_cast<int>(sendedFrame.data[2])
         << " " << hex << uppercase << setw(4)
         << static_cast<int>(sendedFrame.data[1]) << " " << hex << uppercase
         << setw(4) << static_cast<int>(sendedFrame.data[0]) << nouppercase
         << dec << endl;
#endif

    canbusInterface_.canWrite(sendedFrame);
  }
}

int main(int argc, char **argv) {
  printHeader();
  printAuthors();

#ifdef LOG
  cout << "Check configuration data...\n";
#endif
  checkArguments(argc, argv);

  init(argc, argv, "tx_node");
  NodeHandle n;
  cout << "Listening (ctrl+c to exit)" << endl;
  Subscriber sub = n.subscribe("tx_topic", 1, chatterCallback);
  spin();

  exit(EXIT_SUCCESS);
}

void printUsage() {
  cout << "\n\n BIOMOT:\n";
  cout << "Option            Argument          Action / Notes\n";
  cout << "------            --------          --------------\n";
  cout << "-help, -h                           Print the command-line options "
          "for tx_node_to_driver.\n";
}

void printAuthors() {
  time_t now = time(0);
  tm *gmtm = gmtime(&now);
  cout << "Copyright (C) " << gmtm->tm_year + 1900 << endl;
  cout << "Juan Francisco Rascón Crespo, Monica Reggiani, Marco Matteo "
          "Bassa\n\n";
}

void printHeader() { cout << "Ready to send commands to MACCEPA actuator.\n "; }

void checkArguments(const int argc, char **argv) {
  string option = "";

  for (int i = 1; i <= (argc - 1); i++) {
    option = argv[i];
    // print the usage options
    if ((option == "-help") || (option == "-h")) {
      printUsage();
      exit(EXIT_SUCCESS);
    }
  }
}
