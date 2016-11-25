//______________________________________________________________________________
// Author(s): Monica Reggiani, Marco Matteo Bassa, Juan Francisco Rascón Crespo
// November 2014
// email: monica.reggiani@gmail.com, bassamarco91@gmail.com, jfrascon@gmail.com
//_____________________________________________________________________________
//

#ifndef CANbusReader_h
#define CANbusReader_h

#include "CANbusInterface.h"
#include "DataFromCANbus.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <time.h>
#include <vector>

namespace H2ros
{

class CANbusReader
{

 public:
  CANbusReader() = delete;
  CANbusReader(DataFromCANbus& dataFromCANbus);
  ~CANbusReader();
  void operator()();
  void addData(const can_frame& currentData, timeval timeStamp);

 private:
  DataFromCANbus& dataFromCANbus_;
  can_frame currentData_;
  char* canDevice;
};
}

#endif
