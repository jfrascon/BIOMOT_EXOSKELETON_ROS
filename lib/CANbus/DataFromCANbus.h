//______________________________________________________________________________
// Author(s): Monica Reggiani, Marco Matteo Bassa, Juan Francisco Rascón Crespo
// November 2014
// email: monica.reggiani@gmail.com, bassamarco91@gmail.com, jfrascon@gmail.com
//_____________________________________________________________________________
//

#ifndef DataFromCANbus_h
#define DataFromCANbus_h

#include "Latch.h"
#include "Queue.h"
#include "QueueData.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <list>
#include <vector>

namespace H2ros
{
class DataFromCANbus
{
 public:
  DataFromCANbus();
  DataFromCANbus(char* nameDevice);
  DataFromCANbus(const DataFromCANbus& dataFromCANbus) = delete;
  ~DataFromCANbus();

  char* nameOfDevice;
  typedef QueueData<can_frame> FrameType;
  CEINMS::Concurrency::Queue<FrameType> queue;
};
}

#endif
