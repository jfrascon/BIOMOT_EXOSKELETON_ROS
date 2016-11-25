//______________________________________________________________________________
// Author(s): Monica Reggiani, Marco Matteo Bassa, Juan Francisco Rascón Crespo
// November 2014
// email: monica.reggiani@gmail.com, bassamarco91@gmail.com, jfrascon@gmail.com
//_____________________________________________________________________________
//

#include "CANbusReader.h"
#include "DataFromCANbus.h"
#include <string>
using std::string;
#include <vector>
using std::vector;
#include "QueueData.h"
#include <cstdlib>

namespace H2ros
{

CANbusReader::CANbusReader(DataFromCANbus& dataFromCANbus)
    : dataFromCANbus_(dataFromCANbus)
{
  canDevice = dataFromCANbus_.nameOfDevice;
}

CANbusReader::~CANbusReader() {}

void CANbusReader::addData(const can_frame& currentCANbusData,
                           timeval timeStamp)
{
  QueueData<can_frame> canbusDataToPush;
  canbusDataToPush.data = currentCANbusData;
  canbusDataToPush.time = timeStamp;
  dataFromCANbus_.queue.push(canbusDataToPush);
}

void CANbusReader::operator()() /* Function executed automatically with the
                                   thread (dynamic object)*/
{
  bool stillData = true;
  timeval timeStamp_;
  CANbusInterface canbusInterface_(canDevice);
  while (stillData)
  {
    canbusInterface_.read(currentData_);
    gettimeofday(&timeStamp_, NULL);
    addData(currentData_, timeStamp_);
  }
}
}
