//______________________________________________________________________________
// Author(s): Monica Reggiani, Marco Matteo Bassa, Juan Francisco Rascón Crespo
// November 2014
// email: monica.reggiani@gmail.com, bassamarco91@gmail.com, jfrascon@gmail.com
//_____________________________________________________________________________
//

#include "DataFromCANbus.h"
#include <vector>

namespace H2ros
{

DataFromCANbus::DataFromCANbus() {}
DataFromCANbus::DataFromCANbus(char* nameDevice) { nameOfDevice = nameDevice; }

DataFromCANbus::~DataFromCANbus() {}
};
