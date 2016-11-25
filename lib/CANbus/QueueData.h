//______________________________________________________________________________
// Author(s): Monica Reggiani, Marco Matteo Bassa, Juan Francisco Rascón Crespo
// November 2014
// email: monica.reggiani@gmail.com, bassamarco91@gmail.com, jfrascon@gmail.com
//_____________________________________________________________________________
//

#ifndef QueueData_h
#define QueueData_h
#include <time.h>

template <typename T>
struct QueueData
{
  timeval time;
  T data;
};

#endif
