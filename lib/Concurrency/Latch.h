//______________________________________________________________________________
// Author(s): Monica Reggiani, Marco Matteo Bassa, Juan Francisco Rascón Crespo
// November 2014
// email: monica.reggiani@gmail.com, bassamarco91@gmail.com, jfrascon@gmail.com
//_____________________________________________________________________________
//

#ifndef Latch_h
#define Latch_h

#include <condition_variable>
#include <mutex>
#include <thread>

namespace CEINMS
{
namespace Concurrency
{
class Latch
{
 public:
  Latch();
  Latch(int count);

  void setCount(int count);

  void wait();

  Latch(const Latch&) = delete;
  Latch& operator=(const Latch&) = delete;

 private:
  int count_;

  std::condition_variable condition_;
  std::mutex mutex_;
};
};
};

#endif
