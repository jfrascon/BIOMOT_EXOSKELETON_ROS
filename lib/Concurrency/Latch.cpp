//______________________________________________________________________________
// Author(s): Monica Reggiani, Marco Matteo Bassa, Juan Francisco Rascón Crespo
// November 2014
// email: monica.reggiani@gmail.com, bassamarco91@gmail.com, jfrascon@gmail.com
//_____________________________________________________________________________
//

#include <algorithm>
#include <iostream>

#include "Latch.h"

namespace CEINMS
{
namespace Concurrency
{

Latch::Latch() : count_(0) {}

Latch::Latch(int count) : count_(count) {}

void Latch::setCount(int count)
{
  if (count_ != 0)
  {
    std::cout << "You are not allowed to reset a Latch\n";
    exit(EXIT_FAILURE);
  }
  else
    count_ = count;
}

void Latch::wait()
{
  std::unique_lock<std::mutex> mlock(mutex_);
  if (count_ == 0)
  {
    throw std::logic_error("internal count == 0");
  }
  if (--count_ == 0)
    condition_.notify_all();
  else
  {
    while (count_ > 0)
      condition_.wait(mlock);
  }
  mlock.unlock();
}
};
};
