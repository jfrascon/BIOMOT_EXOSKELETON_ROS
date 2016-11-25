/** Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * Thread used to read the pattern file meanwhile the pattern GUI is still
  * responsive.
  */

#include "worker_thread_patterns_gui.h"
#include "constants.h"
#include <fstream>
#include <iostream>

using namespace std;

void WorkerThread::run()
{
  std::ifstream reader;
  reader.open(filename.toStdString());
  pattern.clear();

  int i = 0;
  float value;

  std::unique_lock<std::mutex> mlock(file_reader_threads_mutex);
  active_threads_number++;
  mlock.unlock();

  while (/*!reader.eof()*/ reader >> value && i < num_values_pattern)
  {
    // cout << i << " " << value << endl;
    pattern.push_back(value);
    i++;
  }

  mlock.lock();
  active_threads_number--;
  mlock.unlock();

  emit pattern_loaded(pattern_index, i);
  reader.close();
}
