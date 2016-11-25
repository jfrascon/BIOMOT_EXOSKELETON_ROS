
/** Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  */

#ifndef WORKER_THREAD_GUI_H
#define WORKER_THREAD_GUI_H

#include <mutex>
#include <vector>

#include <QString>
#include <QtCore>

Q_DECLARE_METATYPE(int);

class WorkerThread : public QThread
{
  Q_OBJECT

 public:
  WorkerThread(const QString& filename_, const int pattern_index_,
               std::vector<float>& pattern_,
               std::mutex& _file_reader_threads_mutex,
               int& _active_threads_number)
      : filename(filename_),
        pattern_index(pattern_index_),
        pattern(pattern_),
        file_reader_threads_mutex(_file_reader_threads_mutex),
        active_threads_number(_active_threads_number)
  {
  }

 signals:
  void pattern_loaded(int pattern_index, int num_setpoints_in_file);

 private:
  const QString filename;
  const int pattern_index;
  std::vector<float>& pattern;
  std::mutex& file_reader_threads_mutex;
  int& active_threads_number;
  void run();
};

#endif
