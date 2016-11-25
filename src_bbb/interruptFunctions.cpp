/**
  * Author: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  */

#include "interruptFunctions.h"
#include "CANbusInterface.h"
#include "constants.h"
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <thread>

using namespace std;
using namespace H2ros;

void get_current_time(struct StartTimeContainer *start_time_container,
                      struct timeval *current_time) {
  unique_lock<mutex> mlock(start_time_container->start_time_mutex);

  // cout << std::this_thread::get_id() << " get_current_time" << endl;
  // cout << std::this_thread::get_id() << " writing_or_wrinting_available:" <<
  // start_time_container->writing_or_wrinting_available << endl;

  while (start_time_container->writing_or_wrinting_available) {
    // cout << std::this_thread::get_id() << " Me duermo" << endl;
    start_time_container->read_start_time.wait(mlock);
  }

  start_time_container->number_readers++;
  // cout << std::this_thread::get_id() << " Numero lectores " <<
  // start_time_container->number_readers << endl;
  mlock.unlock();

  timeval absolute_current_time;
  gettimeofday(&absolute_current_time, NULL);

  // uint64_t timestamp_usec  = 1000000 * (absolute_current_time.tv_sec -
  // start_time_container->start_time.tv_sec) + (absolute_current_time.tv_usec -
  // start_time_container->start_time.tv_usec);
  uint64_t timestamp_usec = absolute_current_time.tv_sec;
  timestamp_usec -= start_time_container->start_time.tv_sec;
  timestamp_usec *= 1000000;
  timestamp_usec += absolute_current_time.tv_usec;
  timestamp_usec -= start_time_container->start_time.tv_usec;

  current_time->tv_sec = (timestamp_usec / 1000000); // Transform into seconds
  // Remaining time expressed in microseconds
  current_time->tv_usec = timestamp_usec - (1000000 * current_time->tv_sec);

  mlock.lock();
  // if(--start_time_container->number_readers == 0)
  start_time_container->number_readers--;
  // cout << std::this_thread::get_id() << " Numero lectores " <<
  // start_time_container->number_readers << endl;
  // cout << std::this_thread::get_id() << " Notifico al escritor." << endl;
  start_time_container->write_start_time.notify_one();
  mlock.unlock();
}

/* Functions used to handle an extern interrupt on the BBB's port gpio7.*/

void resettingId(struct StartTimeContainer *start_time_container) {

  // cout << std::this_thread::get_id() << " resettingId" << endl;

  GMainLoop *loop = g_main_loop_new(0, 0);
  int fd = open("/sys/class/gpio/gpio7/value", O_RDONLY | O_NONBLOCK);
  /*
   Will only work if previously were executed
   -echo 7 > /sys/class/gpio/export
   -echo both > /sys/class/gpio/gpio7/edge
   */
  GIOChannel *channel = g_io_channel_unix_new(fd);
  GIOCondition cond = GIOCondition(G_IO_PRI);
  guint id = g_io_add_watch(channel, cond, onTriggerEvent,
                            (gpointer)start_time_container);
  g_main_loop_run(loop); /*Waits for an interrupt on port gpio7 (pin p9-42)*/
}

static gboolean onTriggerEvent(GIOChannel *channel, GIOCondition condition,
                               gpointer user_data) {
  GError *error = 0;
  gchar buf[8];
  gsize bytes_read = 0;
  gsize buf_sz = sizeof(buf);
  g_io_channel_seek_position(channel, 0, G_SEEK_SET, 0);
  GIOStatus rc =
      g_io_channel_read_chars(channel, buf, buf_sz - 1, &bytes_read, &error);

  if ((atoi(buf)) == 1) {
    StartTimeContainer *start_time_container = (StartTimeContainer *)user_data;

    std::unique_lock<std::mutex> mlock(start_time_container->start_time_mutex);
    // cout << std::this_thread::get_id() << " onTriggerEvent, bloqueo el mutex"
    // << endl;
    // A writer thread needs to write, so, all the reader threads that are
    // reading now must stop first. No more reader threads are allowed to read
    // until the writer thread finishes the updating.
    start_time_container->writing_or_wrinting_available = true;
    // cout << std::this_thread::get_id() << " number_readers " <<
    // start_time_container->number_readers << endl;

    while (start_time_container->number_readers) {
      // cout << std::this_thread::get_id() << ". Duermo" << endl;
      start_time_container->write_start_time.wait(mlock);
    }

    timeval actual_time;
    // gettimeofday(&start_time_container->start_time, NULL);
    gettimeofday(&actual_time, NULL);

    bool send_reset = false;

    // Avoid multiple resets within a time interval less than
    // SECS_BETWEEN_TRIGGERS
    if (actual_time.tv_sec - start_time_container->start_time.tv_sec >
        SECS_BETWEEN_TRIGGERS) {
      start_time_container->start_time.tv_sec = actual_time.tv_sec;
      start_time_container->start_time.tv_usec = actual_time.tv_usec;
      send_reset = true;
    }

    start_time_container->writing_or_wrinting_available = false;
    // cout << std::this_thread::get_id() << ". Termino de escribir la hora y
    // notifico a los lectores." << endl;
    start_time_container->read_start_time.notify_all();

    if (send_reset) {
      can_frame sendedFrame;
      memset(&sendedFrame, 0, sizeof(can_frame));
      sendedFrame.can_id = 0x210;
      sendedFrame.can_dlc = 6;
      sendedFrame.data[0] = 1;
      CANbusInterface canbusInterface_("can0");
      canbusInterface_.canWrite(sendedFrame);
    }
  }

  return 1;
}
