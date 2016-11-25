/**
  * Authors: Juan Francisco Rascón Crespo. jfrascon@gmail.com
  *          Marco Matteo Bassa. bassamarco91@gmail.com
  */

#ifndef INTERRUPT_FUNCTIONS_H
#define INTERRUPT_FUNCTIONS_H

/* Functions used to handle an extern interrupt on the BBB's port gpio7.
 * This extern interruption is a digital pulse used for devices synchronization
 * purpose.
 * Whenever this pulse is received in the BBB the reference time base for all
 * the ROS messages in reseted.
 */

#include <condition_variable>
#include <fcntl.h>
#include <glib.h> /*Glib2 library needs to be set-up.*/
#include <mutex>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

struct StartTimeContainer {
  timeval start_time;
  std::mutex start_time_mutex;
  std::condition_variable write_start_time;
  std::condition_variable read_start_time;
  int number_readers;
  bool writing_or_wrinting_available;
};

void resettingId(struct StartTimeContainer *start_time_container);
static gboolean onTriggerEvent(GIOChannel *channel, GIOCondition condition,
                               gpointer user_data);
void get_current_time(struct StartTimeContainer *start_time_container,
                      timeval *current_time);

#endif
