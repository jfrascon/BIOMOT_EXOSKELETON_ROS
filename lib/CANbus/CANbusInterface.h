//______________________________________________________________________________
// Author(s): Monica Reggiani, Marco Matteo Bassa, Juan Francisco Rascón Crespo
// November 2014
// email: monica.reggiani@gmail.com, bassamarco91@gmail.com, jfrascon@gmail.com
//_____________________________________________________________________________
//

#ifndef CANbusInterface_h
#define CANbusInterface_h

#include <ctype.h>
#include <errno.h>
#include <libgen.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <net/if.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#define MAXANI 4

// defines that were included in "lib.h"
#define CANLIB_VIEW_INDENT_SFF 0x10
#define CL_DATA sizeof(".AA")
#define CL_CFSZ (2 * CL_ID + 64 * CL_DATA)
#define CL_ID (sizeof("12345678##1"))

#define MAXSOCK 16    /* max. number of CAN interfaces */
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */
#define ANYDEV "any"  /* name of interface to receive from any CAN interface */
#define ANL "\r\n"    /* newline in ASC mode */

namespace H2ros
{
class CANbusInterface
{
 public:
  CANbusInterface(char *device);
  ~CANbusInterface();
  void read(can_frame &currentData);
  void canWrite(const can_frame &dataToWrite);
  // void canWrite(const void* dataToWrite, int num_bytes);

 private:
  can_frame frame;
  fd_set rdfs;
  int s[MAXSOCK];
  useconds_t bridge_delay;
  unsigned char dropmonitor;
  unsigned char extra_msg_info;
  unsigned char view;
  char *ptr, *nptr, *ifnameW;
  struct sockaddr_can addr;
  char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
  struct iovec iov;
  struct msghdr msg;
  struct cmsghdr *cmsg;
  struct can_filter *rfilter;
  can_err_mask_t err_mask;
  int nbytes, i, maxdlen, bridge, count, rcvbuf_size, opt, ret, currmax,
      numfilter, sW;
  struct ifreq ifr;
  struct timeval timeout, timeout_config, *timeout_current = NULL;
  bool builtSocket;
};
}

#endif
