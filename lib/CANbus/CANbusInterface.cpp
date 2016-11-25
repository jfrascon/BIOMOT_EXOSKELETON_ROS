//______________________________________________________________________________
// Author(s): Monica Reggiani, Marco Matteo Bassa, Juan Francisco Rascón Crespo
// November 2014
// email: monica.reggiani@gmail.com, bassamarco91@gmail.com, jfrascon@gmail.com
//_____________________________________________________________________________
//

#include "CANbusInterface.h"
#include <vector>

using std::vector;

static char *cmdlinename[MAXSOCK];
static __u32 dropcnt[MAXSOCK];
static __u32 last_dropcnt[MAXSOCK];
static char devname[MAXIFNAMES][IFNAMSIZ + 1];
static int dindex[MAXIFNAMES];
static int max_devname_len; /* to prevent frazzled device name output */
const int canfd_on = 1;

namespace H2ros
{

CANbusInterface::CANbusInterface(char *device)
{
  bridge = 0;
  bridge_delay = 0;
  dropmonitor = 0;
  extra_msg_info = 0;
  view = 0;
  count = 0;
  builtSocket = false;
  rcvbuf_size = 0;
  timeout_config = {0, 0};
  ifnameW = device;
  currmax = 1; /*number of CAN devices, left to allow easy code modification */

  if (currmax > MAXSOCK)
  {
    fprintf(stderr, "More than %d CAN devices given!\n", MAXSOCK);
    exit(1);
  }

  for (i = 0; i < currmax; i++)
  {

    ptr = device;
    nptr = strchr(ptr, ',');
    s[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s[i] < 0)
    {
      perror("socket");
      exit(1);
    }
    cmdlinename[i] = ptr; /* save pointer to cmdline name of this socket */

    if (nptr)
      nbytes = nptr - ptr; /* interface name is up the first ',' */
    else
      nbytes = strlen(ptr); /* no ',' found => no filter definitions */

    if (nbytes >= IFNAMSIZ)
    {
      fprintf(stderr, "name of CAN device '%s' is too long!\n", ptr);
      exit(1);
    }

    if (nbytes > max_devname_len)
      max_devname_len = nbytes; /* for nice printing */

    addr.can_family = AF_CAN;

    memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
    strncpy(ifr.ifr_name, ptr, nbytes);
    if (strcmp(ANYDEV, ifr.ifr_name))
    {
      if (ioctl(s[i], SIOCGIFINDEX, &ifr) < 0)
      {
        perror("SIOCGIFINDEX");
        exit(1);
      }
      addr.can_ifindex = ifr.ifr_ifindex;
    }
    else
      addr.can_ifindex = 0; /* any can interface */

    if (nptr)
    {

      /* found a ',' after the interface name => check for filters */
      /* determine number of filters to alloc the filter space */
      numfilter = 0;
      ptr = nptr;
      while (ptr)
      {
        numfilter++;
        ptr++;                  /* hop behind the ',' */
        ptr = strchr(ptr, ','); /* exit condition */
      }

      /*rfilter = malloc(sizeof(struct can_filter) * numfilter);
			if (!rfilter) {
				fprintf(stderr, "Failed to create filter space!\n");
				return 1;
			}*/  // raising exceptions

      numfilter = 0;
      err_mask = 0;

      while (nptr)
      {

        ptr = nptr + 1;          /* hop behind the ',' */
        nptr = strchr(ptr, ','); /* update exit condition */

        if (sscanf(ptr, "%x:%x", &rfilter[numfilter].can_id,
                   &rfilter[numfilter].can_mask) == 2)
        {
          rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
          numfilter++;
        }
        else if (sscanf(ptr, "%x~%x", &rfilter[numfilter].can_id,
                        &rfilter[numfilter].can_mask) == 2)
        {
          rfilter[numfilter].can_id |= CAN_INV_FILTER;
          rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
          numfilter++;
        }
        else if (sscanf(ptr, "#%x", &err_mask) != 1)
        {
          fprintf(stderr, "Error in filter option parsing: '%s'\n", ptr);
          exit(1);
        }
      }

      if (err_mask)
        setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask,
                   sizeof(err_mask));

      if (numfilter)
        setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_FILTER, rfilter,
                   numfilter * sizeof(struct can_filter));

      free(rfilter);

    } /* if (nptr) */

    /* try to switch the socket into CAN FD mode */
    setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on,
               sizeof(canfd_on));

    if (rcvbuf_size)
    {

      int curr_rcvbuf_size;
      socklen_t curr_rcvbuf_size_len = sizeof(curr_rcvbuf_size);

      /* try SO_RCVBUFFORCE first, if we run with CAP_NET_ADMIN */
      if (setsockopt(s[i], SOL_SOCKET, SO_RCVBUFFORCE, &rcvbuf_size,
                     sizeof(rcvbuf_size)) < 0)
      {
#ifdef DEBUG
        printf("SO_RCVBUFFORCE failed so try SO_RCVBUF ...\n");
#endif
        if (setsockopt(s[i], SOL_SOCKET, SO_RCVBUF, &rcvbuf_size,
                       sizeof(rcvbuf_size)) < 0)
        {
          perror("setsockopt SO_RCVBUF");
          exit(1);
        }

        if (getsockopt(s[i], SOL_SOCKET, SO_RCVBUF, &curr_rcvbuf_size,
                       &curr_rcvbuf_size_len) < 0)
        {
          perror("getsockopt SO_RCVBUF");
          exit(1);
        }

        /* Only print a warning the first time we detect the adjustment */
        /* n.b.: The wanted size is doubled in Linux in net/sore/sock.c */
        if (!i && curr_rcvbuf_size < rcvbuf_size * 2)
          fprintf(stderr,
                  "The socket receive buffer size was "
                  "adjusted due to /proc/sys/net/core/rmem_max.\n");
      }
    }

    if (1)
    {
      const int timestamp_on = 1;
      if (setsockopt(s[i], SOL_SOCKET, SO_TIMESTAMP, &timestamp_on,
                     sizeof(timestamp_on)) < 0)
      {
        perror("setsockopt SO_TIMESTAMP");
        exit(1);
      }
    }
    if (bind(s[i], (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      perror("bind");
      exit(1);
    }
  }

  /* these settings are static and can be held out of the hot path */
  iov.iov_base = &frame;
  msg.msg_name = &addr;
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = &ctrlmsg;
}

CANbusInterface::~CANbusInterface()
{
  if (builtSocket)
    close(sW);
  for (i = 0; i < currmax; i++)
    close(s[i]);
}

int idx2dindex(int ifidx, int socket)
{

  int i;
  struct ifreq ifr;

  for (i = 0; i < MAXIFNAMES; i++)
  {
    if (dindex[i] == ifidx)
      return i;
  }
  /* create new interface index cache entry */
  /* remove index cache zombies first */
  for (i = 0; i < MAXIFNAMES; i++)
  {
    if (dindex[i])
    {
      ifr.ifr_ifindex = dindex[i];
      if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
        dindex[i] = 0;
    }
  }
  for (i = 0; i < MAXIFNAMES; i++)
    if (!dindex[i]) /* free entry */
      break;
  if (i == MAXIFNAMES)
  {
    fprintf(stderr, "Interface index cache only supports %d interfaces.\n",
            MAXIFNAMES);
    exit(1);
  }
  dindex[i] = ifidx;

  ifr.ifr_ifindex = ifidx;
  if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
    perror("SIOCGIFNAME");

  if (max_devname_len < strlen(ifr.ifr_name))
    max_devname_len = strlen(ifr.ifr_name);

  strcpy(devname[i], ifr.ifr_name);

#ifdef DEBUG
  printf("new index %d (%s)\n", i, devname[i]);
#endif

  return i;
}

void CANbusInterface::read(can_frame &currentData)
{
  FD_ZERO(&rdfs);
  for (i = 0; i < currmax; i++)
    FD_SET(s[i], &rdfs);

  if ((ret = select(s[currmax - 1] + 1, &rdfs, NULL, NULL, timeout_current)) <=
      0)
  {
    perror("select");
    exit(1);
  }

  for (i = 0; i < currmax; i++)
  { /* check all CAN RAW sockets */

    if (FD_ISSET(s[i], &rdfs))
    {

      int idx;

      /* these settings may be modified by recvmsg() */
      iov.iov_len = sizeof(frame);
      msg.msg_namelen = sizeof(addr);
      msg.msg_controllen = sizeof(ctrlmsg);
      msg.msg_flags = 0;

      nbytes = recvmsg(s[i], &msg, 0);  // receiving happens here
      if (nbytes < 0)
      {
        perror("read");
        exit(1);
      }

      if ((size_t)nbytes == CAN_MTU)
        maxdlen = CAN_MAX_DLEN;
      else if ((size_t)nbytes == CANFD_MTU)
        maxdlen = CANFD_MAX_DLEN;
      else
      {
        fprintf(stderr, "read: incomplete CAN frame\n");
        exit(1);
      }

      if (count && (--count == 0))
        exit(1);

      for (cmsg = CMSG_FIRSTHDR(&msg); cmsg && (cmsg->cmsg_level == SOL_SOCKET);
           cmsg = CMSG_NXTHDR(&msg, cmsg))
      {
        if (cmsg->cmsg_type == SO_RXQ_OVFL)
          dropcnt[i] = *(__u32 *)CMSG_DATA(cmsg);
      }

      /* check for (unlikely) dropped frames on this specific socket */
      if (dropcnt[i] != last_dropcnt[i])
      {

        __u32 frames = dropcnt[i] - last_dropcnt[i];

        printf(
            "DROPCOUNT: dropped %d CAN frame%s on '%s' socket (total drops "
            "%d)\n",
            frames, (frames > 1) ? "s" : "", cmdlinename[i], dropcnt[i]);

        last_dropcnt[i] = dropcnt[i];
      }

      idx = idx2dindex(addr.can_ifindex, s[i]);

      /* once we detected a EFF frame indent SFF frames accordingly */
      if (frame.can_id & CAN_EFF_FLAG)
        view |= CANLIB_VIEW_INDENT_SFF;
      currentData = frame;
    }
  }
}

void CANbusInterface::canWrite(const can_frame &dataToWrite)
// void CANbusInterface::canWrite(const void* dataToWrite, int num_bytes)
{
  if (!builtSocket) /*Opens the socket only when the first message is sent */
  {
    if ((sW = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
      perror("Error while opening socket");
      return;
    }

    strcpy(ifr.ifr_name, ifnameW);
    ioctl(sW, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

    if (bind(sW, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      perror("Error in socket bind");
      return;
    }
    builtSocket = true;
  }
  nbytes = write(sW, &dataToWrite,
                 sizeof(can_frame));  // use & frame if passing a copy
  // nbytes = write(sW, dataToWrite, num_bytes);

  // printf("Wrote %d bytes\n", nbytes);
  return;
}
}
