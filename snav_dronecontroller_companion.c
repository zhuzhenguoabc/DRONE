/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

// system includes
#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

// Snapdragon Navigator
#include "snapdragon_navigator.h"

#define MAXBUFLEN 512

#define LOG_FILE "/usr/logs/drone_control.log"


// Buttons bitfield mappings
#define BUTTON_LAND 16
#define BUTTON_SPIN_PROPS 32

void print_usage()
{
  printf("-i [IP_ADDRESS]     IP address to bind to, default = 192.168.1.1\n");
  printf("-p [PORT]           UDP port, default = 14556\n");
  printf("-v                  Print version information\n");
  printf("-h                  print this message.\n");
}

void print_version()
{
  printf("v%s\n",VERSION);
}

int main(int argc, char* argv[])
{
  const char* ip_address = "192.168.1.1";
  int port = 14556;
  int c;

	/*
	FILE *fp;

	//clear the file first
	if ((fp = fopen(LOG_FILE, "w+")) != NULL)
	{
		fclose(fp);
	}

	freopen(LOG_FILE, "a", stdout); setbuf(stdout, NULL);
	freopen(LOG_FILE, "a", stderr); setbuf(stderr, NULL);
	*/

  // Parse command line args
  while ((c = getopt(argc, argv, "i:p:vh")) != -1)
  {
    switch(c)
    {
      case 'i':
        ip_address = optarg;
        break;
      case 'p':
        port = atoi(optarg);
        break;
      case 'v':
        print_version();
        return 0;
      case 'h':
        print_usage();
        return 0;
      default:
        print_usage();
        return -1;
    }
  }

  int sockfd;
  struct sockaddr_in my_addr; // my address information
  struct sockaddr_in their_addr; // connector's address information
  socklen_t addr_len;
  int numbytes;
  char buf[MAXBUFLEN];

  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
  {
    perror("socket");
    exit(1);
  }

  my_addr.sin_family = AF_INET;    // host byte order
  my_addr.sin_addr.s_addr = inet_addr(ip_address); // automatically fill with my IP
  my_addr.sin_port = htons(port);  // short, network byte order
  memset(&(my_addr.sin_zero), '\0', 8); // zero the rest of the struct

  const int kMaxNumAttempts = 10;
  printf("Using IP %s and port %d\n", ip_address, port);
  while (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) != 0)
  {
    static int attempt_number = 0;
    printf("Attempt %d to bind to IP %s and port %d was unsuccessful\n", attempt_number,
        ip_address, port);

    if (attempt_number >= kMaxNumAttempts)
    {
      printf("Unable to bind after %d attempts.\n", attempt_number);
      return -1;
    }

    ++attempt_number;
    usleep(1e6);
  }
  printf("Bound to IP %s and port %d\n", ip_address, port);

  addr_len = sizeof(struct sockaddr);

  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return -1;
  }

  while(1)
  {
    // blocking until message is received
    if ((numbytes = recvfrom(sockfd, buf, MAXBUFLEN-1 , 0,
            (struct sockaddr *)&their_addr, &addr_len)) == -1)
    {
      perror("recvfrom");
      exit(1);
    }


	struct timeval time_val;
	gettimeofday(&time_val, NULL);
	double time_now = time_val.tv_sec + time_val.tv_usec * 1e-6;

	printf("dronecontroller loop time_now=%lf\n",time_now);

    /**
     * Check packet header explicitly
     * byte 0 = MAVLINK_STX
     * byte 1 = length
     * byte 3 = sysid
     * byte 4 = compid
     * byte 5 = msgid
     * TODO check MAVLINK CRC
     */
    if (buf[0] == 0xfe && buf[1] == 0xb && buf[3] == 0xff && buf[4] == 0xbe && buf[5] == 0x45)
    {
      int16_t rolli;   memcpy(&rolli,buf+8,2);
      int16_t pitchi;  memcpy(&pitchi,buf+6,2);
      int16_t yawi;    memcpy(&yawi,buf+12,2);
      int16_t thrusti; memcpy(&thrusti,buf+10,2);
      uint16_t buttons; memcpy(&buttons,buf+14,2);


	  printf("dronecontroller loop data:%d,%d,%d,%d\n",rolli, pitchi, yawi, thrusti);

      static bool landing_in_progress = false;
      if (buttons == BUTTON_SPIN_PROPS)
      {
        sn_spin_props();
      }
      else if (buttons == BUTTON_LAND)
      {
        landing_in_progress = true;
      }

      /**
       * Remap from what DroneController is sending into the range -1 to 1
       * and send commands to flight controller.
       *
       * Remember:
       * cmd0: "forward/backward" type command, "forward" is positive
       * cmd1: "left/right" type command, "left" is positive
       * cmd2: "up/down" type command, "up" is positive
       * cmd3: "rotate" type command, "counter-clockwise" is positive
       * See Snapdragon Navigator Developer Guide for details.
       */
      const float kMin = -1;
      const float kMax = 1;
      float cmd0 = -((float)(pitchi+441)*(kMax-kMin)/882.+ kMin);
      float cmd1 = -((float)(rolli+441)*(kMax-kMin)/882.+ kMin);
      float cmd2 = (float)(thrusti)*(kMax-kMin)/1000.+ kMin;
      float cmd3 = -((float)(yawi+250)*(kMax-kMin)/500.+ kMin);

      // Use type to control how the commands get interpreted.
      SnRcCommandType type = SN_RC_OPTIC_FLOW_POS_HOLD_CMD;

      if (sn_update_data() != 0)
      {
        printf("sn_update_data failed. Verify flight control is running.\n");
      }
      else
      {
        if (landing_in_progress)
        {
          // If user touches roll/pitch stick, stop landing
          if (fabs(cmd0) > 1e-4 || fabs(cmd1) > 1e-4)
          {
            landing_in_progress = false;
          }
          else
          {
            /**
             * Override commands from DroneController to land:
             *   zero lateral velocity
             *   negative vertical velocity to descend
             *   zero yaw rate
             */
            cmd0 = 0;
            cmd1 = 0;
            cmd2 = -0.75;
            cmd3 = 0;
          }

          // Check if landed
          if (snav_data->general_status.on_ground == 1)
          {
            // Flight control detects ground, safe to stop propellers
            sn_stop_props();
            landing_in_progress = false;
          }

        }


		printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		printf("UDP SNAV_SEND_CMD cmd0,cmd1,cmd2,cmd3:%f,%f,%f,%f\n",
						cmd0,cmd1,cmd2,cmd3);
		printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n");
		fflush(stdout);

        // Send the commands to Snapdragon Navigator with default RC options
        sn_send_rc_command(type, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
      }
    }
  }

  close(sockfd);

  //fclose(stdout);
  //fclose(stderr);

  return 0;
}

