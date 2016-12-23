/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "snapdragon_navigator.h"

void print_usage()
{
  printf("-r  Send RPM commands to ESCs, default\n");
  printf("-p  Send PWM commands to ESCs\n");
  printf("-h  Print this message\n");
}

int main(int argc, char* argv[])
{
  int c;
  bool send_rpms = true;

  while ((c = getopt(argc, argv, "rph")) != -1)
  {
    switch (c)
    {
      case 'r':
        send_rpms = true;
        break;
      case 'p':
        send_rpms = false;
        break;
      case 'h':
        print_usage();
        return -1;
      default:
        print_usage();
        return -1;
    }
  }

  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return -1;
  }

  bool keep_going = true;
  while (keep_going)
  {
    // Always call sn_update_data to refresh the internal cache of
    // flight control data
    int update_ret = sn_update_data();

    if (update_ret != 0)
    {
      printf("\nDetected likely failure in SN. Ensure it is running\n");
      keep_going = false;
    }
    else
    {
      static unsigned int cntr = 0;

      // Cycle through ESC IDs 0 to 3 for requesting feedback
      static int fb_id = 0;

      if (send_rpms)
      {
        // Initialize RPM commands to all zeros
        int rpms[4] = {0, 0, 0, 0};

        // ESC ID = cntr/100 gets 2000 RPM command
        rpms[cntr/100] = 2000;

        // Send the RPM commands and request feedback from ESC ID = fb_id
        sn_send_esc_rpm(rpms, 4, fb_id);
        printf("Sending RPMs = %d %d %d %d | fb_id = %d\n",
            rpms[0], rpms[1], rpms[2], rpms[3], fb_id);
      }
      else
      {
        // Initialize PWM commands to all zeros
        int pwms[4] = {0, 0, 0, 0};

        // ESC ID = cntr/100 gets 100 PWM command
        pwms[cntr/100] = 100;

        // Send the PWM commands and request feedback from ESC ID = fb_id
        sn_send_esc_pwm(pwms, 4, fb_id);
        printf("Sending PWMs = %d %d %d %d | fb_id = %d\n",
            pwms[0], pwms[1], pwms[2], pwms[3], fb_id);
      }

      // Verify that the flight controller is acknowledging the commands
      static unsigned int mode_not_correct_cntr = 0;
      if ((send_rpms && snav_data->general_status.current_mode != SN_ESC_RPM_MODE)
          || (!send_rpms && snav_data->general_status.current_mode != SN_ESC_PWM_MODE))
      {
        if (++mode_not_correct_cntr > 1)
        {
          printf("Flight control is not responding to commands, exiting.\n");
          keep_going = false;
        }
      }

      if (++cntr == 400) cntr = 0;
      if (++fb_id == 4) fb_id = 0;
    }
    usleep(10000);
  }

  return 0;
}

