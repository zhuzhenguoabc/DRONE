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
  printf("-h  Print this message\n");
}

int main(int argc, char* argv[])
{
  int c;

  while ((c = getopt(argc, argv, "rph")) != -1)
  {
    switch (c)
    {
      case 'h':
        print_usage();
        return -1;
      default:
        break;
    }
  }


  unsigned int cntr = 0;
  bool keep_going = true;
  uint8_t led_colors[3] = {0,0,0};  //R, G, B

  int32_t timeout_us = 1000000; //timeout for flight controller to take over LED control after API commands stop


  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return -1;
  }

  while (keep_going)
  {
    int update_ret = sn_update_data();  //need to do this at least once to make sure that SNAV is running

    if (update_ret != 0)
    {
      printf("\nDetected likely failure in SN. Ensure it is running\n");
      break;
    }

    int cntr2 = cntr % 400;

    if      (cntr2 < 100) { led_colors[0] = 0;   led_colors[1] = 0;   led_colors[2] = 0;   }
    else if (cntr2 < 200) { led_colors[0] = 255; led_colors[1] = 0;   led_colors[2] = 0;   }
    else if (cntr2 < 300) { led_colors[0] = 0;   led_colors[1] = 255; led_colors[2] = 0;   }
    else if (cntr2 < 400) { led_colors[0] = 0;   led_colors[1] = 0;   led_colors[2] = 255; }

    int ret = sn_set_led_colors(led_colors, sizeof(led_colors), timeout_us);
    if (ret != 0)
    {
      printf("\nsn_set_led_colors returned %d\n",ret);
    }

    usleep(10000);  //note that commands should only be sent as often as needed (minimize message traffic)
    cntr++;
  }

  return 0;
}

