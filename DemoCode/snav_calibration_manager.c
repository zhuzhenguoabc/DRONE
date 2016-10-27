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
  printf("-s  Attempt to run the on-ground accel offset calibration\n");
  printf("      a.k.a. static accel calibration\n");
  printf("-d  Attempt to run the in-flight accel offset and trim estimation\n");
  printf("      a.k.a. dynamic accel calibration\n");
  printf("      WARNING: This will attempt to pilot the vehicle\n");
  printf("-t  Attempt to run the thermal IMU calibration\n");
  printf("-o  Attempt to run the optic flow camera yaw calibration\n");
  printf("-m  Attempt to run the magnetometer calibration\n");
  printf("      a.k.a. compass calibration\n");
  printf("-v  Print version information.\n");
  printf("-h  Print this message\n");
}

typedef enum
{
  NO_CALIB,
  STATIC_ACCEL_CALIB,
  DYNAMIC_ACCEL_CALIB,
  THERMAL_IMU_CALIB,
  OPTIC_FLOW_CAM_CALIB,
  MAG_CALIB,
} CalibProcedure;

// State Machine for dynamic calibration
typedef enum
{
  ON_GROUND,
  STARTING_PROPS,
  TAKEOFF,
  LOITER,
  LANDING
} MissionState;

int main(int argc, char* argv[])
{
  int c;
  CalibProcedure calib = NO_CALIB;

  // Assume that vehicle is on ground when this program starts
  MissionState state = ON_GROUND;

  while ((c = getopt(argc, argv, "sdtomvh")) != -1)
  {
    switch(c)
    {
      case 's':
        calib = STATIC_ACCEL_CALIB;
        break;
      case 'd':
        calib = DYNAMIC_ACCEL_CALIB;
        break;
      case 't':
        calib = THERMAL_IMU_CALIB;
        break;
      case'o':
        calib = OPTIC_FLOW_CAM_CALIB;
        break;
      case 'm':
        calib = MAG_CALIB;
        break;
      case 'v':
        printf("v%s\n",VERSION);
        return -1;
      case'h':
        print_usage();
        return -1;
      default:
        print_usage();
        return -1;
    }
  }

  if (calib == NO_CALIB)
  {
    print_usage();
    return -1;
  }

  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return -1;
  }

  bool keep_going = true;
  bool calib_started = false;
  unsigned int attempt_number = 0;
  const unsigned int kMaxAttempts = 10;
  while (keep_going)
  {
    static unsigned int loop_counter = 0;

    int update_ret = sn_update_data();

    if(update_ret!=0)
    {
      printf("\nDetected likely failure in SN. Ensure it is running and attempt calibration call again.\n\n");
    }
    else
    {
      if (calib == STATIC_ACCEL_CALIB)
      {
        if (snav_data->general_status.current_mode == SN_STATIC_ACCEL_CALIBRATION_MODE)
        {
          calib_started = true;
          SnCalibStatus status;
          sn_get_static_accel_calibration_status(&status);
          if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
          {
            printf("[%u] Static accel calibration is in progress\n",
                loop_counter);
          }
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && calib_started)
        {
          printf("[%u] Static accel calibration was completed successfully\n",
              loop_counter);
          keep_going = false;
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
        {
          printf("[%u] Static accel calibration failed\n", loop_counter);
          keep_going = false;
        }
        else
        {
          if (attempt_number < kMaxAttempts)
          {
            printf("[%u] Sending command (attempt %u) to start static accel calibration\n",
                loop_counter, attempt_number);
            sn_start_static_accel_calibration();
            attempt_number++;
          }
          else
          {
            printf("[%u] Unable to start calibration\n", loop_counter);
            keep_going = false;
          }
        }
      }

      else if (calib == DYNAMIC_ACCEL_CALIB)
      {
        // Constant parameters for defining the flight sequence
        const float kTakeoffSpeed = 0.75;
        const float kLandingSpeed = -0.75;
        const float kDesTakeoffAlt = 1;

        // Commands that will be sent to flight control
        float cmd0 = 0;
        float cmd1 = 0;
        float cmd2 = 0;
        float cmd3 = 0;

        // Estimated z position at startup used to zero out z at takeoff
        static float z_est_startup;

        // Dynamic accel calibration requires flight, so use simple state
        // machine to track the progress
        if (state == ON_GROUND)
        {
          static unsigned int cntr = 0;
          if (++cntr > 20)
          {
            // delay a little bit before starting props
            state = STARTING_PROPS;
          }

          printf("[%u] Preparing to start propellers.\n", loop_counter);
        }

        else if (state == STARTING_PROPS)
        {
          if (snav_data->general_status.props_state == SN_PROPS_STATE_NOT_SPINNING)
          {
            z_est_startup = snav_data->high_level_control_data.position_estimated[2];
            sn_spin_props();
          }
          else if (snav_data->general_status.props_state == SN_PROPS_STATE_SPINNING)
          {
            state = TAKEOFF;
          }

          printf("[%u] Starting propellers.\n", loop_counter);
        }

        else if (state == TAKEOFF)
        {
          sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING, 0, 0, kTakeoffSpeed, 0, &cmd0, &cmd1, &cmd2, &cmd3);

          if (snav_data->high_level_control_data.position_desired[2] - z_est_startup > kDesTakeoffAlt)
          {
            state = LOITER;
          }

          printf("[%u] Taking off.\n", loop_counter);
        }

        else if (state == LOITER)
        {
          if (!calib_started)
          {
            // Delay the start of the calibration to let the vehicle settle
            // out a bit after taking off
            static unsigned int cntr = 0;
            if (++cntr > 20)
            {
              if (attempt_number < kMaxAttempts)
              {
                printf("[%u] Sending command (attempt %u) to start dynamic accel calibration\n",
                    loop_counter, attempt_number);
                sn_start_dynamic_accel_calibration();
                attempt_number++;
              }
              else
              {
                printf("[%u] Unable to start calibration\n", loop_counter);
                state = LANDING;
              }
            }
            else
            {
              printf("[%u] Waiting to start calibration.\n", loop_counter);
            }
          }

          SnCalibStatus status;
          sn_get_dynamic_accel_calibration_status(&status);
          if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
          {
            calib_started = true;
            printf("[%u] Dynamic accel calibration is in progress\n",
                loop_counter);
          }
          else
          {
            if (calib_started)
            {
              state = LANDING;
            }
          }
        }

        else if (state == LANDING)
        {
          if (snav_data->general_status.current_mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
          {
            if (snav_data->general_status.props_state == SN_PROPS_STATE_SPINNING)
            {
              // Command constant negative z velocity during landing
              sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING, 0, 0, kLandingSpeed, 0, &cmd0, &cmd1, &cmd2, &cmd3);

              if (snav_data->general_status.on_ground == 1)
              {
                // Snapdragon Navigator has determined that vehicle is on ground,
                // so it is safe to kill the propellers
                sn_stop_props();
              }
            }

            printf("[%u] Landing.\n",loop_counter);
          }
          else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && calib_started)
          {
            printf("[%u] Dynamic accel calibration was completed successfully\n",
                loop_counter);

            // Stall a bit before ending the program to keep printing the
            // result
            static unsigned int cntr = 0;
            if (++cntr > 100)
            {
              keep_going = false;
            }
          }
          else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
          {
            printf("[%u] Dynamic accel calibration failed\n",
                loop_counter);

            // Stall a bit before ending the program to keep printing the
            // result
            static unsigned int cntr = 0;
            if (++cntr > 100)
            {
              keep_going = false;
            }
          }
        }

        else
        {
          printf("Error: unknown mission state. Exiting.\n");
          keep_going = false;
        }

        sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING, cmd0, cmd1, cmd2, cmd3);
      }

      else if (calib == THERMAL_IMU_CALIB)
      {
        if (snav_data->general_status.current_mode == SN_THERMAL_IMU_CALIBRATION_MODE)
        {
          calib_started = true;
          SnCalibStatus status;
          sn_get_imu_thermal_calibration_status(&status);
          if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
          {
            printf("[%u] Thermal IMU calibration is in progress\n",
                loop_counter);
          }
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && calib_started)
        {
          printf("[%u] Thermal IMU calibration was completed successfully\n",
              loop_counter);
          keep_going = false;
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
        {
          printf("[%u] Thermal IMU calibration failed\n",
              loop_counter);
          keep_going = false;
        }
        else
        {
          if (attempt_number < kMaxAttempts)
          {
            printf("[%u] Sending command (attempt %u) to start thermal imu calibration\n",
                loop_counter, attempt_number);
            sn_start_imu_thermal_calibration();
            attempt_number++;
          }
          else
          {
            printf("[%u] Unable to start calibration\n",
                loop_counter);
            keep_going = false;
          }
        }
      }

      else if (calib == OPTIC_FLOW_CAM_CALIB)
      {
        if (snav_data->general_status.current_mode == SN_OPTIC_FLOW_CAM_YAW_CALIBRATION_MODE)
        {
          calib_started = true;
          SnCalibStatus status;
          sn_get_optic_flow_camera_yaw_calibration_status(&status);
          if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
          {
            printf("[%u] Optic flow camera yaw calibration is in progress\n",
                loop_counter);
          }
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && calib_started)
        {
          printf("[%u] Optic flow camera yaw calibration was completed successfully\n",
              loop_counter);
          keep_going = false;
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
        {
          printf("[%u] Optic flow camera yaw calibration failed\n",
              loop_counter);
          keep_going = false;
        }
        else
        {
          if (attempt_number < kMaxAttempts)
          {
            printf("[%u] Sending command (attempt %u) to start optic flow camera yaw calibration\n",
                loop_counter, attempt_number);
            sn_start_optic_flow_camera_yaw_calibration();
            attempt_number++;
          }
          else
          {
            printf("[%u] Unable to start calibration\n", loop_counter);
            keep_going = false;
          }
        }
      }

      else if (calib == MAG_CALIB)
      {
        if (snav_data->general_status.current_mode == SN_MAGNETOMETER_CALIBRATION_MODE)
        {
          calib_started = true;
          SnCalibStatus status;
          sn_get_magnetometer_calibration_status(&status);
          if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
          {
            printf("[%u] Magnetometer calibration is in progress\n",
                loop_counter);
          }
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && calib_started)
        {
          printf("[%u] Magnetometer calibration was completed successfully\n",
              loop_counter);
          keep_going = false;
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
        {
          printf("[%u] Magnetometer calibration failed\n", loop_counter);
          keep_going = false;
        }
        else
        {
          if (attempt_number < kMaxAttempts)
          {
            printf("[%u] Sending command (attempt %u) to start magnetometer calibration\n",
                loop_counter, attempt_number);
            sn_start_magnetometer_calibration();
            attempt_number++;
          }
          else
          {
            printf("[%u] Unable to start calibration\n", loop_counter);
            keep_going = false;
          }
        }
      }

      else
      {
        keep_going = false;
      }
    }
    loop_counter++;
    usleep(100000);
  }

  // Call sync to make sure any calibration files get written to disk
  system("sync");

  return 0;
}

