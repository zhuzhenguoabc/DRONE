/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
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
  printf("-r  Attempt to run the thermal IMU calibration with specified\n");
  printf("      temperature ramp rate.  Units are C/min.\n");
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

// Return codes for this program
// Errors start with 3 because 1 and 2 have special meanings in bash
typedef enum
{
  RESULT_CALIBRATION_SUCCESS = 0,         /**< Calibration completed successfully. */
  RESULT_CALIBRATION_FAILURE = 3,         /**< Calibration started but did not complete successfully. */
  RESULT_CALIBRATION_UNABLE_TO_START = 4, /**< Calibration was unable to start. */
  RESULT_NO_CALIBRATION_SELECTED = 5,     /**< No calibration was selected. */
  RESULT_SNAV_CACHED_DATA_ERROR = 6,      /**< Error getting pointer to SnavCachedData struct. */
  RESULT_SNAV_COMM_ERROR = 7,             /**< Error communicating with SNAV. */
} ProgramResult;

// IMU temperature calibration controller
#define   TEMP_CTRL_NUM_THREADS 4
int       temp_ctrl_init();
void      temp_ctrl_run(SnavCachedData* snav_data);
void*     temp_ctrl_load_cpu(void* p_idx);
uint64_t  get_time_us();
float     temp_ctrl_pwm;
float     temp_ctrl_rate;
int       thread_idx[TEMP_CTRL_NUM_THREADS]; // Must be global scope.
pthread_t threads[TEMP_CTRL_NUM_THREADS];

int main(int argc, char* argv[])
{
  int c;
  CalibProcedure calib = NO_CALIB;

  // Assume that vehicle is on ground when this program starts
  MissionState state = ON_GROUND;

  while ((c = getopt(argc, argv, "sdtomvhr:")) != -1)
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
        temp_ctrl_rate = 0;
        break;
      case 'r':
        calib = THERMAL_IMU_CALIB;
        temp_ctrl_rate = atof(optarg);
        break;
      case'o':
        calib = OPTIC_FLOW_CAM_CALIB;
        break;
      case 'm':
        calib = MAG_CALIB;
        break;
      case 'v':
        printf("v%s\n",VERSION);
        return RESULT_NO_CALIBRATION_SELECTED;
      case'h':
        print_usage();
        return RESULT_NO_CALIBRATION_SELECTED;
      default:
        print_usage();
        return RESULT_NO_CALIBRATION_SELECTED;
    }
  }

  if (calib == NO_CALIB)
  {
    print_usage();
    return RESULT_NO_CALIBRATION_SELECTED;
  }

  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return RESULT_SNAV_CACHED_DATA_ERROR;
  }

  bool keep_going = true;
  bool calib_started = false;
  unsigned int attempt_number = 0;
  const unsigned int kMaxAttempts = 10;
  ProgramResult program_result = RESULT_CALIBRATION_UNABLE_TO_START;
  while (keep_going)
  {
    static unsigned int loop_counter = 0;

    int update_ret = sn_update_data();

    if(update_ret!=0)
    {
      printf("\nDetected likely failure in SN. Ensure it is running and attempt calibration call again.\n\n");
      keep_going = false;
      program_result = RESULT_SNAV_COMM_ERROR;
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
          program_result = RESULT_CALIBRATION_SUCCESS;
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
        {
          printf("[%u] Static accel calibration failed\n", loop_counter);
          keep_going = false;
          program_result = RESULT_CALIBRATION_FAILURE;
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
            program_result = RESULT_CALIBRATION_UNABLE_TO_START;
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

          static unsigned int loop_counter_initial = 0;
          if (loop_counter_initial == 0)
          {
            loop_counter_initial = loop_counter;
          }

          if (loop_counter - loop_counter_initial > 50)
          {
            printf("[%u] Unable to start propellers.\n", loop_counter);
            keep_going = false;
            program_result = RESULT_CALIBRATION_UNABLE_TO_START;
          }
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
              program_result = RESULT_CALIBRATION_SUCCESS;
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
              program_result = RESULT_CALIBRATION_FAILURE;
            }
          }
        }

        else
        {
          printf("Error: unknown mission state. Exiting.\n");
          keep_going = false;
          program_result = RESULT_CALIBRATION_FAILURE;
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
            temp_ctrl_run(snav_data);
            printf("[%u] Thermal IMU calibration is in progress (current temp = %6.2f C).\n",
                   loop_counter,snav_data->imu_0_raw.temp);
          }
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_SUCCESS && calib_started)
        {
          printf("[%u] Thermal IMU calibration was completed successfully\n",
              loop_counter);
          keep_going = false;
          program_result = RESULT_CALIBRATION_SUCCESS;
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
        {
          printf("[%u] Thermal IMU calibration failed\n",
              loop_counter);
          keep_going = false;
          program_result = RESULT_CALIBRATION_FAILURE;
        }
        else
        {
          if (attempt_number == 0)
          {
            int temp_ctrl_init_ret = temp_ctrl_init();
            if (temp_ctrl_init_ret != 0)
            {
              keep_going = false;
              program_result = RESULT_CALIBRATION_UNABLE_TO_START;
            }
          }
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
            program_result = RESULT_CALIBRATION_UNABLE_TO_START;
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
          program_result = RESULT_CALIBRATION_SUCCESS;
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
        {
          printf("[%u] Optic flow camera yaw calibration failed\n",
              loop_counter);
          keep_going = false;
          program_result = RESULT_CALIBRATION_FAILURE;
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
            program_result = RESULT_CALIBRATION_UNABLE_TO_START;
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
          program_result = RESULT_CALIBRATION_SUCCESS;
        }
        else if (snav_data->general_status.current_mode == SN_CALIBRATION_FAILURE && calib_started)
        {
          printf("[%u] Magnetometer calibration failed\n", loop_counter);
          keep_going = false;
          program_result = RESULT_CALIBRATION_FAILURE;
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
            program_result = RESULT_CALIBRATION_UNABLE_TO_START;
          }
        }
      }

      else
      {
        keep_going = false;
        program_result = RESULT_NO_CALIBRATION_SELECTED;
      }
    }
    loop_counter++;
    usleep(100000);
  }

  // Call sync to make sure any calibration files get written to disk
  system("sync");

  return program_result;
}

int temp_ctrl_init()
{
  // legacy behavior if 0 rate is specified.
  if (temp_ctrl_rate == 0)
  {
    return 0;
  }

  // init pwm and spawn the loading threads
  temp_ctrl_pwm = 0;
  int result;
  int i;
  for (i = 0; i < TEMP_CTRL_NUM_THREADS; ++i)
  {
    thread_idx[i] = i;
    result = pthread_create(&threads[i], NULL, temp_ctrl_load_cpu, &thread_idx[i]);
    if (result != 0)
    {
      printf("Error creating thread.\n");
      return -1;
    }
  }

  // preheat for 5 mins if negative rate is specified
  if (temp_ctrl_rate < 0)
  {
    printf("Negative temp rate specified, preheating vehicle for 5 mins.\n");
    temp_ctrl_pwm = 100.0 * TEMP_CTRL_NUM_THREADS;
    usleep(5 * 60 * 1000000);
  }

  return 0;
}

void temp_ctrl_run(SnavCachedData* snav_data)
{
  static bool initialized = false;
  static float desired_temp;
  static uint64_t last_update_time_us;
  static FILE* fp = NULL;

  // legacy behavior if 0 rate is specified.
  if (temp_ctrl_rate == 0)
  {
    return;
  }

  if (initialized == false)
  {
    desired_temp = snav_data->imu_0_raw.temp;
    last_update_time_us = get_time_us();
    fp = fopen("/var/log/snav/calib/temp_cal.log","w");
    initialized = true;
  }
  else
  {
    const float kf = 4.0;
    const float kp = 200.0;
    const float nominal_temp  = 25.0;

    uint64_t delta_time_us = get_time_us() - last_update_time_us;
    desired_temp += temp_ctrl_rate/60.0 * (delta_time_us / 1000000.0);
    desired_temp  = fmin(desired_temp, 75.0);

    float ffd_ctrl = kf * (desired_temp - nominal_temp);
    float prp_ctrl = kp * (desired_temp - snav_data->imu_0_raw.temp);

    temp_ctrl_pwm = prp_ctrl + ffd_ctrl;
    temp_ctrl_pwm = fmin(fmax(temp_ctrl_pwm, 0), TEMP_CTRL_NUM_THREADS * 100.0);

    // tag current time for next update
    last_update_time_us = get_time_us();
  }

  // log data to file
  if (fp != NULL)
  {
    fprintf(fp,
            "%8.6f %6.2f %6.2f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f\n",
            (float)snav_data->imu_0_raw.time / 1000000.0,
            snav_data->imu_0_raw.temp,
            temp_ctrl_pwm,
            snav_data->imu_0_raw.lin_acc[0],
            snav_data->imu_0_raw.lin_acc[1],
            snav_data->imu_0_raw.lin_acc[2],
            snav_data->imu_0_raw.ang_vel[0],
            snav_data->imu_0_raw.ang_vel[1],
            snav_data->imu_0_raw.ang_vel[2]);
    fflush(fp);
  }
}

void* temp_ctrl_load_cpu(void* p_idx)
{
  float duty_cycle;
  int   thread_idx = *((int *)p_idx);
  const float pwm_period_us = 1000000.0;

  while (1)
  {
    duty_cycle = temp_ctrl_pwm - (thread_idx * 100.0);
    duty_cycle = fmin(fmax(duty_cycle, 0), 100.0)/100.0;

    uint64_t run_start = get_time_us();
    while (get_time_us() - run_start < pwm_period_us)
    {
      uint64_t busy_start = get_time_us();

      int i = 0;
      while (get_time_us() - busy_start < duty_cycle * pwm_period_us)
        ++i;

      usleep((useconds_t)((1.0 - duty_cycle) * pwm_period_us));
    }
  }

  return NULL;
}

uint64_t get_time_us()
{
  struct timespec time_struct;
  clock_gettime(0,&time_struct);
  return ((time_struct.tv_nsec)/1000 + ((uint64_t)time_struct.tv_sec)*1000000.0);
}
