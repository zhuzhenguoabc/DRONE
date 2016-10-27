/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "snapdragon_navigator.h"

void print_sn_calib_status_string(SnCalibStatus status)
{
  if (status == SN_CALIB_STATUS_NOT_CALIBRATED)
    printf("SN_CALIB_STATUS_NOT_CALIBRATED\n");
  else if (status == SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS)
    printf("SN_CALIB_STATUS_CALIBRATION_IN_PROGRESS\n");
  else if (status == SN_CALIB_STATUS_CALIBRATED)
    printf("SN_CALIB_STATUS_CALIBRATED\n");
  else
    printf("invalid code\n");
}

void print_sn_data_status_string(SnDataStatus status)
{

  if (status == SN_DATA_UNCALIBRATED)
    printf("SN_DATA_UNCALIBRATED\n");
  else if (status == SN_DATA_WARNING)
    printf("SN_DATA_WARNING\n");
  else if (status == SN_DATA_VALID)
    printf("SN_DATA_VALID\n");
  else
    printf("SN_DATA_INVALID\n");

}

int main(int argc, char* argv[])
{
  printf("\nExecuting SN API Test Application\n");

  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return -1;
  }

  // put vehicle into infinite loop
  int loop_counter = 0;
  unsigned int ii = 0;
  for(loop_counter=0;;loop_counter++)
  {
    // IMPORTANT update the current state. This should be done once per control loop
    int update_ret = sn_update_data();

    if(update_ret!=0)
    {
      printf("Detected likely failure in SN. Ensure it is running.\n");
    }
    else
    {
      // Print some information about the different commands
      // This will only be done once since it is fixed
      static unsigned int printed_command_info = 0;
      if (printed_command_info == 0)
      {
        const char* cmd_name[SN_RC_NUM_CMD_TYPES];
        SnRcCommandType rc_cmd_type[SN_RC_NUM_CMD_TYPES]
          = { SN_RC_RATES_CMD, SN_RC_THRUST_ANGLE_CMD, SN_RC_ALT_HOLD_CMD,
            SN_RC_GPS_POS_HOLD_CMD, SN_RC_OPTIC_FLOW_POS_HOLD_CMD,
            SN_RC_VIO_POS_HOLD_CMD };

        for (ii = 0; ii < SN_RC_NUM_CMD_TYPES; ++ii)
        {
          cmd_name[ii] = sn_get_cmd_name(rc_cmd_type[ii]);
          printf("%s\n",cmd_name[ii]);

          const char* units[4];
          float min[4];
          float max[4];
          unsigned int jj = 0;
          for (jj = 0; jj < 4; ++jj)
          {
            units[jj] = sn_get_dimensioned_units(rc_cmd_type[ii], jj);
            min[jj] = sn_get_min_value(rc_cmd_type[ii], jj);
            max[jj] = sn_get_max_value(rc_cmd_type[ii], jj);
            printf("cmd%d has range [%f %s, %f %s]\n", jj, min[jj], units[jj],
                max[jj], units[jj]);
          }
        }

        printf("Live view starting in ");
        for (ii = 10; ii > 0; --ii)
        {
          printf("%d...",ii);
          fflush(stdout);
          usleep(1000000);
        }
        printf("\n");

        printed_command_info = 1;
      }

      printf("------------------------------------\n");
      // Read in current mode
      SnMode mode = (SnMode) snav_data->general_status.current_mode;
      if (mode == SN_SENSOR_ERROR_MODE)
        printf("mode = SN_SENSOR_ERROR_MODE\n");
      else if (mode == SN_WAITING_FOR_DEVICE_TO_CONNECT)
        printf("mode = SN_WAITING_FOR_DEVICE_TO_CONNECT\n");
      else if (mode == SN_EMERGENCY_KILL_MODE)
        printf("mode = SN_EMERGENCY_KILL_MODE\n");
      else if (mode == SN_EMERGENCY_LANDING_MODE)
        printf("mode = SN_EMERGENCY_LANDING_MODE\n");
      else if (mode == SN_THERMAL_IMU_CALIBRATION_MODE)
        printf("mode = SN_THERMAL_IMU_CALIBRATION_MODE\n");
      else if (mode == SN_STATIC_ACCEL_CALIBRATION_MODE)
        printf("mode = SN_STATIC_ACCEL_CALIBRATION_MODE\n");
      else if (mode == SN_OPTIC_FLOW_CAM_YAW_CALIBRATION_MODE)
        printf("mode = SN_OPTIC_FLOW_CAM_YAW_CALIBRATION_MODE\n");
      else if (mode == SN_MAGNETOMETER_CALIBRATION_MODE)
        printf("mode = SN_MAGNETOMETER_CALIBRATION_MODE\n");
      else if (mode == SN_CALIBRATION_SUCCESS)
        printf("mode = SN_CALIBRATION_SUCCESS\n");
      else if (mode == SN_CALIBRATION_FAILURE)
        printf("mode = SN_CALIBRATION_FAILURE\n");
      else if (mode == SN_ESC_RPM_MODE)
        printf("mode = SN_ESC_RPM_MODE\n");
      else if (mode == SN_ESC_PWM_MODE)
        printf("mode = SN_ESC_PWM_MODE\n");
      else if (mode == SN_RATE_MODE)
        printf("mode = SN_RATE_MODE\n");
      else if (mode == SN_THRUST_ANGLE_MODE)
        printf("mode = SN_THRUST_ANGLE_MODE\n");
      else if (mode == SN_ALT_HOLD_MODE)
        printf("mode = SN_ALT_HOLD_MODE\n");
      else if (mode == SN_THRUST_GPS_HOVER_MODE)
        printf("mode = SN_THRUST_GPS_HOVER_MODE\n");
      else if (mode == SN_GPS_POS_HOLD_MODE)
        printf("mode = SN_GPS_POS_HOLD_MODE\n");
      else if (mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
        printf("mode = SN_OPTIC_FLOW_POS_HOLD_MODE\n");
      else if (mode == SN_VIO_POS_HOLD_MODE)
        printf("mode = SN_VIO_POS_HOLD_MODE\n");
      else
        printf("mode = SN_UNDEFINED_MODE\n");

      // Read in current state of propellers
      SnPropsState props_state = (SnPropsState) snav_data->general_status.props_state;
      if (props_state == SN_PROPS_STATE_NOT_SPINNING)
        printf("props_state = SN_PROPS_STATE_NOT_SPINNING\n");
      else if (props_state == SN_PROPS_STATE_STARTING)
        printf("props_state = SN_PROPS_STATE_STARTING\n");
      else if (props_state == SN_PROPS_STATE_SPINNING)
        printf("props_state = SN_PROPS_STATE_SPINNING\n");
      else
        printf("props_state = SN_PROPS_STATE_UNKNOWN\n");

      // Get the on ground flag
      if (snav_data->general_status.on_ground == 0)
        printf("flight control thinks vehicle is NOT ON GROUND\n");
      else
        printf("flight control thinks vehicle is ON GROUND\n");

      // Get the battery voltage
      printf("voltage = %f\n", snav_data->general_status.voltage);

      // Get the status of the IMU
      SnDataStatus imu_status = (SnDataStatus) snav_data->data_status.imu_0_status;
      printf("imu_status = ");
      print_sn_data_status_string(imu_status);

      if (imu_status == SN_DATA_VALID)
      {
        // Get the temp of the IMU
        printf("imu_temp = %f\n", snav_data->imu_0_raw.temp);

        // Get the lin acc from IMU
        printf("lin_acc = %f %f %f\n", snav_data->imu_0_compensated.lin_acc[0],
            snav_data->imu_0_compensated.lin_acc[1],
            snav_data->imu_0_compensated.lin_acc[2]);

        // Get the ang vel from IMU
        printf("ang_vel = %f %f %f\n", snav_data->imu_0_compensated.ang_vel[0],
            snav_data->imu_0_compensated.ang_vel[1],
            snav_data->imu_0_compensated.ang_vel[2]);
      }

      // Get the baro status
      SnDataStatus baro_status = (SnDataStatus) snav_data->data_status.baro_0_status;
      printf("baro_status = ");
      print_sn_data_status_string(baro_status);

      if (baro_status == SN_DATA_VALID)
      {
        // Get the baro data
        printf("pressure = %f, baro temp = %f\n", snav_data->barometer_0_raw.pressure,
            snav_data->barometer_0_raw.temp);
      }

      // Get the RC status
      SnDataStatus rc_status = (SnDataStatus) snav_data->data_status.spektrum_rc_0_status;
      printf("rc_status = ");
      print_sn_data_status_string(rc_status);

      if (rc_status == SN_DATA_VALID)
      {
        // Get the RC data
        for (ii = 0; ii < snav_data->spektrum_rc_0_raw.num_channels; ++ii)
        {
          printf("%u ",snav_data->spektrum_rc_0_raw.vals[ii]);
        }
        printf("\n");
      }

      // Get the mag status
      SnDataStatus mag_status = (SnDataStatus) snav_data->data_status.mag_0_status;
      printf("mag_status = ");
      print_sn_data_status_string(mag_status);

      // Get the GPS status
      SnDataStatus gps_status = (SnDataStatus) snav_data->data_status.gps_0_status;
      printf("gps_status = ");
      print_sn_data_status_string(gps_status);

      // Get the sonar status
      SnDataStatus sonar_status = (SnDataStatus) snav_data->data_status.sonar_0_status;
      printf("sonar_status = ");
      print_sn_data_status_string(sonar_status);

      if (sonar_status == SN_DATA_VALID)
      {
        // Get the sonar data
        printf("sonar range = %f\n", snav_data->sonar_0_raw.range);
      }

      // Get the optic flow status
      SnDataStatus optic_flow_status = (SnDataStatus) snav_data->data_status.optic_flow_0_status;
      printf("optic_flow_status = ");
      print_sn_data_status_string(optic_flow_status);

      if (optic_flow_status == SN_DATA_VALID)
      {
        // Get optic flow sample size
        printf("optic_flow_sample_size = %d\n", snav_data->optic_flow_0_raw.sample_size);
      }

      // Get control loop frequency
      printf("frequency = %f\n", snav_data->update_rates.control_loop_freq);

      // Read in current state estimate
      printf("xyz_est | yaw_est = %f, %f, %f | %f\n",
          snav_data->high_level_control_data.position_estimated[0],
          snav_data->high_level_control_data.position_estimated[1],
          snav_data->high_level_control_data.position_estimated[2],
          snav_data->high_level_control_data.yaw_estimated);

      // Read in current desired state
      printf("xyz_des | yaw_des = %f, %f, %f | %f\n",
          snav_data->high_level_control_data.position_desired[0],
          snav_data->high_level_control_data.position_desired[1],
          snav_data->high_level_control_data.position_desired[2],
          snav_data->high_level_control_data.yaw_desired);

      // Read in current rotation estimate
      float qw,qx,qy,qz;
      sn_get_quat_est(&qw,&qx,&qy,&qz);
      printf("quat_est = %f %f %f %f\n",qw, qx, qy, qz);

      printf("rot_est = %f %f %f %f %f %f %f %f %f\n",
          snav_data->attitude_estimate.rotation_matrix[0],
          snav_data->attitude_estimate.rotation_matrix[1],
          snav_data->attitude_estimate.rotation_matrix[2],
          snav_data->attitude_estimate.rotation_matrix[3],
          snav_data->attitude_estimate.rotation_matrix[4],
          snav_data->attitude_estimate.rotation_matrix[5],
          snav_data->attitude_estimate.rotation_matrix[6],
          snav_data->attitude_estimate.rotation_matrix[7],
          snav_data->attitude_estimate.rotation_matrix[8]);

      printf("rpy = %f %f %f\n",
          snav_data->attitude_estimate.roll,
          snav_data->attitude_estimate.pitch,
          snav_data->attitude_estimate.yaw);

      // Get ESC feedback
      int number_of_escs = 4;
      printf("esc_sw_versions = ");
      for (ii = 0; ii < number_of_escs; ++ii)
      {
        printf("%d ", snav_data->version_info.esc_sw_version[ii]);
      }
      printf("\n");

      printf("esc_hw_versions = ");
      for (ii = 0; ii < number_of_escs; ++ii)
      {
        printf("%d ", snav_data->version_info.esc_hw_version[ii]);
      }
      printf("\n");

      printf("esc_packet_cntr_fb = ");
      for (ii = 0; ii < number_of_escs; ++ii)
      {
        printf("%d ", (int) snav_data->esc_raw.packet_cntr[ii]);
      }
      printf("\n");

      printf("esc_rpm_fb = ");
      for (ii = 0; ii < number_of_escs; ++ii)
      {
        printf("%d ", snav_data->esc_raw.rpm[ii]);
      }
      printf("\n");

      printf("esc_power_fb = ");
      for (ii = 0; ii < number_of_escs; ++ii)
      {
        printf("%d ", (int) snav_data->esc_raw.power[ii]);
      }
      printf("\n");

      printf("esc_voltage_fb = ");
      for (ii = 0; ii < number_of_escs; ++ii)
      {
        printf("%f ", snav_data->esc_raw.voltage[ii]);
      }
      printf("\n");

      /* Note that this data will be included in snav_cached_data in future releases */
      unsigned int fb_used;
      int esc_sw_versions[number_of_escs];
      sn_get_esc_sw_versions(esc_sw_versions, number_of_escs, &fb_used);
      SnMotorState state_feedback[number_of_escs];
      sn_get_esc_state_feedback(state_feedback, number_of_escs, &fb_used);
      printf("esc_state_fb = ");
      for (ii = 0; ii < fb_used; ++ii)
      {
        if (state_feedback[ii] == SN_MOTOR_STATE_NOT_SPINNING)
          printf("SN_MOTOR_STATE_NOT_SPINNING ");
        else if (state_feedback[ii] == SN_MOTOR_STATE_STARTING)
          printf("SN_MOTOR_STATE_STARTING ");
        else if (state_feedback[ii] == SN_MOTOR_STATE_SPINNING_FORWARD)
          printf("SN_MOTOR_STATE_SPINNING_FORWARD ");
        else if (state_feedback[ii] == SN_MOTOR_STATE_SPINNING_BACKWARD)
          printf("SN_MOTOR_STATE_SPINNING_BACKWARD ");
        else
          printf("SN_MOTOR_STATE_UNKNOWN ");
      }
      printf("\n");

      // Get static accel calib status
      SnCalibStatus static_accel_calib_status;
      sn_get_static_accel_calibration_status(&static_accel_calib_status);
      printf("static_accel_calib_status = ");
      print_sn_calib_status_string(static_accel_calib_status);

      // Get dynamic accel calib status
      SnCalibStatus dynamic_accel_calib_status;
      sn_get_dynamic_accel_calibration_status(&dynamic_accel_calib_status);
      printf("dynamic_accel_calib_status = ");
      print_sn_calib_status_string(dynamic_accel_calib_status);

      // Get thermal imu calib status
      SnCalibStatus thermal_imu_calib_status;
      sn_get_imu_thermal_calibration_status(&thermal_imu_calib_status);
      printf("thermal_imu_calib_status = ");
      print_sn_calib_status_string(thermal_imu_calib_status);

      // Get optic flow cam yaw calib status
      SnCalibStatus optic_flow_cam_yaw_calib_status;
      sn_get_optic_flow_camera_yaw_calibration_status(&optic_flow_cam_yaw_calib_status);
      printf("optic_flow_cam_yaw_calib_status = ");
      print_sn_calib_status_string(optic_flow_cam_yaw_calib_status);

      // Get magnetometer calib status
      SnCalibStatus mag_calib_status;
      sn_get_magnetometer_calibration_status(&mag_calib_status);
      printf("mag_calib_status = ");
      print_sn_calib_status_string(mag_calib_status);
    }

    //Sleep until next loop
    usleep(100000);
  }

  return 0;
}
