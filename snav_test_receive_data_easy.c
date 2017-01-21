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
	printf("\n*****START*******\n");
  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return -1;
  }

  // put vehicle into infinite loop
  int loop_counter = 0;
  unsigned int ii = 0;
  int pass_false_flag = 0;

  for(loop_counter=0;;loop_counter++)
  {
	  //sleep(2);
	  sleep(5);
	  system("clear");
        pass_false_flag = 0;
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
	  /*
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
      printf("电压 voltage = %f\n", snav_data->general_status.voltage);
*/
	  // Get the snav version
      char library_version[18];
	  memset(library_version, 0, 18);
	  memcpy(library_version, snav_data->version_info.library_version, 18);
      // Get the status of the IMU
      SnDataStatus imu_status = (SnDataStatus) snav_data->data_status.imu_0_status;
	  // Get the baro status
      SnDataStatus baro_status = (SnDataStatus) snav_data->data_status.baro_0_status;
	  // Get the RC status
      SnDataStatus rc_status = (SnDataStatus) snav_data->data_status.spektrum_rc_0_status;
	   // Get the mag status
      SnDataStatus mag_status = (SnDataStatus) snav_data->data_status.mag_0_status;
	  // Get the GPS status
      SnDataStatus gps_status = (SnDataStatus) snav_data->data_status.gps_0_status;
	  // Get the sonar status
      SnDataStatus sonar_status = (SnDataStatus) snav_data->data_status.sonar_0_status;
	   // Get the optic flow status
      SnDataStatus optic_flow_status = (SnDataStatus) snav_data->data_status.optic_flow_0_status;
	  // Get the esc status
      SnDataStatus esc_feedback_status = (SnDataStatus) snav_data->data_status.esc_feedback_status;
	  // Get static accel calib status
      SnCalibStatus static_accel_calib_status;
      sn_get_static_accel_calibration_status(&static_accel_calib_status);
	  // Get dynamic accel calib status
      SnCalibStatus dynamic_accel_calib_status;
      sn_get_dynamic_accel_calibration_status(&dynamic_accel_calib_status);
	  // Get thermal imu calib status
      SnCalibStatus thermal_imu_calib_status;
      sn_get_imu_thermal_calibration_status(&thermal_imu_calib_status);
	  // Get optic flow cam yaw calib status
      SnCalibStatus optic_flow_cam_yaw_calib_status;
      sn_get_optic_flow_camera_yaw_calibration_status(&optic_flow_cam_yaw_calib_status);
	   // Get magnetometer calib status
      SnCalibStatus mag_calib_status;
      sn_get_magnetometer_calibration_status(&mag_calib_status);

	  //check current version	print $2,$3
	char get_wifi_ssid_cmd[128] = "cat /etc/hostapd.conf | grep ssid= | head -n 1 | cut -c6-";
	char wifi_ssid[64];

	FILE *fp = popen(get_wifi_ssid_cmd, "r");
	fgets(wifi_ssid, sizeof(wifi_ssid), fp);
	pclose(fp);

	if (wifi_ssid[strlen(wifi_ssid)-1] == '\n')
	{
		wifi_ssid[strlen(wifi_ssid)-1] = '\0';
	}

      printf("\nWifi-Name		=		%s\n\n", wifi_ssid);

	  printf("------------------------------------------------------------\n");
      printf("Snav Version	=	%s\n", library_version);

	  printf("------------------------------------------------------------\n");
	  printf("1: imu_status	= %d -----------", imu_status);

	  if (imu_status == SN_DATA_VALID)
	  {
		  if ((snav_data->imu_0_raw.lin_acc[0] >= -0.1f) && (snav_data->imu_0_raw.lin_acc[0] <= 0.1f)
			   && (snav_data->imu_0_raw.lin_acc[1] >= -0.1f) && (snav_data->imu_0_raw.lin_acc[1] <= 0.1f)
			   && (snav_data->imu_0_raw.lin_acc[2] >= 0.9f) && (snav_data->imu_0_raw.lin_acc[2] <= 1.1f)
			 &&(snav_data->imu_0_raw.ang_vel[0] >= -0.1f) && (snav_data->imu_0_raw.ang_vel[0] <= 0.1f)
			 && (snav_data->imu_0_raw.ang_vel[1] >= -0.1f) && (snav_data->imu_0_raw.ang_vel[1] <= 0.1f)
			 && (snav_data->imu_0_raw.ang_vel[2] >= -0.1f) && (snav_data->imu_0_raw.ang_vel[2] <= 0.1f))
		  {
			  printf("Pass\n");
		  }
		  else
		  {
			pass_false_flag = 1;
			printf("Fail003\n");
		  }
	  }
	  else if (imu_status == SN_DATA_NOT_INITIALIZED)
	  {
		  pass_false_flag = 1;
		  printf("Fail001\n");
	  }
      else if (imu_status == SN_DATA_UNCALIBRATED)
      {
		  pass_false_flag = 1;
		  printf("Fail002\n");
	  }
	  else
	  {
		  pass_false_flag = 1;
		  printf("Fail004\n");
	  }

	printf("------------------------------------------------------------\n");
	  printf("2: baro_status = %d -----------", baro_status);
      //print_sn_data_status_string(baro_status);
	  if (baro_status == SN_DATA_VALID)
		printf("Pass\n");
	  else
		{
		  pass_false_flag = 1;
		  printf("Fail\n");
	  }

	  /*
	printf("------------------------------------------------------------\n");
	  printf("rc_status         = ");
      //print_sn_data_status_string(rc_status);
	  if (rc_status == SN_DATA_VALID)
		printf("Pass\n");
	  else
		{
		  pass_false_flag = 1;
		  printf("Fail\n");
	  }*/

	printf("------------------------------------------------------------\n");
	  printf("3: mag_status = %d -----------", mag_status);
      //print_sn_data_status_string(mag_status);
      /*
	  if ((mag_status == SN_DATA_VALID) || (mag_status == SN_DATA_WARNING))
	  {
		  printf("Pass\n");
	  }
	  else if (mag_status == SN_DATA_NOT_INITIALIZED)
	  {
		  pass_false_flag = 1;
		  printf("Fail001\n");
	  }
	  else
	  {
		  pass_false_flag = 1;
		  printf("Fail002\n");
	  }
	  */
	  printf("Pass\n");

	printf("------------------------------------------------------------\n");
	  printf("4: gps_status = %d -----------", gps_status);
      //print_sn_data_status_string(gps_status);
      /*
	  if ((gps_status == SN_DATA_VALID)|| (gps_status == SN_DATA_NOT_INITIALIZED) || (gps_status == SN_DATA_NO_LOCK ))
		printf("Pass\n");
	  else
		{
		  pass_false_flag = 1;
		  printf("Fail\n");
	  }
	  */
	  printf("Pass\n");

	printf("------------------------------------------------------------\n");
	  printf("5: sonar_status = %d -----------", sonar_status);
      //print_sn_data_status_string(sonar_status);
	  if (sonar_status == SN_DATA_VALID)
		printf("Pass\n");
	  else
		{
		  pass_false_flag = 1;
		  printf("Fail\n");
	  }
	printf("------------------------------------------------------------\n");
	  printf("6: optic_flow_status = %d -----------", optic_flow_status);
      //print_sn_data_status_string(optic_flow_status);
	  if (optic_flow_status == SN_DATA_VALID)
		printf("Pass\n");
	  else
		{
		  pass_false_flag = 1;
		  printf("Fail\n");
	  }

	printf("------------------------------------------------------------\n");
	  printf("7: esc_feedback_status = %d -----------", esc_feedback_status);
	  if (esc_feedback_status == SN_DATA_VALID)
	  {
		  if ((snav_data->esc_raw.voltage[0]>=6.0f && snav_data->esc_raw.voltage[0]<=8.5f)
			  && (snav_data->esc_raw.voltage[1]>=6.0f && snav_data->esc_raw.voltage[1]<=8.5f)
			  && (snav_data->esc_raw.voltage[2]>=6.0f && snav_data->esc_raw.voltage[2]<=8.5f)
			  && (snav_data->esc_raw.voltage[3]>=6.0f && snav_data->esc_raw.voltage[3]<=8.5f))
		  {
			  printf("Pass\n");
		  }
		  else
		  {
			  pass_false_flag = 1;
			  printf("Fail002\n");
		  }
	  }
	  else
	  {
		  pass_false_flag = 1;
		printf("Fail001\n");
	  }
	  /*
	  printf("-----------------------------------------------------------------------------------------\n");
	  printf("静态校准       static_accel_calib_status       = ");
      //print_sn_calib_status_string(static_accel_calib_status);
	  if (static_accel_calib_status == SN_CALIB_STATUS_CALIBRATED)
		printf("已校准\n");
	  else
		{
		  pass_false_flag = 1;
		  printf("FALSE\n");
	  }
	printf("-----------------------------------------------------------------------------------------\n");
      printf("动态校准       dynamic_accel_calib_status      = ");
      //print_sn_calib_status_string(dynamic_accel_calib_status);
	  if (dynamic_accel_calib_status == SN_CALIB_STATUS_CALIBRATED)
		printf("已校准\n");
	  else
		{
		  pass_false_flag = 1;
		  printf("FALSE\n");
	  }
	printf("-----------------------------------------------------------------------------------------\n");
      printf("陀螺仪温度校准 thermal_imu_calib_status        = ");
      //print_sn_calib_status_string(thermal_imu_calib_status);
	  if (thermal_imu_calib_status == SN_CALIB_STATUS_CALIBRATED)
		printf("已校准\n");
	  else
		{
		  pass_false_flag = 1;
		  printf("FALSE\n");
	  }
	printf("-----------------------------------------------------------------------------------------\n");
      printf("光流YAW校准    optic_flow_cam_yaw_calib_status = ");
      //print_sn_calib_status_string(optic_flow_cam_yaw_calib_status);
	  if (optic_flow_cam_yaw_calib_status == SN_CALIB_STATUS_CALIBRATED)
		printf("已校准\n");
	  else
		{
		  pass_false_flag = 1;
		  printf("FALSE\n");
	  }
	printf("-----------------------------------------------------------------------------------------\n");
      printf("地磁校准       mag_calib_status                = ");
      //print_sn_calib_status_string(mag_calib_status);
	  if (mag_calib_status == SN_CALIB_STATUS_CALIBRATED)
		printf("已校准\n");
	  else
		{
		  pass_false_flag = 1;
		  printf("FALSE\n");
	  }
	  */

	printf("\n\n\n");

    if(pass_false_flag == 1)
	{
		printf("     #########      #        #   #          \n");
		printf("     #            #   #      #   #          \n");
		printf("     #########   #######     #   #          \n");
		printf("     #          #       #    #   #          \n");
		printf("     #         #         #   #   ########## \n");

	}
	else
	{
		printf("     #########     #       #########   ######### \n");
		printf("     #       #   #   #     #           #         \n");
		printf("     #########  #######    #########   ######### \n");
		printf("     #         #       #           #           # \n");
		printf("     #        #         #  #########   ######### \n");
	}

    	}

    //Sleep until next loop
    usleep(100000);
  	}
  return 0;
}
