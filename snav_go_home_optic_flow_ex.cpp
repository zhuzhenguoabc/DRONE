/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

// system includes
#include <cmath>
#include <cstring>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include <string>

// Snapdragon Navigator API
#include "snapdragon_navigator.h"

// Waypoint utilities
#include "snav_waypoint_utils.hpp"


int main(int argc, char* argv[])
{
  const float kLandingSpeed = -0.65;  // m/s
  float distance_home_squared_threshold = 1;

  GoHomeState state = GoHomeState::HOME_OK;
  bool mission_in_progress = true;
  int loop_counter = 0;
  bool wp_goal=false;
  float yaw_target_home, distance_home_squared;
  float x_vel_des, y_vel_des, z_vel_des, yaw_vel_des;

  FlatVars output_vel;

  // Position at startup
  static float x_est_startup = 0;
  static float y_est_startup = 0;
  static float z_est_startup = 0;

  SnavCachedData *cached_data;
  if(sn_get_flight_data_ptr(sizeof(SnavCachedData),&cached_data)!=0)
  {
    printf("Error getting cached data.\n");
    return -1;
  };

	time_t now;
	struct tm *curTime;
	now = time(NULL);
	curTime = localtime(&now);

	char log_filename[256];
	sprintf(log_filename,"/home/linaro/dev/examples/log_go_home_%04d-%02d-%02d-%02d-%02d-%02d",
					curTime->tm_year+1900,curTime->tm_mon+1,curTime->tm_mday,
					curTime->tm_hour,curTime->tm_min,curTime->tm_sec);
	printf("log_filename=%s\n", log_filename);

	FILE *fp;
	if ((fp = fopen(log_filename, "w+")) != NULL)
	{
		fclose(fp);
	}

	freopen(log_filename, "a", stdout); setbuf(stdout, NULL);
	freopen(log_filename, "a", stderr); setbuf(stderr, NULL);


  // Begin mission loop
  while (mission_in_progress)
  {
    // Update the cached flight data
    if (sn_update_data() != 0)
    {
      printf("sn_update_data failed\n");
      mission_in_progress = false;
    }
    else
    {
      // Ensure gps is disabled for this mode. Currently, the desired position
      // will be wrong for optic flow control if GPS is enabled.
      int gps_enabled;
      sn_is_gps_enabled(&gps_enabled);
      if(gps_enabled != 0)
      {
        printf("Error: GPS enabled. Desired state will be incorrect for optic flow modes\n");
        state = GoHomeState::HOME_NOT_OK;
        mission_in_progress = false;
        continue;
      }

      // Get the current mode
      SnMode mode;
      sn_get_mode(&mode);

      // Get the current state of the propellers
      SnPropsState props_state;
      sn_get_props_state(&props_state);

      // Get the "on ground" flag
      int on_ground_flag;
      sn_get_on_ground_flag(&on_ground_flag);

      // Get the current estimated position and yaw
      float x_est, y_est, z_est, yaw_est;
      sn_get_position_est(&x_est, &y_est, &z_est);
      sn_get_yaw_est(&yaw_est);

      // Get the current desired position and yaw
      // NOTE this is the setpoint that will be controlled by sending
      // velocity commands
      float x_des, y_des, z_des, yaw_des;
      sn_get_position_des(&x_des, &y_des, &z_des);
      sn_get_yaw_des(&yaw_des);

      // Get the current battery voltage
      float voltage;
      sn_get_voltage(&voltage);

	  x_est_startup = -1.445697;
		y_est_startup = -1.571068;
		z_est_startup = 1.897083;

      // Distance and direction from current position to home
      distance_home_squared = (x_est_startup-x_est)*(x_est_startup-x_est)+(y_est_startup-y_est)*(y_est_startup-y_est);
      yaw_target_home = atan2(y_est_startup-y_est,x_est_startup-x_est);

      if (state == GoHomeState::HOME_OK)
      {
          if (distance_home_squared > distance_home_squared_threshold)
          {
            // If sufficiently far away, turn home before flying home
            state = GoHomeState::TURN_HOME;
          }
          else
          {
            // If close to home already, fly straight home without yawing
            state = GoHomeState::FLY_HOME;
          }

          x_vel_des = 0;
          y_vel_des = 0;
          z_vel_des = 0;
          yaw_vel_des = 0;
      }
      else if (state == GoHomeState::TURN_HOME)
      {
        // Go to waypoint that is in the same xyz position, but facing home
        goto_waypoint({x_des, y_des, z_des, yaw_des}, {x_des, y_des, z_des, yaw_target_home},
            {x_vel_des, y_vel_des, z_vel_des, yaw_vel_des}, &output_vel, &wp_goal);

        x_vel_des = output_vel.x;
        y_vel_des = output_vel.y;
        z_vel_des = output_vel.z;
        yaw_vel_des = output_vel.yaw;

        if (wp_goal == true)
        {
          // If waypoint is reached (facing home), enter Fly_home
          wp_goal = false;
          state = GoHomeState::FLY_HOME;
        }
      }
      else if (state == GoHomeState::FLY_HOME)
      {
        // Go to home waypoint
        goto_waypoint({x_des, y_des, z_des, yaw_des}, {x_est_startup, y_est_startup, z_des, yaw_des}, {x_vel_des, y_vel_des, z_vel_des, yaw_vel_des}, &output_vel, &wp_goal);

        x_vel_des = output_vel.x;
        y_vel_des = output_vel.y;
        z_vel_des = output_vel.z;
        yaw_vel_des = output_vel.yaw;

        if (wp_goal == true)
        {
          // If home, enter landing
          state = GoHomeState::LANDING;
          wp_goal = false;
        }
      }
      else if (state == GoHomeState::LANDING)
      {
        // Command constant negative z velocity during landing
        x_vel_des = 0;
        y_vel_des = 0;
        z_vel_des = kLandingSpeed;
        yaw_vel_des = 0;

        if (props_state == SN_PROPS_STATE_SPINNING
            && on_ground_flag == 1)
        {
          // Snapdragon Navigator has determined that vehicle is on ground,
          // so it is safe to kill the propellers
          sn_stop_props();
        }
        if (props_state == SN_PROPS_STATE_NOT_SPINNING)
        {
          state = GoHomeState::ON_GROUND;
        }
      }

      // Rotate velocity by estimated yaw angle before sending
      // This puts velocity in body-relative Z-up frame
      float x_vel_des_yawed = x_vel_des*cos(-yaw_est) - y_vel_des*sin(-yaw_est);
      float y_vel_des_yawed = x_vel_des*sin(-yaw_est) + y_vel_des*cos(-yaw_est);

      // Send desired velocity
      float cmd0 = 0;
      float cmd1 = 0;
      float cmd2 = 0;
      float cmd3 = 0;

	  printf("[sn_apply_cmd_mapping x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des]: [%f,%f,%f,%f]\n",
	  											x_vel_des_yawed,y_vel_des_yawed,z_vel_des,yaw_vel_des);
      sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING, x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des, &cmd0, &cmd1, &cmd2, &cmd3);


	  printf("[sn_send_rc_command cmd0 cmd1 cmd2 cmd3]: [%f,%f,%f,%f]\n",cmd0,cmd1,cmd2,cmd3);
      sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING, cmd0, cmd1, cmd2, cmd3);

      // Print some information
      if(mode == SN_OPTIC_FLOW_POS_HOLD_MODE){printf("\n[%d] OPTIC FLOW CONTROL MODE.",loop_counter);}
      else if(mode == SN_SENSOR_ERROR_MODE){printf("\n[%d] SENSOR ERROR MODE.",loop_counter);}
      else{printf("\n[%d] UNDEFINED MODE. ",loop_counter);}
      if(props_state == SN_PROPS_STATE_NOT_SPINNING){printf("Propellers NOT spinning\n");}
      else if(props_state == SN_PROPS_STATE_STARTING){printf("Propellers attempting to spin\n");}
      else if (props_state == SN_PROPS_STATE_SPINNING){printf("Propellers spinning\n");}
      else{printf("Unknown propeller state\n");}
      printf("[%d] commanded rates: [%f,%f,%f,%f]\n",loop_counter,x_vel_des_yawed,y_vel_des_yawed,z_vel_des,yaw_vel_des);
      printf("[%d] [x_est,y_est,z_est,yaw_est]: [%f,%f,%f,%f]\n",loop_counter,x_est,y_est,z_est,yaw_est);
      printf("[%d] [x_des,y_des,z_des,yaw_des]: [%f,%f,%f,%f]\n",loop_counter,x_des,y_des,z_des,yaw_des);
      printf("[%d] [x_vel,y_vel,z_vel,yaw_vel: [%f,%f,%f,%f]\n",loop_counter,x_vel_des,y_vel_des,z_vel_des,yaw_vel_des);
      printf("[%d] position_est_startup: [%f,%f,%f]\n",loop_counter,x_est_startup,y_est_startup,z_est_startup);
      printf("[%d] battery_voltage: %f\n",loop_counter,voltage);
      printf("[%d] distance_home_squared: %f\n",loop_counter,distance_home_squared);
      if (state == GoHomeState::ON_GROUND)
        printf("[%d] ON_GROUND\n",loop_counter);
      else if (state == GoHomeState::HOME_OK)
        printf("[%d] HOME_OK\n",loop_counter);
      else if (state == GoHomeState::FLY_HOME)
        printf("[%d] FLY_HOME\n",loop_counter);
      else if (state == GoHomeState::TURN_HOME)
        printf("[%d] TURN_HOME\n",loop_counter);
      else if (state == GoHomeState::LANDING)
        printf("[%d] LANDING\n",loop_counter);
      else if (state == GoHomeState::HOME_NOT_OK)
        printf("[%d] HOME_NOT_OK\n",loop_counter);
      else if (state == GoHomeState::EMERGENCY_LANDING)
        printf("[%d] EMERGENCY_LANDING\n",loop_counter);
    }
    loop_counter++;
    usleep(20000);
  }

  fclose(stdout);
	fclose(stderr);

  return 0;
}
