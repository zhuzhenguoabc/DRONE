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

// Snapdragon Navigator API
#include "snapdragon_navigator.h"

// Waypoint utilities
#include "snav_waypoint_utils.hpp"

void ensure_optic_flow_mode_active(SnMode mode, int * state)
{
  // This function should only be called when rc_cmd_source == SN_RC_CMD_API_INPUT
  // There may be some transition time between switching to this input source and
  // the mode updating to SN_OPTIC_FLOW_POS_HOLD_MODE so allow a few errors before
  // moving to a new mode.
  static int mode_err_cntr = 0;
  if(mode != SN_OPTIC_FLOW_POS_HOLD_MODE)
  {
    mode_err_cntr++;
    printf("Error in control mode [%d]\n",mode_err_cntr);
    if(mode_err_cntr>10)
    {
      // 10 counts should be sufficient to transition. Emergency land if not
      printf("Error in control mode. Emergency landing.\n");
      *state = (int) GoHomeState::EMERGENCY_LANDING;
    }
  }
  else
  {
    // reset counter to 0 if mode is correct
    mode_err_cntr = 0;
  }
}

int main(int argc, char* argv[])
{
  const float kLandingSpeed = -0.65;  // m/s
  float distance_home_squared_threshold = 1;

  GoHomeState state = GoHomeState::HOME_NOT_OK;
  bool mission_in_progress = true;
  int loop_counter = 0;
  float yaw_target_home, distance_home_squared;
  float x_vel_des, y_vel_des, z_vel_des, yaw_vel_des;
  uint8_t wp_goal_ret = 0b11111111;
  uint8_t wp_goal_mask = 0b11111111;

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

      // Get the current mode
      SnMode mode = (SnMode)(cached_data->general_status.current_mode);

      // Get the source of the RC input (spektrum vs API) here
      SnRcCommandSource rc_cmd_source = (SnRcCommandSource)(cached_data->rc_active.source);

      // Get the current state of the propellers
      SnPropsState props_state = (SnPropsState)(cached_data->general_status.props_state);

      // Get the "on ground" flag
      int on_ground_flag = (cached_data->general_status.on_ground);

      // Get the current estimated position and yaw
      float x_est = (cached_data->optic_flow_pos_vel.position_estimated[0]);
      float y_est = (cached_data->optic_flow_pos_vel.position_estimated[1]);
      float z_est = (cached_data->optic_flow_pos_vel.position_estimated[2]);
      float yaw_est = (cached_data->optic_flow_pos_vel.yaw_estimated);

      // Get the current desired position and yaw
      float x_des = (cached_data->optic_flow_pos_vel.position_desired[0]);
      float y_des = (cached_data->optic_flow_pos_vel.position_desired[1]);
      float z_des = (cached_data->optic_flow_pos_vel.position_desired[2]);
      float yaw_des = (cached_data->optic_flow_pos_vel.yaw_desired);

      // Get the current battery voltage
      float voltage = (cached_data->general_status.voltage);

      // Distance and direction from current position to home
      distance_home_squared = (x_est_startup-x_est)*(x_est_startup-x_est)+(y_est_startup-y_est)*(y_est_startup-y_est);
      yaw_target_home = atan2(y_est_startup-y_est,x_est_startup-x_est);

      if (props_state == SN_PROPS_STATE_NOT_SPINNING)
      {
        state = GoHomeState::ON_GROUND;
      }
      if (state == GoHomeState::HOME_NOT_OK)
      {
        if (rc_cmd_source == SN_RC_CMD_API_INPUT)
        {
          // If no initial home position is stored, enter emergency landing instead of go_home
          state = GoHomeState::EMERGENCY_LANDING;
        }
      }
      else if (state == GoHomeState::ON_GROUND)
      {
        // Store home location while on ground before spinning props
        x_est_startup = x_est;
        y_est_startup = y_est;
        z_est_startup = z_est;

        if (props_state == SN_PROPS_STATE_STARTING)
        {
          state = GoHomeState::HOME_OK;
        }
      }
      else if (state == GoHomeState::HOME_OK)
      {
        if (rc_cmd_source == SN_RC_CMD_API_INPUT)
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

          ensure_optic_flow_mode_active(mode, (int*) &state);
        }
      }
      else if (state == GoHomeState::TURN_HOME)
      {
        // Go to waypoint that is in the same xyz position, but facing home
        goto_waypoint({x_des, y_des, z_des, yaw_des}, {x_des, y_des, z_des, yaw_target_home},
            {x_vel_des, y_vel_des, z_vel_des, yaw_vel_des}, true, &output_vel, &wp_goal_ret);

        x_vel_des = output_vel.x;
        y_vel_des = output_vel.y;
        z_vel_des = output_vel.z;
        yaw_vel_des = output_vel.yaw;


        if ((wp_goal_ret&wp_goal_mask) == 0)
        {
          wp_goal_ret = 0b11111111;
          // If waypoint is reached (facing home), enter Fly_home
          state = GoHomeState::FLY_HOME;
        }
        if (rc_cmd_source == SN_RC_CMD_API_INPUT)
        {
          ensure_optic_flow_mode_active(mode, (int*) &state);
        }
        else
        {
          // If switched out of go_home, enter home_ok state
          state = GoHomeState::HOME_OK;//HOME_OK;
          wp_goal_ret = 0b11111111;
        }

      }
      else if (state == GoHomeState::FLY_HOME)
      {
        // Go to home waypoint
        goto_waypoint({x_des, y_des, z_des, yaw_des}, {x_est_startup, y_est_startup, z_des, yaw_des},
                      {x_vel_des, y_vel_des, z_vel_des, yaw_vel_des}, true, &output_vel, &wp_goal_ret);

        x_vel_des = output_vel.x;
        y_vel_des = output_vel.y;
        z_vel_des = output_vel.z;
        yaw_vel_des = output_vel.yaw;

        if ((wp_goal_ret&wp_goal_mask) == 0)
        {
          // If home, enter landing
          state = GoHomeState::LANDING;
          wp_goal_ret = 0b11111111;
        }
        if (rc_cmd_source == SN_RC_CMD_API_INPUT)
        {
          ensure_optic_flow_mode_active(mode, (int*) &state);
        }
        else
        {
          // If switched out of go_home, enter home_ok state
          state = GoHomeState::HOME_OK;
          wp_goal_ret = 0b11111111;
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

        if (rc_cmd_source == SN_RC_CMD_API_INPUT)
        {
          ensure_optic_flow_mode_active(mode, (int*) &state);
        }
        else
        {
          state = GoHomeState::HOME_OK;
        }
      }
      else if (state == GoHomeState::EMERGENCY_LANDING)
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

        if (rc_cmd_source != SN_RC_CMD_API_INPUT)
        {
          state = GoHomeState::HOME_NOT_OK;//HOME_OK;
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
      sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING, x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des, &cmd0, &cmd1, &cmd2, &cmd3);
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

  return 0;
}
