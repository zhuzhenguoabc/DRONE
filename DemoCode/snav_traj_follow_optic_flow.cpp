/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

// system includes
#include <cmath>
#include <cstdbool>
#include <cstring>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

// Snapdragon Navigator
#include "snapdragon_navigator.h"

// States used to control mission
enum class MissionState
{
  UNKNOWN,
  ON_GROUND,
  STARTING_PROPS,
  TAKEOFF,
  LOITER,
  LANDING,
  TRAJECTORY_FOLLOW
};

struct Position
{
  float x;   // m
  float y;   // m
  float z;   // m
};

float constrain_min_max(float input, float min, float max)
{
  if (input > max)
    return max;
  else if (input < min)
    return min;
  else
    return input;
}

int main(int argc, char* argv[])
{
  // Desired takeoff altitude
  const float kDesTakeoffAlt = 1.5;  // m

  // Fixed takeoff and landing speed
  const float kLandingSpeed = -0.75;  // m/s
  const float kTakeoffSpeed = 0.75;   // m/s

  // Delay before starting props
  const float kStartDelay = 3;       // s

  // Time to loiter
  const float kLoiterTime = 3;       // s

  Position pos0;
  pos0.x = 1;
  pos0.y = 0;
  pos0.z = 1;

  Position pos1;
  pos1.x = 1;
  pos1.y = 1;
  pos1.z = 1;

  Position pos2;
  pos2.x = 0;
  pos2.y = 1;
  pos2.z = 1;

  Position pos3;
  pos3.x = 0;
  pos3.y = 0;
  pos3.z = 1;

  std::vector<Position> positions;
  positions.push_back(pos0);
  positions.push_back(pos1);
  positions.push_back(pos2);
  positions.push_back(pos3);

  MissionState state = MissionState::UNKNOWN;
  bool mission_in_progress = true;
  bool mission_success = false;
  int loop_counter = 0;

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
      // ensure gps is disabled. Currently, the desired position will
      // be wrong for optic flow control if GPS is enabled.
      int gps_enabled;
      sn_is_gps_enabled(&gps_enabled);
/*      if(gps_enabled != 0)
      {
        printf("Error: GPS enabled. Desired state will be incorrect for optic flow modes\n");
        mission_in_progress = false;
        continue;
      }
*/
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

      static bool mission_has_begun = false;
      // This mission will use optic flow for position control
      if (mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
      {
        if (!mission_has_begun)
        {
          if (props_state == SN_PROPS_STATE_NOT_SPINNING)
          {
            // Ready to start the mission
            state = MissionState::ON_GROUND;
          }
          else
          {
            printf("Props must be stopped when this mission begins.\n");
            mission_in_progress = false;
            mission_success = false;
          }
        }
      }
      else
      {
        if (mission_has_begun)
        {
          // Abort mission if mode changed
          mission_in_progress = false;
          mission_success = false;
        }
        else
        {
          printf("Vehicle must be in SN_OPTIC_FLOW_POS_HOLD_MODE for mission to proceed.\n");
          state = MissionState::UNKNOWN;
        }
      }

      struct timeval tv;
      gettimeofday(&tv, NULL);
      double t_now = tv.tv_sec + tv.tv_usec * 1e-6;

      // Desired velocity in vehicle world frame
      float x_vel_des = 0;
      float y_vel_des = 0;
      float z_vel_des = 0;
      float yaw_vel_des = 0;

      // Position at startup
      static float x_est_startup = 0;
      static float y_est_startup = 0;
      static float z_est_startup = 0;

      // Mission State Machine
      static size_t current_position = 0;
      if (state == MissionState::ON_GROUND)
      {
        // Send zero velocity while vehicle sits on ground
        x_vel_des = 0;
        y_vel_des = 0;
        z_vel_des = 0;
        yaw_vel_des = 0;

        mission_has_begun = true;

        static double t_start = t_now;

        static bool commanded_spinup = false;
        if (!commanded_spinup)
        {
          if ((t_now - t_start) > kStartDelay)
          {
            state = MissionState::STARTING_PROPS;
            commanded_spinup = true;
          }
          else
          {
            printf("Countdown to mission start: %f\n",kStartDelay - (t_now - t_start));
          }
        }
      }

      else if (state == MissionState::STARTING_PROPS)
      {
        x_vel_des = 0;
        y_vel_des = 0;
        z_vel_des = 0;
        yaw_vel_des = 0;

        if (props_state == SN_PROPS_STATE_NOT_SPINNING)
        {
          x_est_startup = x_est;
          y_est_startup = y_est;
          z_est_startup = z_est;
          sn_spin_props();
        }
        else if (props_state == SN_PROPS_STATE_SPINNING)
        {
          state = MissionState::TAKEOFF;
        }
      }

      else if (state == MissionState::TAKEOFF)
      {
        if (props_state == SN_PROPS_STATE_SPINNING)
        {
          // Command constant positive z velocity during takeoff
          x_vel_des = 0;
          y_vel_des = 0;
          z_vel_des = kTakeoffSpeed;
          yaw_vel_des = 0;

          if (z_des - z_est_startup > kDesTakeoffAlt)
          {
            state = MissionState::LOITER;
          }
        }
        else
        {
          mission_in_progress = false;
          mission_success = false;
        }
      }

      else if (state == MissionState::LANDING)
      {
        if (props_state == SN_PROPS_STATE_SPINNING)
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
            state = MissionState::ON_GROUND;
          }
        }
        else
        {
          state = MissionState::ON_GROUND;
          mission_success = true;
          mission_in_progress = false;
        }
      }

      else if(state == MissionState::TRAJECTORY_FOLLOW)
      {
        float vel_target = 0.5;   //m/sec
        float accel_max  = 1.5;   //m/sec/sec
        float stopping_accel = 1.0; //m/sec/sec

        static double t_last = 0;

        static float vel_x_des_sent = 0;
        static float vel_y_des_sent = 0;
        static float vel_z_des_sent = 0;

        float command_dir_x;
        float command_dir_y;
        float command_dir_z;

        float vel_x_target;
        float vel_y_target;
        float vel_z_target;
        float yaw_target;

        yaw_vel_des = 0;

        command_dir_x = positions[current_position].x - (x_des-x_est_startup);
        command_dir_y = positions[current_position].y - (y_des-y_est_startup);
        command_dir_z = positions[current_position].z - (z_des-z_est_startup);

        float command_mag = sqrt(command_dir_x*command_dir_x +
                                 command_dir_y*command_dir_y +
                                 command_dir_z*command_dir_z);

        if(current_position == (positions.size()-1) )
        {
          //if it is on the last waypoint then slow down before stopping
          float stopping_vel = sqrt(2*stopping_accel*command_mag);
          if(stopping_vel<vel_target) { vel_target = stopping_vel; }
        }

        yaw_target = atan2(command_dir_y,command_dir_x);
        float e_yaw = yaw_target - yaw_des;

        if (e_yaw > M_PI) e_yaw = e_yaw - 2*M_PI;
        else if (e_yaw < -M_PI) e_yaw = e_yaw + 2*M_PI;

        if(command_mag<0.04)
        {
          // Close enough, move on
          current_position++;
          if (current_position >= positions.size())
          {
            // No more positions, so land
            state = MissionState::LANDING;
          }
        }

        if(command_mag<0.01){command_mag = 0.01;}  //to prevent dividing by zero

        vel_x_target = command_dir_x/command_mag * vel_target;
        vel_y_target = command_dir_y/command_mag * vel_target;
        vel_z_target = command_dir_z/command_mag * vel_target;

        float delT;
        if(t_last != 0)
        {
          delT = (t_now - t_last);
        }
        else
        {
          delT = 0.02;
        }

        t_last = t_now;

        //now converge the velocity to desired velocity

        float v_del_max = accel_max*delT;

        float vel_x_diff = (vel_x_target - vel_x_des_sent);
        float vel_y_diff = (vel_y_target - vel_y_des_sent);
        float vel_z_diff = (vel_z_target - vel_z_des_sent);

        float vel_diff_mag = sqrt(vel_x_diff*vel_x_diff +
                                  vel_y_diff*vel_y_diff +
                                  vel_z_diff*vel_z_diff);

        if(vel_diff_mag<v_del_max)
        {
          //send through the target velocity
          vel_x_des_sent = vel_x_target;
          vel_y_des_sent = vel_y_target;
          vel_z_des_sent = vel_z_target;
        }
        else
        {
          //converge to the target velocity at the max acceleration rate
          vel_x_des_sent += vel_x_diff/vel_diff_mag * v_del_max;
          vel_y_des_sent += vel_y_diff/vel_diff_mag * v_del_max;
          vel_z_des_sent += vel_z_diff/vel_diff_mag * v_del_max;
        }

        x_vel_des = vel_x_des_sent;
        y_vel_des = vel_y_des_sent;
        z_vel_des = vel_z_des_sent;
      }

      else if (state == MissionState::LOITER)
      {
        if (props_state == SN_PROPS_STATE_SPINNING)
        {
          // Maintain current position
          x_vel_des = 0;
          y_vel_des = 0;
          z_vel_des = 0;
          yaw_vel_des = 0;

          static bool entering_loiter = true;
          static double t_loiter_start = 0;
          if (entering_loiter)
          {
            t_loiter_start = t_now;
            entering_loiter = false;
          }

          if (t_now - t_loiter_start > kLoiterTime)
          {
            state = MissionState::TRAJECTORY_FOLLOW;
          }
        }
        else
        {
          mission_in_progress = false;
          mission_success = false;
        }
      }

      else
      {
        // Unknown state has been encountered
        x_vel_des = 0;
        y_vel_des = 0;
        z_vel_des = 0;
        yaw_vel_des = 0;

        if (props_state == SN_PROPS_STATE_SPINNING
            && on_ground_flag == 1)
        {
          sn_stop_props();
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

      // Go from the commands in real units computed above to the
      // dimensionless commands that the interface is expecting using a
      // linear mapping
      sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
          x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des,
          &cmd0, &cmd1, &cmd2, &cmd3);

      // Send the commands
      sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
          cmd0, cmd1, cmd2, cmd3);

      // Print some information
      if(mode == SN_OPTIC_FLOW_POS_HOLD_MODE){printf("\n[%d] OPTIC FLOW VELOCITY MODE. ",loop_counter);}
      else if(mode == SN_SENSOR_ERROR_MODE){printf("\n[%d] SENSOR ERROR MODE. ",loop_counter);}
      else{printf("\n[%d] UNDEFINED MODE. ",loop_counter);}

      if(props_state == SN_PROPS_STATE_NOT_SPINNING){printf("Propellers NOT spinning\n");}
      else if(props_state == SN_PROPS_STATE_STARTING){printf("Propellers attempting to spin\n");}
      else if (props_state == SN_PROPS_STATE_SPINNING){printf("Propellers spinning\n");}
      else{printf("Unknown propeller state\n");}

      printf("[%d] commanded rates: [%f,%f,%f,%f]\n",loop_counter,x_vel_des_yawed,y_vel_des_yawed,z_vel_des,yaw_vel_des);
      printf("[%d] [x_est,y_est,z_est,yaw_est]: [%f,%f,%f,%f]\n",loop_counter,x_est-x_est_startup,y_est-y_est_startup,z_est-z_est_startup,yaw_est);
      printf("[%d] [x_des,y_des,z_des,yaw_des]: [%f,%f,%f,%f]\n",loop_counter,x_des-x_est_startup,y_des-y_est_startup,z_des-z_est_startup,yaw_des);
      printf("[%d] position_est_startup: [%f,%f,%f]\n",loop_counter,x_est_startup,y_est_startup,z_est_startup);
      printf("[%d] battery_voltage: %f\n",loop_counter,voltage);
      if (state == MissionState::ON_GROUND)
        printf("[%d] ON_GROUND\n",loop_counter);
      else if (state == MissionState::STARTING_PROPS)
        printf("[%d] STARTING_PROPS\n",loop_counter);
      else if (state == MissionState::TAKEOFF)
        printf("[%d] TAKEOFF\n",loop_counter);
      else if (state == MissionState::TRAJECTORY_FOLLOW)
      {
        printf("[%d] TRAJECTORY_FOLLOW\n",loop_counter);
        printf("[%d] position #%u: [%f,%f,%f]\n",loop_counter,
            current_position,positions[current_position].x,
            positions[current_position].y, positions[current_position].z);
      }
      else if (state == MissionState::LOITER)
        printf("[%d] LOITER\n",loop_counter);
      else if (state == MissionState::LANDING)
        printf("[%d] LANDING\n",loop_counter);
      else
        printf("[%d] STATE UNKNOWN\n",loop_counter);
    }

    loop_counter++;
    usleep(20000);
  }


  if (mission_success)
  {
    printf("Trajectory Following Mission was completed successfully.\n");
  }
  else
  {
    printf("Trajectory Following Mission was aborted.\n");
  }

  return 0;
}
