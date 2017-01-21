/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */


// system includes
#include <cmath>
#include <cstdbool>
#include <cstring>
#include <fstream>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
using namespace std;

// Snapdragon Navigator
#include "snapdragon_navigator.h"

// GPS Waypoint Utilities
#include "snav_waypoint_utils.hpp"

// States used to control mission
enum class MissionState
{
  UNKNOWN,
  ON_GROUND,
  STARTING_PROPS,
  TAKEOFF,
  LOITER,
  LANDING,
  WAYPOINT_FOLLOW,
  EMERGENCY_LANDING
};

enum class YawOption
{
  YAW_NONE,
  YAW_WAYPOINT,
  YAW_VELOCITY_DIR,
  YAW_AWAY_FROM_HOME,
  YAW_CONSTANT_VEL
};

enum class PosOption
{
  POS_NONE,
  POS_DFT,
  POS_GPS,
};

int main(int argc, char* argv[])
{

  //Copy Waypoints from txt file into vector array
  //ifstream infile("waypoints.txt");
  float x, y, z, yaw;
  FlatVars wp_temp;

  std::vector<FlatVars> waypoints;

	/*
  while(!infile.eof()){

    infile >> x >> y >> z >> yaw;

    wp_temp.x = x;
    wp_temp.y = y;
    wp_temp.z = z;
    wp_temp.yaw = yaw;

    waypoints.push_back(wp_temp);
  }*/


  FlatVars wp_value[4];
  int i;


  wp_value[0].x = 1;
  wp_value[0].y = 0;
  wp_value[0].z = 1.5f;
  wp_value[0].yaw = 0;

  wp_value[1].x = 1;
  wp_value[1].y = 1;
  wp_value[1].z = 1.5f;
  wp_value[1].yaw = 0.5f;

  wp_value[2].x = 0;
  wp_value[2].y = 1;
  wp_value[2].z = 1.5f;
  wp_value[2].yaw = -0.5f;

  wp_value[3].x = 0;
  wp_value[3].y = 0;
  wp_value[3].z = 1.5f;
  wp_value[3].yaw = 0;

  for (i=0; i<4; i++)
  {
	waypoints.push_back(wp_value[i]);
  }


  // Desired takeoff altitude
  const float kDesTakeoffAlt = 1.5;  // m

  // Fixed takeoff and landing speed
  const float kLandingSpeed = -0.75;  // m/s
  const float maxTakeoffSpeed = 1;   // m/s
  const float minTakeoffSpeed = 0.05; // m/s (z-vel during arrival at desired takeoff altitude)
  float z_des_accel_takeoff = .25;

  // Delay before starting props
  const float kStartDelay = 1;       // s

  // Time to loiter
  const float kLoiterTimeTakeoff = 1;       // s
  const float kLoiterTimeLand = 40;	//2;

 // Time after landing to stop props
  const float kStopPropTime = 2;       // s

  // Set max velocities for goto_waypoint
  set_max_lin_velocity(0.5);
  set_max_lin_acceleration(0.5);
  set_max_ang_velocity(1.0);
  set_max_ang_acceleration(0.5);

  // Choose desired Yaw control mode
  //YawOption yaw_option = YawOption::YAW_NONE; // No yaw control
  //YawOption yaw_option = YawOption::YAW_WAYPOINT; // Yaw control to yaw waypoint from .txt
  YawOption yaw_option = YawOption::YAW_VELOCITY_DIR; // Yaw control to velocity direction
  //YawOption yaw_option = YawOption::YAW_AWAY_FROM_HOME; // Yaw control away from home coordinates (input below)
  //YawOption yaw_option = YawOption::YAW_CONSTANT_VEL; // Yaw at a constant rate

  // For YawOption::YAW_AWAY_FROM_HOME, input the coordinates of pilot (HOME)
  float x_pilot = -3; //Default: stand 3 m behind vehicle (or west of [if mag enabled])
  float y_pilot = 0; //Default: 0m

  // For YawOption::YAW_CONSTANT_VEL, input yaw speed and direction
  float yaw_vel_const = .5; //rad/s
  float yaw_dir = 0;

  MissionState state = MissionState::UNKNOWN;
  bool mission_in_progress = true;
  bool mission_success = false;
  uint8_t wp_goal_ret = 0b11111111;
  uint8_t wp_goal_mask = 0b11111111;
    // from goto_waypoint, 0's represent goal_reached. 8 bits are: xp,yp,zp,yawp,xv,yv,zv,yawv
  bool waypoints_complete = false;
  bool entering_loiter = true;
  bool stop_at_wp = false;

  bool clockwise_mission = false;
  bool anticlockwise_mission = false;
  const float clockwise_speed = 0.2f;       // s

  int loop_counter = 0;
  int wp_ret = 0;
  int nan = 0;
  float lateral_speed=0;
  float distance_to_wp=0;

  FlatVars output_vel;

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
  static size_t current_wp = 0;
  static double t_stop_props = 0;
  FlatVars wp_state = {waypoints[current_wp].x, waypoints[current_wp].y, waypoints[current_wp].z, waypoints[current_wp].yaw};

  int pos_arg;
  PosOption pos_option = PosOption::POS_NONE;

  while ((pos_arg = getopt(argc, argv, "dgh")) != -1)
  {
    switch(pos_arg)
    {
      case 'd':
        pos_option = PosOption::POS_DFT;
        break;
      case 'g':
        pos_option = PosOption::POS_GPS;
        break;
      case 'h':
        mission_in_progress = false;
        print_usage();
        return -1;
      default:
        mission_in_progress = false;
        print_usage();
        return -1;
    }
  }

  if (pos_option == PosOption::POS_NONE)
  {
    mission_in_progress = false;
    print_usage();
    return -1;
  }

  SnavCachedData *cached_data;
  if(sn_get_flight_data_ptr(sizeof(SnavCachedData),&cached_data)!=0)
  {
    printf("Error getting cached data.\n");
    return -1;
  };

  system("camera_super");

  // Begin mission loop
  while (mission_in_progress)
  {

    // Always need to call this
    if (sn_update_data() != 0)
    {
      printf("sn_update_data failed\n");
      break;
    }

    int gps_enabled;
    sn_is_gps_enabled(&gps_enabled);
    if(pos_option == PosOption::POS_GPS && gps_enabled != 1)
    {
      printf("Error: GPS NOT enabled. Desired state will be incorrect for GPS modes\n");
      break;
    }

    // Get the current mode
    SnMode mode = (SnMode)(cached_data->general_status.current_mode);

    // Get the current state of the propellers
    SnPropsState props_state = (SnPropsState)(cached_data->general_status.props_state);

    // Get the "on ground" flag
    int on_ground_flag = (cached_data->general_status.on_ground);


    // Get the current estimated and desired position and yaw
    float x_est, y_est, z_est, yaw_est, x_des, y_des, z_des, yaw_des;
    if(pos_option == PosOption::POS_GPS)
    {
      x_est = (cached_data->gps_pos_vel.position_estimated[0]);
      y_est = (cached_data->gps_pos_vel.position_estimated[1]);
      z_est = (cached_data->gps_pos_vel.position_estimated[2]);
      yaw_est = (cached_data->gps_pos_vel.yaw_estimated);

      x_des = (cached_data->gps_pos_vel.position_desired[0]);
      y_des = (cached_data->gps_pos_vel.position_desired[1]);
      z_des = (cached_data->gps_pos_vel.position_desired[2]);
      yaw_des = (cached_data->gps_pos_vel.yaw_desired);
    }
    else if(pos_option == PosOption::POS_DFT)
    {
      x_est = (cached_data->optic_flow_pos_vel.position_estimated[0]);
      y_est = (cached_data->optic_flow_pos_vel.position_estimated[1]);
      z_est = (cached_data->optic_flow_pos_vel.position_estimated[2]);
      yaw_est = (cached_data->optic_flow_pos_vel.yaw_estimated);

      x_des = (cached_data->optic_flow_pos_vel.position_desired[0]);
      y_des = (cached_data->optic_flow_pos_vel.position_desired[1]);
      z_des = (cached_data->optic_flow_pos_vel.position_desired[2]);
      yaw_des = (cached_data->optic_flow_pos_vel.yaw_desired);
    }

	printf("Cpu temp:%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d.\n",
			(int)cached_data->cpu_stats.temp[0],(int)cached_data->cpu_stats.temp[1],
			(int)cached_data->cpu_stats.temp[2],(int)cached_data->cpu_stats.temp[3],
			(int)cached_data->cpu_stats.temp[4],(int)cached_data->cpu_stats.temp[5],
			(int)cached_data->cpu_stats.temp[6],(int)cached_data->cpu_stats.temp[7],
			(int)cached_data->cpu_stats.temp[8],(int)cached_data->cpu_stats.temp[9],
			(int)cached_data->cpu_stats.temp[10],(int)cached_data->cpu_stats.temp[11],
			(int)cached_data->cpu_stats.temp[12]);


    // Get the current battery voltage
    float voltage = (cached_data->general_status.voltage);

    static bool mission_has_begun = false;

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
        break;
      }
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);
    double t_now = tv.tv_sec + tv.tv_usec * 1e-6;


    // Check to ensure the correct mode can be entered
    static int mode_err_cntr = 0;
    if(((pos_option == PosOption::POS_DFT) && (mode != SN_OPTIC_FLOW_POS_HOLD_MODE)) ||
      ((pos_option == PosOption::POS_GPS) && (mode != SN_GPS_POS_HOLD_MODE)))
    {
      mode_err_cntr++;
      printf("Error in control mode [%d]\n",mode_err_cntr);
      if(mode_err_cntr>10)
      {
        // 10 counts should be sufficient to transition. Emergency land if not
        printf("Error in control mode. Emergency landing.\n");
        state = MissionState::EMERGENCY_LANDING;
      }
    }
    else
    {
      // reset counter to 0 if mode is correct
      mode_err_cntr = 0;
    }

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
        // Start props after waiting for sufficient delay
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

      // Store estimated position before takeoff, and start props
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
        // Take off and smoothly slow down to desired takeoff altitude
        float z_error_takeoff = kDesTakeoffAlt - (z_des - z_est_startup);

        if (z_error_takeoff < 0){z_error_takeoff=0;}
        z_vel_des=sqrtf(z_error_takeoff)*sqrtf(2*z_des_accel_takeoff)+minTakeoffSpeed;

        if(z_vel_des > maxTakeoffSpeed)
        {
          z_vel_des = maxTakeoffSpeed;
        }

        // Command constant positive z velocity during takeoff
        x_vel_des = 0;
        y_vel_des = 0;
        yaw_vel_des = 0;

        // After reaching takeoff height, enter loiter mode
        if (z_des - z_est_startup > kDesTakeoffAlt)
        {
          state = MissionState::LOITER;
        }
      }
      else //Props stopped spinning
      {
        printf("PROPS STOPPED SPINNING.\n");
        break;
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
          t_stop_props = t_now;
        }
      }
      else
      {
        // Allows sufficient time for props to stop before ending mission
        if (t_now - t_stop_props > kStopPropTime)
        {
          state = MissionState::ON_GROUND;
          mission_success = true;
          mission_in_progress = false;
        }
      }
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

        static double t_loiter_start = 0;
        if (entering_loiter)
        {
          t_loiter_start = t_now;
          entering_loiter = false;
        }

        // If not all waypoints have been reached, enter waypoint follow mode after loiter
        if (!waypoints_complete)
        {
          if (t_now - t_loiter_start > kLoiterTimeTakeoff)
          {
            state = MissionState::WAYPOINT_FOLLOW;
            entering_loiter=true;
          }
        }
        // If all waypoints have been reached, enter landing mode after loiter
        else if (waypoints_complete)
        {
        	if ((t_now - t_loiter_start > 5) && (t_now - t_loiter_start <= 15))
        	{
				clockwise_mission = true;
				anticlockwise_mission = false;
			}
			else if ((t_now - t_loiter_start > 16) && (t_now - t_loiter_start <= 26))
			{
				clockwise_mission = false;
				anticlockwise_mission = true;
			}
			else if (t_now - t_loiter_start > 60)
			{
				clockwise_mission = false;
				anticlockwise_mission = false;

				state = MissionState::LANDING;
				entering_loiter=true;
			}
			else
			{
				clockwise_mission = false;
				anticlockwise_mission = false;
			}

        	/*
			if (t_now - t_loiter_start > kLoiterTimeLand)
			{
				state = MissionState::LANDING;
				entering_loiter=true;
			}
			*/
        }
      }
      else
      {
        printf("PROPS STOPPED SPINNING.\n");
        break;
      }
    }

    else if(state == MissionState::WAYPOINT_FOLLOW)
    {
      FlatVars current_state = {x_des-x_est_startup, y_des-y_est_startup, z_des-z_est_startup, yaw_des};
      FlatVars current_vel = {x_vel_des, y_vel_des, z_vel_des, yaw_vel_des};

      if (yaw_option==YawOption::YAW_WAYPOINT)
      {
        // Yaw waypoint is given in txt file
        wp_state = {waypoints[current_wp].x, waypoints[current_wp].y, waypoints[current_wp].z, waypoints[current_wp].yaw};
      }
      else if (yaw_option==YawOption::YAW_VELOCITY_DIR)
      {
        // Calculate direction of velocity for yaw waypoint
        if (abs(x_vel_des)<.05 && abs(y_vel_des)<0.05) {yaw_dir = yaw_des;}
        else {yaw_dir = atan2(y_vel_des, x_vel_des);}
        wp_state = {waypoints[current_wp].x, waypoints[current_wp].y, waypoints[current_wp].z, yaw_dir};
      }
      else if (yaw_option==YawOption::YAW_AWAY_FROM_HOME)
      {
        // Calculate direction from pilot to set yaw waypoint pointing away from home position
        if (abs(current_state.x-x_pilot)<.05 && abs(current_state.y-y_pilot)<0.05) {yaw_dir = yaw_des;}
        else {yaw_dir = atan2(current_state.y-y_pilot, current_state.x-x_pilot);}
        wp_state = {waypoints[current_wp].x, waypoints[current_wp].y, waypoints[current_wp].z, yaw_dir};
      }
      else //YAW_NONE; YAW_CONSTANT_VEL
      {
        wp_state = {waypoints[current_wp].x, waypoints[current_wp].y, waypoints[current_wp].z, yaw_des};
      }

      // Stop only at last waypoint. If stopping at all waypoints is desired, make always true
      if (current_wp==waypoints.size()-1) {stop_at_wp=true;}
      else {stop_at_wp=false;}

      // Get desired output velocities given current state and velocity and goal state
      wp_ret = goto_waypoint(current_state, wp_state, current_vel, stop_at_wp, &output_vel, &wp_goal_ret);

      // If YAW_CONSTANT_VEL, output a constant vel in yaw, and mask out yaw from wp_goal flags
      if (yaw_option==YawOption::YAW_CONSTANT_VEL)
      {
        output_vel.yaw=yaw_vel_const;
        wp_goal_mask=0b11101110;
      }

      x_vel_des = output_vel.x;
      y_vel_des = output_vel.y;
      z_vel_des = output_vel.z;
      yaw_vel_des = output_vel.yaw;

      distance_to_wp = sqrtf((current_state.x-wp_state.x)*(current_state.x-wp_state.x)+(current_state.y-wp_state.y)*(current_state.y-wp_state.y)+(current_state.z-wp_state.z)*(current_state.z-wp_state.z));
      lateral_speed = sqrtf((x_vel_des*x_vel_des)+(y_vel_des*y_vel_des)+(z_vel_des*z_vel_des));

      // If reached waypoint is met, increment waypoint
      if ((wp_goal_ret&wp_goal_mask) == 0)
      {
        current_wp++;
        wp_goal_ret = 0b11111111;

        // If last waypoint, enter loiter
        if (current_wp>= waypoints.size())
        {
        	current_wp = (waypoints.size()-1);
          waypoints_complete = true;
          state = MissionState::LOITER;
        }
      }
    }
    else if (state == MissionState::EMERGENCY_LANDING)
    {
      if (props_state == SN_PROPS_STATE_SPINNING)
      {
        //Command constant negative z velocity during landing
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
          t_stop_props = t_now;
        }
      }
      else
      {
        // Allows sufficient time for props to stop before ending mission
        if (t_now - t_stop_props > kStopPropTime)
        {
          state = MissionState::ON_GROUND;
          mission_success = true;
          mission_in_progress = false;
        }
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
    if (pos_option == PosOption::POS_DFT)
    {
      sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                          x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des,
                          &cmd0, &cmd1, &cmd2, &cmd3);

		cmd0 = cmd0*0.5f;
		cmd1 = cmd1*0.5f;

		if (clockwise_mission)
		{
			cmd3 = clockwise_speed;
		}
		else if (anticlockwise_mission)
		{
			cmd3 = -clockwise_speed;
		}

      // Send the commands if in the right mode.
      sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                        cmd0, cmd1, cmd2, cmd3);
    }
    else if (pos_option == PosOption::POS_GPS)
    {
      sn_apply_cmd_mapping(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                          x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des,
                          &cmd0, &cmd1, &cmd2, &cmd3);

		cmd0 = cmd0*0.5f;
		cmd1 = cmd1*0.5f;

		if (clockwise_mission)
		{
			cmd3 = clockwise_speed;
		}
		else if (anticlockwise_mission)
		{
			cmd3 = -clockwise_speed;
		}

      sn_send_rc_command(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                        cmd0, cmd1, cmd2, cmd3);
    }

    loop_counter++;
    usleep(20000);
  }

  system("pkill camera_super");

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
