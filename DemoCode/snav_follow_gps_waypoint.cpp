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
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>

// Waypoint utilities
#include "snav_waypoint_utils.hpp"

// Snapdragon Navigator
#include "snapdragon_navigator.h"
#define MAXBUFLEN 512
using namespace std;

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

struct GpsPosition
{
  int latitude;   // xx.xxxxxx
  int longitude;   //xxx.xxxxx
  int altitude;   //
  float yaw;
};

std::vector<std::string> split(const  std::string& s, const std::string& delim)
{
    std::vector<std::string> elems;
    size_t pos = 0;
    size_t len = s.length();
    size_t delim_len = delim.length();
    if (delim_len == 0) return elems;
    while (pos < len)
    {
        int find_pos = s.find(delim, pos);
        if (find_pos < 0)
        {
            elems.push_back(s.substr(pos, len - pos));
            break;
        }
        elems.push_back(s.substr(pos, find_pos - pos));
        pos = find_pos + delim_len;
    }
    return elems;
}

float rad(double d)
{
 const float PI = 3.1415926;
 return d * PI / 180.0;
}

float  CalcDistance(float fLati1, float fLong1, float fLati2, float fLong2)
{
  const float EARTH_RADIUS = 6378137;

  double radLat1 = rad(fLati1);
  double radLat2 = rad(fLati2);
  double a = radLat1 - radLat2;
  double b = rad(fLong1) - rad(fLong2);
  double s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
  s = s * EARTH_RADIUS;
  return s;
}

float  CalcAxisDistance(float f1,  float f2)
{
  const float EARTH_RADIUS = 6378137.0; //r =6378.137km earth radius
  const float PI = 3.1415926;

  double s =(f1-f2) * PI * EARTH_RADIUS / 180.0; //  n*pi*r/180
  return s;
}

int main(int argc, char* argv[])
{
  // Desired takeoff altitude
  const float kDesTakeoffAlt = 2.0;  // m

  // Fixed takeoff and landing speed
  const float kLandingSpeed = -0.75;  // m/s
  const float kTakeoffSpeed = 0.75;   // m/s

  // Delay before starting props
  const float kStartDelay = 3;       // s

  // Time to loiter
  const float kLoiterTime = 1;       // s
  const char* ip_address = "192.168.1.1";
  int port = 14567;

  GpsPosition posLast;
  GpsPosition posCurrent;
  GpsPosition posDestination;
  float destyaw = 0;
  float distance_to_dest=0;

  MissionState state = MissionState::UNKNOWN;
  bool mission_in_progress = true;
  bool mission_success = false;
  int loop_counter = 0;

  int sockfd;
  struct sockaddr_in my_addr; // my address information
  struct sockaddr_in their_addr; // connector's address information
  socklen_t addr_len;
  int numbytes;
  char buf[MAXBUFLEN];

  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
  {
    perror("socket");
    exit(1);
  }

  my_addr.sin_family = AF_INET;    // host byte order
  my_addr.sin_addr.s_addr = inet_addr(ip_address); // automatically fill with my IP
  my_addr.sin_port = htons(port);  // short, network byte order
  memset(&(my_addr.sin_zero), '\0', 8); // zero the rest of the struct

  printf("Using IP %s and port %d\n", ip_address, port);
  if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1)
  {
    perror("bind");
    exit(1);
  }

  addr_len = sizeof(struct sockaddr);

  SnavCachedData* snav_data = NULL;
  if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
  {
    printf("\nFailed to get flight data pointer!\n");
    return -1;
  }
  printf("argc:%d\n",argc);

/*  printf("distance:%f\n", CalcDistance(22.5278256f,113.9334979f,22.5278321f,113.9336830f));
  printf("distancey:%f\n", CalcAxisDistance((float)225278256/1e7,(float)225278321/1e7));
 printf("distancex:%f\n", CalcAxisDistance((float)1139334979/1e7,(float)1139336830/1e7));

  if(argc >2)
  {
	posDestination.latitude = atoi(argv[1]);
	posDestination.longitude = atoi(argv[2]);
	printf("posDestination gps latitude,longitude:%d,%d \n",
		posDestination.latitude,posDestination.longitude);
  }
*/
  // Begin mission loop
  while (mission_in_progress)
  {
    // Always need to call this
    if (sn_update_data() != 0)
    {
      printf("sn_update_data failed\n");
      mission_in_progress = false;
    }
    else
    {
/*      int gps_enabled;
      sn_is_gps_enabled(&gps_enabled);
      if(gps_enabled != 1)
      {
        printf("Error: GPS disabled. Desired state will be incorrect for optic flow modes\n");
        mission_in_progress = false;
        continue;
      }

	  SnDataStatus gps_status = (SnDataStatus) snav_data->data_status.gps_0_status;
	  if (gps_status != SN_DATA_VALID)
	  {
		printf("cant get gps location \n");
		mission_in_progress = false;
	  }
*/
      // Get the current mode
      SnMode mode;
      mode = (SnMode)snav_data->general_status.current_mode;

      // Get the current state of the propellers
      SnPropsState props_state;
      props_state = (SnPropsState) snav_data->general_status.props_state;

      // Get the "on ground" flag
      int on_ground_flag;
	  on_ground_flag = (int)snav_data->general_status.on_ground;

	  if((int)snav_data->gps_0_raw.fix_type == 3 )
	  {
	      if(on_ground_flag == 1)// on ground
	      {
			  posLast.latitude = (int)snav_data->gps_0_raw.latitude;
			  posLast.longitude = (int)snav_data->gps_0_raw.longitude;
			  posLast.altitude = (int)snav_data->gps_0_raw.altitude;
			  posLast.yaw = (float)snav_data->high_level_control_data.yaw_estimated;
	  	  }
		  else
		  {
			  posCurrent.latitude = (int)snav_data->gps_0_raw.latitude;
			  posCurrent.longitude = (int)snav_data->gps_0_raw.longitude;
			  posCurrent.altitude = (int)snav_data->gps_0_raw.altitude;
			  posCurrent.yaw = (float)snav_data->high_level_control_data.yaw_estimated;
		  }
	  }

      // Get the current estimated position and yaw
      float x_est, y_est, z_est, yaw_est;
      x_est = (float)snav_data->high_level_control_data.position_estimated[0];
	  y_est = (float)snav_data->high_level_control_data.position_estimated[1];
	  z_est = (float)snav_data->high_level_control_data.position_estimated[2];
	  yaw_est = (float)snav_data->high_level_control_data.yaw_estimated;

      // Get the current desired position and yaw
      // NOTE this is the setpoint that will be controlled by sending
      // velocity commands
      float x_des, y_des, z_des, yaw_des;
	  x_des = (float)snav_data->high_level_control_data.position_desired[0];
	  y_des = (float)snav_data->high_level_control_data.position_desired[1];
	  z_des = (float)snav_data->high_level_control_data.position_desired[2];
	  yaw_des = (float)snav_data->high_level_control_data.yaw_desired;

      // Get the current battery voltage
      float voltage;
	  voltage = (float)snav_data->general_status.voltage;

      static bool mission_has_begun = false;
      // This mission will use SN_GPS_POS_HOLD_MODE for position control
      if (mode == SN_GPS_POS_HOLD_MODE)
      //if (mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
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
          printf("Vehicle must be in SN_GPS_POS_HOLD_MODE for mission to proceed.\n");
          //state = MissionState::UNKNOWN;
          state = MissionState::ON_GROUND;
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
      //static size_t current_position = 0;
      if (state == MissionState::ON_GROUND)
      {
        // Send zero velocity while vehicle sits on ground
        x_vel_des = 0;
        y_vel_des = 0;
        z_vel_des = 0;
        yaw_vel_des = 0;

        mission_has_begun = true;

		//////// test
		// blocking until message is received
		if ((numbytes = recvfrom(sockfd, buf, MAXBUFLEN-1 , 0,
			  (struct sockaddr *)&their_addr, &addr_len)) == -1)
		{
			printf("cant get operration\n");
			exit(1);
		}

		std::string recv;
		std::vector<std::string> gpsparams;
		recv = buf;
		gpsparams = split(recv,":");

		printf("operation:%s\n",buf);

		if((gpsparams.size() >= 1) && (gpsparams[0].compare("takeoff") == 0))
		{
			state = MissionState::STARTING_PROPS;
		}
		//////// test
/*
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
*/
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
        float vel_target = 0.75;   //m/sec
        float accel_max  = 1.5;   //m/sec/sec
        float stopping_accel = 1.0; //m/sec/sec

        static double t_last = 0;

        static float vel_x_des_sent = 0;
        static float vel_y_des_sent = 0;
        static float vel_z_des_sent = 0;

        float command_diff_x;
        float command_diff_y;

        float vel_x_target;
        float vel_y_target;
        //float vel_z_target;
//        float yaw_target;

        yaw_vel_des = 0;

        command_diff_x = CalcAxisDistance((float)posDestination.longitude/1e7 ,
			(float)posCurrent.longitude/1e7);

        command_diff_y = CalcAxisDistance((float)posDestination.latitude/1e7 ,
			(float)posCurrent.latitude/1e7);

		distance_to_dest =CalcDistance(
			(float)posDestination.latitude/1e7,
			(float)posDestination.longitude/1e7,
			(float)posCurrent.latitude/1e7,
			(float)posCurrent.longitude/1e7);

        if(command_diff_x >300 || command_diff_y >300) //about 300m
		{
			printf(" too far !!! command_diff_x:%f command_diff_y:%f",command_diff_x,command_diff_y);
			state = MissionState::LOITER;
			continue;
        }

        float command_mag = sqrt(command_diff_x*command_diff_x +
             command_diff_y*command_diff_y);

		printf(" command_diff_x:%f command_diff_y:%f",command_diff_x,command_diff_y);
		printf("distance_to_dest :%f",distance_to_dest);

        if(distance_to_dest < 2.5) //about 150*150   x,y 1.5m
        {
			//if it is on the last waypoint then slow down before stopping
			float stopping_vel = sqrt(2*stopping_accel*command_mag);
			if(stopping_vel<vel_target) { vel_target = stopping_vel; }
        }

        if(distance_to_dest<1.5) // 0.5m
        {
            state = MissionState::LOITER;
        }

        if(command_mag<0.01){command_mag = 0.01;}  //to prevent dividing by zero

        vel_x_target = command_diff_x/command_mag * vel_target;
        vel_y_target = command_diff_y/command_mag * vel_target;
        //vel_z_target = 0;//command_dir_z/command_mag * vel_target;

		if(command_diff_x <0.75f) vel_x_target =0;
		if(command_diff_y <0.75f) vel_y_target =0;

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
        //float vel_z_diff = (vel_z_target - vel_z_des_sent);

        float vel_diff_mag = sqrt(vel_x_diff*vel_x_diff +
                                  vel_y_diff*vel_y_diff );

        if(vel_diff_mag<v_del_max)
        {
          //send through the target velocity
          vel_x_des_sent = vel_x_target;
          vel_y_des_sent = vel_y_target;
          vel_z_des_sent = 0;
        }
        else
        {
          //converge to the target velocity at the max acceleration rate
          vel_x_des_sent += vel_x_diff/vel_diff_mag * v_del_max;
          vel_y_des_sent += vel_y_diff/vel_diff_mag * v_del_max;
          //vel_z_des_sent += vel_z_diff/vel_diff_mag * v_del_max;
        }

        x_vel_des = vel_x_des_sent;
        y_vel_des = vel_y_des_sent;
        z_vel_des = vel_z_des_sent;
      }

      else if (state == MissionState::LOITER )
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
            state = MissionState::LOITER;
          }
		  printf("LOITER, before get gps data\n");
		  // blocking until message is received
		  if ((numbytes = recvfrom(sockfd, buf, MAXBUFLEN-1 , 0,
				  (struct sockaddr *)&their_addr, &addr_len)) == -1)
		  {
			printf("cant get gps data\n");
			entering_loiter = true;
			state = MissionState::LOITER;
			continue;
		  }
		  printf("LOITER, after gps data\n");

		  std::string recv;
		  std::vector<std::string> gpsparams;
		  recv = buf;
		  gpsparams = split(recv,":");
		  if(gpsparams.size() >= 1 && (gpsparams[0].compare("landing") ==0))
		  {
			 state = MissionState::LANDING;
		  }
		  else if(gpsparams.size() >= 4)
		  {
			posDestination.latitude = atoi(gpsparams[2].c_str());
			posDestination.longitude = atoi(gpsparams[1].c_str());
			destyaw = (float)atof(gpsparams[3].c_str());

			distance_to_dest =CalcDistance(
				(float)posLast.latitude/1e7,
				(float)posLast.longitude/1e7,
				(float)posCurrent.latitude/1e7,
				(float)posCurrent.longitude/1e7);

			printf("LOITER posDetination gps latitude,longitude,destyaw,curyaw:%d,%d,%f,%f\n",
				posDestination.latitude,posDestination.longitude,destyaw,yaw_est);

			if(posDestination.latitude !=0 && posDestination.longitude !=0
				&&distance_to_dest>1.5f) //more than 0.5m
			{
				state = MissionState::TRAJECTORY_FOLLOW;
				posLast = posCurrent;
				entering_loiter = true;
				printf("get gps data,TRAJECTORY_FOLLOW\n");
			}
			else
				state = MissionState::LOITER;
		  }

		float yaw_diff = destyaw - yaw_est;

		if((yaw_diff >0.5f && yaw_diff <3.1415f) ||
			(yaw_diff >-5.7832f && yaw_diff <-3.1415f) )
		    yaw_vel_des = 0.5f;
		else if((yaw_diff <=-0.5f && yaw_diff >-3.1415f )||
			(yaw_diff <5.7832f && yaw_diff >3.1415f))
		    yaw_vel_des =  -0.5f;
		else if( (yaw_diff >0.05f && yaw_diff <=0.5f)
			|| (yaw_diff <=-5.7832f && yaw_diff >-6.2830f))
			yaw_vel_des = 0.15f;
		else if( (yaw_diff <-0.05f && yaw_diff >-0.5f)
			|| (yaw_diff >=5.7832f && yaw_diff <6.2830f))
			yaw_vel_des = -0.15f;
		else
			yaw_vel_des = 0;

		//yaw_vel_des = 0.25;

		printf("yaw_diff:%f  yaw_vel_des:%f\n",yaw_diff,yaw_vel_des);

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
      sn_apply_cmd_mapping(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
      //sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
          x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des,
          &cmd0, &cmd1, &cmd2, &cmd3);

      // Send the commands if in the right mode.
      sn_send_rc_command(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
      //sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
                         cmd0, cmd1, cmd2, cmd3);

      // Print some information
      if(mode == SN_GPS_POS_HOLD_MODE){printf("\n[%d] SN_GPS_POS_HOLD_MODE. ",loop_counter);}
      else if(mode == SN_SENSOR_ERROR_MODE){printf("\n[%d] SENSOR ERROR MODE. ",loop_counter);}
	  else if(mode == SN_OPTIC_FLOW_POS_HOLD_MODE){printf("\n[%d] OPTIC FLOW VELOCITY MODE MODE. ",loop_counter);}
      else{printf("\n[%d] UNDEFINED MODE. ",loop_counter);}

      if(props_state == SN_PROPS_STATE_NOT_SPINNING){printf("Propellers NOT spinning\n");}
      else if(props_state == SN_PROPS_STATE_STARTING){printf("Propellers attempting to spin\n");}
      else if (props_state == SN_PROPS_STATE_SPINNING){printf("Propellers spinning\n");}
      else{printf("Unknown propeller state\n");}

      printf("[%d] commanded rates: [%f,%f,%f,%f]\n",loop_counter,x_vel_des_yawed,y_vel_des_yawed,z_vel_des,yaw_vel_des);
      printf("[%d] battery_voltage: %f\n",loop_counter,voltage);
	  printf("[%d] [x_est,y_est,z_est,yaw_est]: [%f,%f,%f,%f]\n",loop_counter,x_est-x_est_startup,y_est-y_est_startup,z_est-z_est_startup,yaw_est);

      if (state == MissionState::ON_GROUND)
        printf("[%d] ON_GROUND\n",loop_counter);
      else if (state == MissionState::STARTING_PROPS)
        printf("[%d] STARTING_PROPS\n",loop_counter);
      else if (state == MissionState::TAKEOFF)
	  {
        printf("[%d] TAKEOFF\n",loop_counter);
		printf("[%d] position_est_startup: [%f,%f,%f]\n",loop_counter,x_est_startup,y_est_startup,z_est_startup);
	  }
      else if (state == MissionState::TRAJECTORY_FOLLOW)
      {
        printf("[%d] TRAJECTORY_FOLLOW\n",loop_counter);
		printf("[%d] [posCurrent.latitude,posCurrent.latitude]: [%d,%d]\n",loop_counter,posCurrent.latitude,posCurrent.longitude);
		printf("[%d] [x_des,y_des,z_des,yaw_des]: [%f,%f,%f,%f]\n",loop_counter,x_des-x_est_startup,y_des-y_est_startup,z_des-z_est_startup,yaw_des);
      }
      else if (state == MissionState::LOITER)
        printf("[%d] LOITER\n",loop_counter);
      else if (state == MissionState::LANDING)
        printf("[%d] LANDING\n",loop_counter);
      else
        printf("[%d] STATE UNKNOWN\n",loop_counter);

		static double t_start = 0;
		//if(t_start != 0)
		printf("last comand time: %f\n",(t_now - t_start));
		t_start = t_now;

    }

    loop_counter++;
    usleep(10000);
  }

  if (mission_success)
  {
    printf("Trajectory Following Mission was completed successfully.\n");
  }
  else
  {
    printf("Trajectory Following Mission was aborted.\n");
  }

  close(sockfd);
  return 0;
}


