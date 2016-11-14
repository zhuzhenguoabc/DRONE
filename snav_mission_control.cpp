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
#include <sys/un.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <sys/socket.h>


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
  float yaw;
};

struct GpsPosition
{
  int latitude;   // xx.xxxxxx
  int longitude;   //xxx.xxxxx
  int altitude;   //
  float yaw;
};

//cuiyc face detect
struct body_info
{
  bool have_face;
  bool have_body;
  int  body_flag; //1000 upperbody 1001 fullbody
  bool newP;
  float distance;   // m
  float angle;   // m
};

struct body_info cur_body;
const float safe_distance = 2.0f;
const float min_angle_offset = 0.05f;

//cuiyc  face detect

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
  const float EARTH_RADIUS = 6378137;//r =6378.137km earth radius

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

//people detect begin
void* getVideoFaceFollowParam(void*)
{
	//video follow socket begin
	const char* UNIX_DOMAIN ="/tmp/vedio.domain";
	static int recv_php_buf[16];
	static int recv_php_num=0;

	const float window_degree = 72.7768f;
	const float camera_rad =window_degree*M_PI/180; //camera 83.5 degree
	const float face_winth =0.150; //camera 83.5 degree
	int listen_fd;
	int ret=0;
	int i;

	socklen_t len;
	struct sockaddr_un clt_addr;
	struct sockaddr_un srv_addr;
	len=sizeof(clt_addr);
	while(1)
	{
		 //listen_fd=socket(AF_UNIX,SOCK_STREAM,0);
		 listen_fd=socket(AF_UNIX,SOCK_DGRAM,0);
		 if(listen_fd<0)
		 {
		  printf("cannot create listening socket");
		  continue;
		 }
		 else
		 {
			  while(1)
			  {
			   srv_addr.sun_family=AF_UNIX;
			   strncpy(srv_addr.sun_path,UNIX_DOMAIN,sizeof(srv_addr.sun_path)-1);
			   unlink(UNIX_DOMAIN);
			   ret=bind(listen_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));
			   if(ret==-1)
			   {
				printf("cannot bind server socket");
				//close(listen_fd);
				unlink(UNIX_DOMAIN);
				break;
			   }
			   while(true)
			   {
				recv_php_num = recvfrom(listen_fd, recv_php_buf, sizeof(recv_php_buf),
					0, (struct sockaddr *)&clt_addr, &len);
				printf("\n=====face info===== ");
				for(i=0;i<16;i++) //0,flag 1,center-x 2,center-y,3,face_winth,4,_face_height,5,video_winth,6,video_height
				printf("%d ",recv_php_buf[i]);
				printf("\n");

				if(recv_php_buf[0] == 1)
				{
					// normal  face 18cm && camera 83.5 degree
					float distance,angle;
					cur_body.have_face=true;

					if(recv_php_buf[6] == 720) //720p
						distance = face_winth/tan((recv_php_buf[3]*camera_rad/1280));
					else if(recv_php_buf[6] == 1080) //1080p
						distance = face_winth/tan((recv_php_buf[3]*camera_rad/1920));
					else if(recv_php_buf[5] != 0)
						distance = face_winth/tan((recv_php_buf[3]*camera_rad/recv_php_buf[5])/2);

					//dgree for window 72.7768
					angle = window_degree*((recv_php_buf[5]/2 - recv_php_buf[1])*1.0)/recv_php_buf[5];
					printf("face distance :%f angle:%f \n",distance,angle);

					if((cur_body.angle != angle) || (cur_body.distance != distance))
					{
						cur_body.distance = distance;
						cur_body.angle 	= angle;
						cur_body.newP = true;
					}
					else
						cur_body.newP = false;

				}
				else
					cur_body.have_face=false;

			   }
			  }
		 }
	}
	//video follow socket end
}

void* getVideoBodyFollowParam(void*)
{
	//video follow socket begin
	const char* UNIX_DOMAIN ="/tmp/vedio.body.domain";
	static int recv_php_buf[16];
	static int recv_php_num=0;

	const float window_degree = 72.7768f;
	const float camera_rad =window_degree*M_PI/180; //camera 83.5 degree
	const float upper_body_winth =0.70; //camera 83.5 degree
	const float body_winth =1.0; //camera 83.5 degree

	int listen_fd;
	int ret=0;
	int i;

	socklen_t len;
	struct sockaddr_un clt_addr;
	struct sockaddr_un srv_addr;
	len=sizeof(clt_addr);
	while(1)
	{
		 //listen_fd=socket(AF_UNIX,SOCK_STREAM,0);
		 listen_fd=socket(AF_UNIX,SOCK_DGRAM,0);
		 if(listen_fd<0)
		 {
			printf("cannot create listening socket");
			continue;
		 }
		 else
		 {
			  while(1)
			  {
			   srv_addr.sun_family=AF_UNIX;
			   strncpy(srv_addr.sun_path,UNIX_DOMAIN,sizeof(srv_addr.sun_path)-1);
			   unlink(UNIX_DOMAIN);
			   ret=bind(listen_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));
			   if(ret==-1)
			   {
				printf("cannot bind server socket");
				//close(listen_fd);
				unlink(UNIX_DOMAIN);
				break;
			   }
			   while(true)
			   {
				recv_php_num = recvfrom(listen_fd, recv_php_buf, sizeof(recv_php_buf),
					0, (struct sockaddr *)&clt_addr, &len);
				printf("\n=====body info===== ");
				for(i=0;i<16;i++) //0,flag 1,center-x 2,center-y,3,body_winth,4,body_height,5,body flag 1000=halfbody 1001=fullbody
				printf("%d ",recv_php_buf[i]);
				printf("\n");

				if(recv_php_buf[0] == 1 && !cur_body.have_face)
				{
					// normal  face 18cm && camera 83.5 degree
					float distance,angle;
					cur_body.have_body=true;

					if(recv_php_buf[5] == 1000) //upperbody
					{
						distance = upper_body_winth/tan((recv_php_buf[3]*camera_rad/640));
						cur_body.body_flag =1000;
					}
					else if(recv_php_buf[5] == 1001) //pedestrian
					{
						distance = body_winth/tan((recv_php_buf[3]*camera_rad/640));
						cur_body.body_flag =1001;
					}

					//dgree for window 72.7768
					angle = window_degree*((640/2 - recv_php_buf[1])*1.0)/640;
					printf("body distance :%f angle:%f \n",distance,angle);

					if((cur_body.angle != angle) || (cur_body.distance != distance))
					{
						cur_body.distance = distance;
						cur_body.angle 	= angle;
						cur_body.newP = true;
					}
					else
						cur_body.newP = false;

				}
				else //if we have face ,set no body
					cur_body.have_body=false;

				//close(com_fd);
			   }
			  }
		 }
	}
	//video follow socket end
}
//people detect end

//people detect end


int main(int argc, char* argv[])
{
  // Desired takeoff altitude
  float kDesTakeoffAlt = 1.5;  // m

  // Fixed takeoff and landing speed
  const float kLandingSpeed = -0.75;  // m/s
  const float kTakeoffSpeed = 0.75;   // m/s
  float distance_to_home;
  bool circle_misson=false;
  static bool calcCirclePoint = false;
  bool gps_waypiont_mission = false;
  bool gps_point_collect_mission = false;
  bool followme_mission=false;

  // Time to loiter
  const float kLoiterTime = 3;
  //socket
  const char* ip_address = "192.168.1.1";
  int port = 14664;

  SnRcCommandType curSendMode=SN_RC_OPTIC_FLOW_POS_HOLD_CMD;

  //gps params
  GpsPosition posLast;
  GpsPosition posGpsCurrent;
  GpsPosition posGpsDestination;
  float destyaw = 0;
  float speed = 0;
  float distance_to_dest=0;

  //gps postion fly
  std::vector<GpsPosition> gps_positions;
  const float GPS_ACCURACY = 1.0;

  //circle fly
  std::vector<Position> circle_positions;
  float radius = 2.5f;//meter
  float vel_target = 0.75;	 //m/sec
  int point_count = 72;
  float angle_per = 2*M_PI/point_count;
  int clockwise= 1;// anticlockwise = -1

  MissionState state = MissionState::UNKNOWN;
  bool mission_in_progress = true;
  bool mission_success = false;
  int loop_counter = 0;

  //socket
  int sockfd;
  struct sockaddr_in my_addr; // my address information
  struct sockaddr_in their_addr; // connector's address information
  socklen_t addr_len;
  int numbytes;
  char buf[MAXBUFLEN];

  //people detect cuiyc begin
  bool face_mission=false;
  bool body_mission=false;

  pthread_t id;
  int retF,retB;
/*
  retF=pthread_create(&id,NULL,getVideoFaceFollowParam,NULL);

  if(retF!=0){
  	printf ("Create pthread FaceFollowParam error!\n");
  	exit (1);
  }
*/
  retB=pthread_create(&id,NULL,getVideoBodyFollowParam,NULL);

  if(retB!=0){
  	printf ("Create pthread BodyFollow error!\n");
  	exit (1);
  }
  //people detect cuiyc end

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

  // Begin mission loop
  //while (mission_in_progress)
  while (true)
  {
    // Always need to call this
    if (sn_update_data() != 0)
    {
      printf("sn_update_data failed\n");
      mission_in_progress = false;
    }
    else
    {
	  if(gps_waypiont_mission || gps_point_collect_mission)
	  {
		  int gps_enabled;
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
	  }

	std::string recv;
	std::vector<std::string> gpsparams;

	if(!circle_misson && !gps_waypiont_mission){
		// blocking until message is received
		if((numbytes = recvfrom(sockfd, buf, MAXBUFLEN-1 , 0,
			(struct sockaddr *)&their_addr, &addr_len)) == -1)
		{
			printf("cant get operration\n");
			exit(1);
		}

		 recv = buf;
		 gpsparams = split(recv,":");
		 //printf("operation:%s\n",buf);
	 }
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
		  posGpsCurrent.latitude = (int)snav_data->gps_0_raw.latitude;
		  posGpsCurrent.longitude = (int)snav_data->gps_0_raw.longitude;
		  posGpsCurrent.altitude = (int)snav_data->gps_0_raw.altitude;
		  posGpsCurrent.yaw = (float)snav_data->high_level_control_data.yaw_estimated;
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
      //if (mode == SN_GPS_POS_HOLD_MODE)
      if (mode == SN_OPTIC_FLOW_POS_HOLD_MODE || mode == SN_GPS_POS_HOLD_MODE)
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
      static float yaw_est_startup = 0;

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

		if(gpsparams.size() >= 1)
		{
			if(gpsparams[0].compare("takeoff") == 0)
			{
				state = MissionState::STARTING_PROPS;
			}
			else if(gpsparams[0].compare("circle") == 0)
			{
				state = MissionState::STARTING_PROPS;
				curSendMode =SN_RC_OPTIC_FLOW_POS_HOLD_CMD;
				circle_misson = true;
				calcCirclePoint = true;
				kDesTakeoffAlt = atof(gpsparams[1].c_str());
			    radius = atof(gpsparams[2].c_str());
			    vel_target = atof(gpsparams[3].c_str());
				printf("ON_GROUND circle: kDesTakeoffAlt,radius,vel_target:%f,%f,%f\n",
				kDesTakeoffAlt,radius,vel_target);
				clockwise =1;
			}
			else if(gpsparams[0].compare("gps_waypiont") == 0)
			{
				state = MissionState::STARTING_PROPS;

				if(gps_positions.size()>2)
				gps_waypiont_mission = true;
				curSendMode =SN_RC_GPS_POS_HOLD_CMD;
			}
			else if(gpsparams[0].compare("followme") == 0)
			{
				//state = MissionState::STARTING_PROPS;
				curSendMode =SN_RC_GPS_POS_HOLD_CMD;
				kDesTakeoffAlt = 2.5;
			}
			else if(gpsparams[0].compare("gps_point_collect") == 0)
			{
				state = MissionState::ON_GROUND;
				gps_point_collect_mission = true;
				curSendMode =SN_RC_GPS_POS_HOLD_CMD;
			}
			else if(gpsparams[0].compare("gps_point_collect_finish") == 0)
			{
				state = MissionState::ON_GROUND;
				printf("gps_point_collect_finish.\n");
				gps_point_collect_mission = false;

				for(int k=0;k<gps_positions.size();k++){
					printf("gps position collect finished #%u: [%d,%d,%d,%f] ",
					k,gps_positions[k].latitude,gps_positions[k].longitude,
					gps_positions[k].altitude,gps_positions[k].yaw);
				}
			}
			else
				state = MissionState::ON_GROUND;

			if(gps_point_collect_mission)
			{
				 GpsPosition pos;
				 pos.latitude = posGpsCurrent.latitude;
				 pos.longitude = posGpsCurrent.longitude;
				 pos.altitude = posGpsCurrent.altitude;
				 pos.yaw = posGpsCurrent.yaw;

				 if(posGpsDestination.latitude !=0 && posGpsDestination.longitude !=0)
				 gps_positions.push_back(pos);

				 gps_point_collect_mission = false;
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

          yaw_est_startup = yaw_est;
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
          //if (snav_data->sonar_0_raw.range >1.45 && snav_data->sonar_0_raw.range <1.7)
          {
            state = MissionState::LOITER;
          }
		  else if(z_des - z_est_startup > 0.8f*kDesTakeoffAlt)
		  //else if(snav_data->sonar_0_raw.range > 0.8f*kDesTakeoffAlt)
		  {
			z_vel_des = kTakeoffSpeed*0.5f;
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

		  circle_misson=false;
		  calcCirclePoint = false;
		  gps_waypiont_mission = false;
 		  gps_point_collect_mission = false;
		  followme_mission = false;
		  face_mission = false;
		  body_mission=false;
		  break;
        }
      }

      else if(state == MissionState::TRAJECTORY_FOLLOW)
      {
        float accel_max  = 1.5;   //m/sec/sec
        float stopping_accel = 1.0; //m/sec/sec

        static double t_last = 0;

        static float vel_x_des_sent = 0;
        static float vel_y_des_sent = 0;
        static float vel_z_des_sent = 0;
        static float vel_yaw_des_sent = 0;

        float command_diff_x;
        float command_diff_y;
        float command_diff_z;
        float command_diff_yaw;

        static float vel_x_target = 0;
        static float vel_y_target = 0;
        static float vel_z_target = 0;
        static float vel_yaw_target = 0;

        yaw_vel_des = 0;

		if(gpsparams.size() >= 1 && (gpsparams[0].compare("landing") ==0))
		{
		   state = MissionState::LANDING;
		   continue;
		}

		if(gps_waypiont_mission && gps_positions.size()>2)
		{
	        command_diff_x = CalcAxisDistance((float)(gps_positions[current_position].longitude)/1e7 ,
				(float)posGpsCurrent.longitude/1e7);

	        command_diff_y = CalcAxisDistance((float)(gps_positions[current_position].latitude)/1e7 ,
				(float)posGpsCurrent.latitude/1e7);

			distance_to_dest =CalcDistance(
				(float)(gps_positions[current_position].latitude)/1e7,
				(float)(gps_positions[current_position].longitude)/1e7,
				(float)posGpsCurrent.latitude/1e7,
				(float)posGpsCurrent.longitude/1e7);

			if(distance_to_dest<0.01){distance_to_dest = 0.01;}  //to prevent dividing by zero

			command_diff_z = kDesTakeoffAlt - (z_des-z_est_startup);

			vel_x_target = command_diff_x/distance_to_dest * vel_target;
			vel_y_target = command_diff_y/distance_to_dest * vel_target;
			vel_z_target = command_diff_z/distance_to_dest * vel_target;

			if(distance_to_dest < 2*GPS_ACCURACY )
			{
			//if it is on the last waypoint then slow down before stopping
			float stopping_vel = sqrt(2*stopping_accel*distance_to_dest);
			if(stopping_vel<vel_target) { vel_target = stopping_vel; }
			}

			if(distance_to_dest < GPS_ACCURACY) // 1m
			{
			    // Close enough, move on
				current_position++;
				if (current_position >= gps_positions.size())
				{
					// No more circle_positions, so land
				    state = MissionState::LANDING;
					current_position =0;
				}
			}
		}
		else if(circle_misson)
		{
			command_diff_x = circle_positions[current_position].x - (x_des-x_est_startup);
			command_diff_y = circle_positions[current_position].y - (y_des-y_est_startup);
			command_diff_z = circle_positions[current_position].z - (z_des-z_est_startup);
			command_diff_yaw = circle_positions[current_position].yaw - yaw_des;

			if (command_diff_yaw > M_PI) command_diff_yaw = command_diff_yaw - 2*M_PI;
			else if (command_diff_yaw < -M_PI) command_diff_yaw = command_diff_yaw+ 2*M_PI;

			distance_to_dest = sqrt(command_diff_x*command_diff_x +
									 command_diff_y*command_diff_y +
									 command_diff_z*command_diff_z);

			if(distance_to_dest<0.01){distance_to_dest = 0.01;}  //to prevent dividing by zero

			if(distance_to_dest<0.04 /*&& abs(command_diff_yaw) <angle_per*/)
			{
			  // Close enough, move on
			  current_position++;
			  if (current_position >= circle_positions.size())
			  {
				// No more circle_positions, so land
				state = MissionState::LOITER;
				current_position =0;
				circle_misson=false;
		  		calcCirclePoint = false;
			  }
			}
			vel_x_target = command_diff_x/distance_to_dest * vel_target;
			vel_y_target = command_diff_y/distance_to_dest * vel_target;
			vel_z_target = command_diff_z/distance_to_dest * vel_target;
			vel_yaw_target = command_diff_yaw/angle_per*vel_target;
			printf("[%d] [distance_to_dest command_diff_x command_diff_y command_diff_z command_diff_yaw]: [%f %f %f %f %f]\n",
				loop_counter,distance_to_dest,command_diff_x,command_diff_y,command_diff_z,command_diff_yaw);

			//if (current_position >= 0.9*circle_positions.size()) //slow down
			//	vel_yaw_target =0;
			//if(vel_yaw_target>0.8f)vel_yaw_target=0.8f;
		}
		else if(followme_mission && gpsparams[0].compare("followme") == 0)
		{
			//curSendMode =SN_RC_GPS_POS_HOLD_CMD;
			float yaw_diff ;
			float mobile_gps_Lonti = (float)atoi(gpsparams[3].c_str())/1e7;
			float mobile_gps_Lati = (float)atoi(gpsparams[4].c_str())/1e7;

	        command_diff_x = CalcAxisDistance(mobile_gps_Lonti,
				(float)posGpsCurrent.longitude/1e7);

	        command_diff_y = CalcAxisDistance(mobile_gps_Lati,
				(float)posGpsCurrent.latitude/1e7);

			distance_to_dest =CalcDistance(
				(float)mobile_gps_Lati,
				(float)mobile_gps_Lonti,
				(float)posGpsCurrent.latitude/1e7,
				(float)posGpsCurrent.longitude/1e7);

			printf("[%d] followme [distance_to_dest command_diff_x command_diff_y ]: [%f %f %f]\n",
					loop_counter,distance_to_dest,command_diff_x,command_diff_y);

			destyaw = (float)atof(gpsparams[1].c_str());
			speed = (float)atof(gpsparams[2].c_str());
			yaw_diff = destyaw - yaw_est;

			if(speed == 0 && abs(yaw_diff)<0.05f
				&& distance_to_dest<GPS_ACCURACY)
			{
				state = MissionState::LOITER;
				printf("followme TRAJECTORY_FOLLOW-> LOITER\n" );
				continue;
			}

			if(yaw_diff > M_PI)
			yaw_diff = yaw_diff -2*M_PI;
			else if (yaw_diff < -M_PI)
			yaw_diff = yaw_diff + 2*M_PI;

			vel_yaw_target = (yaw_diff)*vel_target;

			command_diff_z = kDesTakeoffAlt - (z_des-z_est_startup);

			if(snav_data->sonar_0_raw.range < 0.5f*kDesTakeoffAlt)
				command_diff_z = kDesTakeoffAlt - snav_data->sonar_0_raw.range;

			if(speed == 0 && distance_to_dest<GPS_ACCURACY)
			{
				vel_x_target =0;
				vel_y_target = 0;
			}
			else if(speed == 0 && distance_to_dest>GPS_ACCURACY)
			{
				vel_x_target = command_diff_x/distance_to_dest * vel_target;
				vel_y_target = command_diff_y/distance_to_dest * vel_target;
			}
			else
			{
				vel_x_target = speed*cos(destyaw);
				vel_y_target = speed*sin(destyaw);
			}
			vel_z_target = command_diff_z/kDesTakeoffAlt * vel_target;

			printf("[%d] [vel_x_target vel_y_target vel_z_target vel_yaw_target]: [%f %f %f %f]\n",
				loop_counter,vel_x_target,vel_y_target,vel_z_target,vel_yaw_target);
		}
	    else if(face_mission) //cuiyc add face detect begin
		{
			//static float f_dest_yaw,f_dest_x,f_dest_y;
			static float distance_remain_x,distance_remain_y;
			static float forword_dis , parallel_dis,angle_face_offset;

			if(cur_body.newP)
			{
				forword_dis = cur_body.distance-safe_distance;
				angle_face_offset = cur_body.angle*M_PI/180;

				parallel_dis = tan(angle_face_offset)*cur_body.distance;

				distance_to_dest = sqrt(forword_dis*forword_dis + parallel_dis*parallel_dis);

				printf("[%d] face_mission newP [forword_dis parallel_dis distance_to_dest ]: [%f %f %f]\n",
						loop_counter,forword_dis,parallel_dis,distance_to_dest);

				distance_remain_x = cos(yaw_est)*forword_dis-sin(yaw_est)*parallel_dis;
				distance_remain_y = sin(yaw_est)*forword_dis+cos(yaw_est)*parallel_dis;
				cur_body.newP = false;
			}

			if(((abs(angle_face_offset))< min_angle_offset && abs(distance_to_dest) <0.05f)
				|| !cur_body.have_face)
			{
				state = MissionState::LOITER;
				printf(" face_mission follow face-> LOITER\n" );
				face_mission = false;
				continue;
			}
			vel_yaw_target = 0;

			//for test rotate
			distance_remain_x = distance_remain_x - vel_x_des_sent*0.02f; //20ms a tick
			distance_remain_y = distance_remain_y - vel_y_des_sent*0.02f;

			distance_to_dest = sqrt(distance_remain_x*distance_remain_x +
									 distance_remain_y*distance_remain_y);

			printf("[%d] face_mission [distance_remain_x distance_remain_y]: [%f %f]\n",
					loop_counter,distance_remain_x,distance_remain_y);

			if(abs(distance_remain_x) <0.05f)
				distance_remain_x =0;
			if(abs(distance_remain_y) <0.05f)
				distance_remain_y =0;

			if(abs(distance_to_dest) >0.05f)
			{
				vel_x_target = distance_remain_x* vel_target*1.5;
				vel_y_target = distance_remain_y* vel_target*1.5;
				if(vel_x_target >vel_target) vel_x_target =vel_target;
				if(vel_y_target >vel_target) vel_y_target =vel_target;
			}
			else
			{
				vel_x_target = 0;
				vel_y_target = 0;
			}

			printf("[%d] face_mission [vel_x_target vel_y_target distance_to_dest vel_yaw_target]: [%f %f %f %f]\n",
				loop_counter,vel_x_target,vel_y_target,distance_to_dest,vel_yaw_target);
		}
		else if(body_mission)
		{
			//static float f_dest_yaw,f_dest_x,f_dest_y;
			static float distance_remain_x,distance_remain_y;
			static float forword_dis , parallel_dis,angle_face_offset;

			if(cur_body.newP)
			{
				forword_dis = cur_body.distance-safe_distance;
				angle_face_offset = cur_body.angle*M_PI/180;

				parallel_dis = tan(angle_face_offset)*cur_body.distance;

				distance_to_dest = sqrt(forword_dis*forword_dis + parallel_dis*parallel_dis);

				printf(" [%d] body_mission newP [forword_dis parallel_dis distance_to_dest ]: [%f %f %f]\n",
						loop_counter,forword_dis,parallel_dis,distance_to_dest);

				distance_remain_x = cos(yaw_est)*forword_dis-sin(yaw_est)*parallel_dis;
				distance_remain_y = sin(yaw_est)*forword_dis+cos(yaw_est)*parallel_dis;

				cur_body.newP = false;
			}

			if(((abs(angle_face_offset))< min_angle_offset && abs(distance_to_dest) <0.15f)
				|| (!cur_body.have_body) )
			{
				state = MissionState::LOITER;
				printf("body_mission follow face-> LOITER\n" );
				face_mission = false;
				continue;
			}

			vel_yaw_target = 0;

			distance_remain_x = distance_remain_x - vel_x_des_sent*0.02f; //20ms a tick
			distance_remain_y = distance_remain_y - vel_y_des_sent*0.02f;

			distance_to_dest = sqrt(distance_remain_x*distance_remain_x +
									 distance_remain_y*distance_remain_y);

			printf(" [%d] body_mission [distance_remain_x distance_remain_y]: [%f %f]\n",
					loop_counter,distance_remain_x,distance_remain_y);

			if(abs(distance_remain_x) <0.05f)
				distance_remain_x =0;
			if(abs(distance_remain_y) <0.05f)
				distance_remain_y =0;


			if(abs(distance_to_dest) >0.05f)
			{
				vel_x_target = distance_remain_x* vel_target*1.5;
				vel_y_target = distance_remain_y* vel_target*1.5;
				if(vel_x_target >vel_target) vel_x_target =vel_target;
				if(vel_y_target >vel_target) vel_y_target =vel_target;
			}
			else
			{
				vel_x_target = 0;
				vel_y_target = 0;
			}

			/*command_diff_z = kDesTakeoffAlt - (z_des-z_est_startup);

			if(abs(command_diff_z)>0.05)
				vel_z_target = command_diff_z/kDesTakeoffAlt * vel_target;
			else
				vel_z_target =0;*/

			printf(" [%d]  body_mission [vel_x_target vel_y_target distance_to_dest vel_yaw_target]: [%f %f %f %f]\n",
				loop_counter,vel_x_target,vel_y_target,distance_to_dest,vel_yaw_target);
		}//cuiyc add face detect begin

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
        float vel_yaw_diff = (vel_yaw_target - vel_yaw_des_sent);

        float vel_diff_mag = sqrt(vel_x_diff*vel_x_diff +
                                  vel_y_diff*vel_y_diff +
                                  vel_z_diff*vel_z_diff);

		if(vel_diff_mag<0.01){vel_diff_mag = 0.01;}

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

		//smooth accel
		if(vel_yaw_diff<v_del_max)
			vel_yaw_des_sent = vel_yaw_target;
		else
			vel_yaw_des_sent += v_del_max;

		distance_to_home = sqrt((x_est_startup-x_est)*(x_est_startup-x_est)+
				(y_est_startup-y_est)*(y_est_startup-y_est)+
				(z_est_startup-z_est)*(z_est_startup-z_est));
		//todo ...height and distance limited

        yaw_vel_des = vel_yaw_des_sent;

        x_vel_des = vel_x_des_sent;
        y_vel_des = vel_y_des_sent;
        z_vel_des = vel_z_des_sent;

		printf("[%d] [x_vel_des,y_vel_des,z_vel_des,yaw_vel_des]: [%f,%f,%f,%f] \n",
			loop_counter,x_vel_des,y_vel_des,z_vel_des,yaw_vel_des);
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

		  if(gpsparams.size() >= 4 && gpsparams[0].compare("circle") == 0)
		  {
			  curSendMode =SN_RC_OPTIC_FLOW_POS_HOLD_CMD;
			  circle_misson = true;
			  calcCirclePoint = true;
			  kDesTakeoffAlt = atof(gpsparams[1].c_str());
			  radius = atof(gpsparams[2].c_str());
			  vel_target = atof(gpsparams[3].c_str());
			  printf("LOITER circle: kDesTakeoffAlt,radius,vel_target:%f,%f,%f\n",
				kDesTakeoffAlt,radius,vel_target);
		  }

		  if(gpsparams.size() >= 1 && (gpsparams[0].compare("landing") ==0))
		  {
			 state = MissionState::LANDING;
		  }
		  else if(gps_waypiont_mission && gps_positions.size()>2)
		  {
			//posGpsDestination.latitude = atoi(gpsparams[2].c_str());
			//posGpsDestination.longitude = atoi(gpsparams[1].c_str());
			//destyaw = (float)atof(gpsparams[3].c_str());

			posGpsDestination.latitude = gps_positions[current_position].latitude;
			posGpsDestination.longitude = gps_positions[current_position].longitude;
			destyaw = gps_positions[current_position].yaw;

			distance_to_dest =CalcDistance(
				(float)posGpsDestination.latitude/1e7,
				(float)posGpsDestination.longitude/1e7,
				(float)posGpsCurrent.latitude/1e7,
				(float)posGpsCurrent.longitude/1e7);

			printf("LOITER posGpsDestination gps latitude,longitude,destyaw,curyaw:%d,%d,%f,%f\n",
				posGpsDestination.latitude,posGpsDestination.longitude,destyaw,yaw_est);

			if(posGpsDestination.latitude !=0 && posGpsDestination.longitude !=0
				&&distance_to_dest>1.5f) //more than 0.5m
			{
				state = MissionState::TRAJECTORY_FOLLOW;
				posLast = posGpsCurrent;
				entering_loiter = true;
				printf("get gps data,TRAJECTORY_FOLLOW\n");
			}
			else
				state = MissionState::LOITER;

		  }
		  else if(circle_misson)
		  {
			if (t_now - t_loiter_start > kLoiterTime)
			{

				if(calcCirclePoint){
					//circle center
					float circle_center_x;
					float circle_center_y;
					float yaw_t =0;

					circle_center_x = x_est-x_est_startup + radius*cos(yaw_est);
					circle_center_y = y_est-yaw_est_startup + radius*sin(yaw_est);

					circle_positions.clear();//clear and recaculate.
					for(int k=0;k<=point_count;k++){
						Position pos;
						//pos.x = (1-cos(angle_per*k))*radius + x_est;
						//pos.y = -sin(angle_per*k)*radius + y_est;
						pos.z = kDesTakeoffAlt;
						yaw_t = yaw_est+angle_per*k*clockwise;

						if(yaw_t > M_PI)
							yaw_t = yaw_t -2*M_PI;
						else if (yaw_t < -M_PI)
							yaw_t = yaw_t + 2*M_PI;

						if(yaw_t>0){
							pos.x = cos(yaw_t-M_PI)*radius + circle_center_x;
							pos.y = sin(yaw_t-M_PI)*radius + circle_center_y;
						}
						else
						{
							pos.x = cos(yaw_t+M_PI)*radius + circle_center_x;
							pos.y = sin(yaw_t+M_PI)*radius + circle_center_y;
						}
						pos.yaw = yaw_t;

						circle_positions.push_back(pos);
					}
					calcCirclePoint = false;

					for(int k=0;k<=circle_positions.size();k++){
					    printf("@@@@[%d] position #%u: [%f,%f,%f,%f]\n",k,
							k,circle_positions[k].x,circle_positions[k].y,
							circle_positions[k].z,circle_positions[k].yaw);
					}
					//break;
				}

				state = MissionState::TRAJECTORY_FOLLOW;
			    entering_loiter = true;
			}
		  }
		  else if(gpsparams[0].compare("followme") == 0)
		  {
		  		//curSendMode =SN_RC_GPS_POS_HOLD_CMD;
				followme_mission = true;
		  		destyaw = (float)atof(gpsparams[1].c_str());
				speed = (float)atof(gpsparams[2].c_str());
				kDesTakeoffAlt = 2.5;

				float yaw_diff ;
				yaw_diff = destyaw - yaw_est;

				if(abs(yaw_diff) >0.05f || speed != 0)
				{
					state = MissionState::TRAJECTORY_FOLLOW;
					entering_loiter = true;
					printf("yaw_diff:%f	speed:%f\n",yaw_diff,speed);
					printf("followme LOITER -> TRAJECTORY_FOLLOW\n" );
				}
		  }
		  else if(cur_body.have_face) //cuiyc add face detect begin
		  {
			float face_offset ;
			face_offset = M_PI*cur_body.angle/180;
			printf("followme have_face\n" );

			if(abs(face_offset) >0.055f || cur_body.distance>(safe_distance+0.15)
				|| cur_body.distance < (safe_distance-0.15))
			{
				face_mission = true;
				state = MissionState::TRAJECTORY_FOLLOW;
				entering_loiter = true;
				printf("face_offset:%f	distance:%f\n",face_offset,cur_body.distance);
				printf("followme have_face LOITER -> FACE_FOLLOW\n" );
			}
		  }
		  else if(cur_body.have_body)
		  {
			float body_offset ;
			body_offset = M_PI*cur_body.angle/180;
			printf("followme have_body\n" );

			if(abs(body_offset) >0.055f || cur_body.distance>(safe_distance+0.15)
				|| cur_body.distance < (safe_distance-0.15))
			{
				body_mission= true;
				state = MissionState::TRAJECTORY_FOLLOW;
				entering_loiter = true;
				printf("face_offset:%f	distance:%f\n",body_offset,cur_body.distance);
				printf("followme have_body LOITER -> FACE_FOLLOW\n" );
			}
		  }//cuiyc add face detect end

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
      //sn_apply_cmd_mapping(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
      //sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,//curSendMode
      sn_apply_cmd_mapping(curSendMode, RC_OPT_LINEAR_MAPPING,
          x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des,
          &cmd0, &cmd1, &cmd2, &cmd3);

      // Send the commands if in the right mode.
      //sn_send_rc_command(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
      //sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
      sn_send_rc_command(curSendMode, RC_OPT_LINEAR_MAPPING,
                         cmd0, cmd1, cmd2, cmd3);

      // Print some information
      if(mode == SN_GPS_POS_HOLD_MODE){printf("\n[%d] SN_GPS_POS_HOLD_MODE. ",loop_counter);}
      else if(mode == SN_SENSOR_ERROR_MODE){printf("\n[%d] SENSOR ERROR MODE. ",loop_counter);}
	  else if(mode == SN_OPTIC_FLOW_POS_HOLD_MODE){printf("\n[%d] OPTIC FLOW VELOCITY MODE MODE. ",loop_counter);}
      else{printf("\n[%d] UNDEFINED MODE :%d \n",loop_counter,mode);}

      if(props_state == SN_PROPS_STATE_NOT_SPINNING){printf("Propellers NOT spinning\n");}
      else if(props_state == SN_PROPS_STATE_STARTING){printf("Propellers attempting to spin\n");}
      else if (props_state == SN_PROPS_STATE_SPINNING){printf("Propellers spinning\n");}
      else{printf("Unknown propeller state\n");}

      printf("[%d] commanded rates: [%f,%f,%f,%f]\n",loop_counter,x_vel_des_yawed,y_vel_des_yawed,z_vel_des,yaw_vel_des);
      //printf("[%d] battery_voltage: %f\n",loop_counter,voltage);
	  printf("[%d] [x_est,y_est,z_est,yaw_est]: [%f,%f,%f,%f]\n",loop_counter,x_est-x_est_startup,y_est-y_est_startup,z_est-z_est_startup,yaw_est);
	  //printf("[%d] [x_des,y_des,z_des,yaw_des]: [%f,%f,%f,%f]\n",
	//	  loop_counter,x_des-x_est_startup,y_des-y_est_startup,z_des-z_est_startup,yaw_des);

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
		if(gps_waypiont_mission || followme_mission)
			printf("[%d] [posGpsCurrent.latitude,posGpsCurrent.latitude]: [%d,%d]\n",
			loop_counter,posGpsCurrent.latitude,posGpsCurrent.longitude);

		if(circle_misson)
			printf("[%d] circle_misson position #%u: [%f,%f,%f,%f]\n",loop_counter,
            current_position,circle_positions[current_position].x,
            circle_positions[current_position].y, circle_positions[current_position].z,
            circle_positions[current_position].yaw);
      }
      else if (state == MissionState::LOITER)
        printf("[%d] LOITER\n",loop_counter);
      else if (state == MissionState::LANDING)
        printf("[%d] LANDING\n",loop_counter);
      else
        printf("[%d] STATE UNKNOWN\n",loop_counter);

		static double t_start = 0;
		//printf("[%d] last comand time: %f\n",loop_counter,(t_now - t_start));
		t_start = t_now;

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

  close(sockfd);
  return 0;
}


