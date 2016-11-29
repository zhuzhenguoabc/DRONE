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
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <string>
#include <sys/un.h>
#include <stddef.h>

// Waypoint utilities
#include "snav_waypoint_utils.hpp"
// Snapdragon Navigator
#include "snapdragon_navigator.h"

using namespace std;


#define __DEBUG

#ifdef __DEBUG
#define DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define DEBUG(format,...)
#endif



#define LOG_FILE "/home/linaro/dev/examples/log_snav_transfer_control.txt"
#define VERSION_NUM "1.0.1"

#define MAX_SOCKET_CONNECTION 5
#define SOCKET_OVER_TIME 5
#define SERVER_TCP_PORT 14558
#define SERVER_UDP_PORT 14559

#define MAX_BUFF_LEN 512
#define HEADER_BUFF_LENGTH 4

#define MIN_GPS_POSITION_NUM 2
#define MAX_GPS_POSITION_NUM 10

#define STR_SEPARATOR  ","

#define HEART_BEAT_OVER_COUNT 3	//6
#define SNAV_HEART_BEAT "heartbeat"

#define SNAV_CMD_CONROL						"1000"

#define SNAV_CMD_TAKE_OFF					"1001"
#define SNAV_CMD_LAND						"1002"
#define SNAV_CMD_RETURN						"1003"
#define SNAV_CMD_CIRCLE						"1004"
#define SNAV_CMD_TRAIL_NAVIGATION			"1005"
#define SNAV_CMD_GPS_FOLLOW					"1006"
#define SNAV_CMD_MODIFY_SSID_PWD			"1025"
#define SNAV_CMD_FACE_FOLLOW  				"1100"
#define SNAV_CMD_BODY_FOLLOW  				"1101"

#define SNAV_CMD_RETURN_TAKE_OFF			"2001"
#define SNAV_CMD_RETURN_LAND				"2002"
#define SNAV_CMD_RETURN_RETURN				"2003"
#define SNAV_CMD_RETURN_CIRCLE				"2004"
#define SNAV_CMD_RETURN_TRAIL_NAVIGATION	"2005"
#define SNAV_CMD_RETURN_GPS_FOLLOW 			"2006"
#define SNAV_CMD_RETURN_MODIFY_SSID_PWD 	"2025"
#define SNAV_CMD_RETURN_FACE_FOLLOW 		"2100"
#define SNAV_CMD_RETURN_BODY_FOLLOW 		"2101"

#define SNAV_TASK_GET_INFO				"8001"
#define SNAV_TASK_GET_VERSION			"8002"

#define SNAV_TASK_GET_INFO_RESULT		"9001"
#define SNAV_TASK_GET_VERSION_RESULT	"9002"

typedef unsigned char byte;

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

// States of Drone
enum class DroneState
{
  NORMAL,
  MOTOR_ERROR,
  CPU_OVER_HEAT,
  IMU_ERROR,
  BARO_ERROR,
  MAG_ERROR,
  GPS_ERROR,
  SONAR_ERROR,
  OPTIC_FLOW_ERROR,
  MODE_ERROR
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

struct NavigationPosition
{
  float latitude;
  float longitude;
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
static bool face_follow_switch = false;
static bool body_follow_switch = false;
const float safe_distance = 2.0f;
const float min_angle_offset = 0.05f;
//cuiyc  face detect

//ensure only one tcp connection exist
bool tcp_receive_thread_flag = false;
bool tcp_send_thread_flag = false;

typedef struct
{
	int client_sockfd;
}client_arg;

struct prodcons {
    char data[MAX_BUFF_LEN];
	bool bflag;
	bool bSockExit;
	int fd_socket;
    pthread_mutex_t lock;
    pthread_cond_t flag_circle;
	pthread_cond_t flag_handler;
};

struct prodcons pro_tcp_receive;
struct prodcons pro_tcp_send;

struct prodcons pro_udp_receive;


struct timeval timeout_tcp = {SOCKET_OVER_TIME,0};	//5s
struct timeval timeout_udp = {0,300000};			//300ms


vector<string> split(const string& s, const string& delim)
{
    vector<string> elems;

    size_t pos = 0;
    size_t len = s.length();
    size_t delim_len = delim.length();

    if (delim_len == 0)
	{
		return elems;
    }
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

float CalcDistance(float fLati1, float fLong1, float fLati2, float fLong2)
{
	const float EARTH_RADIUS = 6378137;

	double radLat1 = rad(fLati1);
	double radLat2 = rad(fLati2);
	double lati_diff = radLat1 - radLat2;
	double long_diff = rad(fLong1) - rad(fLong2);
	double s = 2*asin(sqrt(pow(sin(lati_diff/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(long_diff/2),2)));
	s = s*EARTH_RADIUS;
	return s;
}

float CalcAxisDistance(float f1,  float f2)
{
  const float EARTH_RADIUS = 6378137.0; //r =6378.137km earth radius
  const float PI = 3.1415926;

  double s =(f1-f2) * PI * EARTH_RADIUS / 180.0; //  n*pi*r/180
  return s;
}


void init_prodcon(struct prodcons *b) {
	b->bflag = false;
	b->bSockExit = false;
	b->fd_socket = -1;
	memset(b->data,0,MAX_BUFF_LEN);
    pthread_mutex_init(&b->lock, NULL);
    pthread_cond_init(&b->flag_circle, NULL);
	pthread_cond_init(&b->flag_handler, NULL);
}

void get_ip_address(unsigned long address,char* ip)
{
	sprintf(ip,"%d.%d.%d.%d",address>>24,(address&0xFF0000)>>24,(address&0xFF00)>>24,address&0xFF);
}

//high byte first
int bytesToInt(byte src[], int offset)
{
    int value;

    value = (int) ( ((src[offset] & 0xFF)<<24)
            |((src[offset+1] & 0xFF)<<16)
            |((src[offset+2] & 0xFF)<<8)
            |(src[offset+3] & 0xFF));

    return value;
}

void* circle_receive(void* arg)
{
	int count = 0;

    client_arg* args=(client_arg*)arg;
    int sockfd=args->client_sockfd;

	int header_length = 0;
	int data_length = 0;
	byte buff_header[HEADER_BUFF_LENGTH];
	byte buff_data[MAX_BUFF_LEN];

	int header_heart_beat_mission_count = 0;
	int data_heart_beat_mission_count = 0;

	DEBUG("\ncircle_receive start!\n");

	pthread_mutex_lock(&pro_tcp_receive.lock);
	pro_tcp_receive.bSockExit = false;
	pthread_mutex_unlock(&pro_tcp_receive.lock);

	while(true)
	{
		//receive the header(length of the data)
		if ((header_length=recv(sockfd,buff_header,HEADER_BUFF_LENGTH,0))>0)
		{
			data_length = bytesToInt(buff_header, 0);

			if ((data_length < 0) || (data_length > 500))
			{
				continue;
			}

			DEBUG("\n[%d]:circle_receive: header_length=%d, data_length=%d\n", count,header_length, data_length);
			//fflush(stdout);

			header_heart_beat_mission_count = 0;

			if (data_length > 0)
			{
				int i_recv_data_length = 0;
				bool b_recv_data_result = false;

				while(!b_recv_data_result)
				{
					i_recv_data_length=recv(sockfd,buff_data,data_length,0);

					if (i_recv_data_length > 0)
					{
						b_recv_data_result = true;

						data_heart_beat_mission_count = 0;

						buff_data[i_recv_data_length]='\0';

						pthread_mutex_lock(&pro_tcp_receive.lock);
						//pthread_cond_wait(&pro_tcp_receive.flag_circle, &pro_tcp_receive.lock);

						//set the flag and data for the snav-handle-thread check
						pro_tcp_receive.bflag = true;
						pro_tcp_receive.fd_socket = sockfd;
						memset(pro_tcp_receive.data,0,MAX_BUFF_LEN);
						memcpy(pro_tcp_receive.data, buff_data, MAX_BUFF_LEN);

					    //pthread_cond_signal(&pro_tcp_receive.flag_handler);
					    pthread_mutex_unlock(&pro_tcp_receive.lock);

						//avoid heartbeat and 8001 override the pro_tcp_receive.data(takeoff or landing)
						if ((strcmp(pro_tcp_receive.data, SNAV_HEART_BEAT) != 0)
							&& (strcmp(pro_tcp_receive.data, SNAV_TASK_GET_INFO) != 0)
							&& (strcmp(pro_tcp_receive.data, SNAV_TASK_GET_VERSION) != 0))
						{
							usleep(200000);	//200ms
						}
					}
					else if ((i_recv_data_length < 0)
							&& (errno == EINTR
								|| errno == EWOULDBLOCK
								|| errno == EAGAIN))
					{
						DEBUG("[%d]circle_receive recv return i_recv_data_length=%d, errno=%d\n", count,i_recv_data_length, errno);

						data_heart_beat_mission_count++;

						if (data_heart_beat_mission_count >= HEART_BEAT_OVER_COUNT)
						{
							DEBUG("circle_receive quit from data_heart_beat over times!\n\n");

							pthread_mutex_lock(&pro_tcp_receive.lock);
							pro_tcp_receive.bSockExit = true;
							pthread_mutex_unlock(&pro_tcp_receive.lock);

							close(sockfd);
							tcp_receive_thread_flag = false;
    						pthread_exit(NULL);
						}
						else
						{
							continue;
						}
					}
					else
					{
						DEBUG("circle_receive return i_recv_data_length=%d, errno=%d\n", i_recv_data_length, errno);
						DEBUG("circle_receive quit from receive_data_socket closed!\n\n");

						pthread_mutex_lock(&pro_tcp_receive.lock);
						pro_tcp_receive.bSockExit = true;
						pthread_mutex_unlock(&pro_tcp_receive.lock);

						close(sockfd);
						tcp_receive_thread_flag = false;
						pthread_exit(NULL);
					}
				}
			}
		}
		else if ((header_length < 0)
					&& (errno == EINTR
						|| errno == EWOULDBLOCK
						|| errno == EAGAIN))
		{
			DEBUG("\ncircle_receive return header_length=%d, errno=%d\n", header_length, errno);

			header_heart_beat_mission_count++;

			if (header_heart_beat_mission_count >= HEART_BEAT_OVER_COUNT)
			{
				DEBUG("circle_receive quit from header_heart_beat over times!\n\n");

				pthread_mutex_lock(&pro_tcp_receive.lock);
				pro_tcp_receive.bSockExit = true;
				pthread_mutex_unlock(&pro_tcp_receive.lock);

				close(sockfd);
				tcp_receive_thread_flag = false;
				pthread_exit(NULL);
			}
			else
			{
				continue;
			}
		}
		else
		{
			DEBUG("\ncircle_receive return header_length=%d, errno=%d\n", header_length, errno);
			DEBUG("circle_receive quit from receive_header_socket closed!\n\n");

			pthread_mutex_lock(&pro_tcp_receive.lock);
			pro_tcp_receive.bSockExit = true;
			pthread_mutex_unlock(&pro_tcp_receive.lock);

			close(sockfd);
			tcp_receive_thread_flag = false;
			pthread_exit(NULL);
		}

		count++;
	}

	DEBUG("\ncircle_receive quit from while break!\n\n");

	pthread_mutex_lock(&pro_tcp_receive.lock);
	pro_tcp_receive.bSockExit = true;
	pthread_mutex_unlock(&pro_tcp_receive.lock);

	close(sockfd);
	tcp_receive_thread_flag = false;
    pthread_exit(NULL);
}


void* circle_send(void* arg)
{
    client_arg* args=(client_arg*)arg;
    int sockfd=args->client_sockfd;

	int header_heart_beat_mission_count = 0;
	int data_heart_beat_mission_count = 0;

	//for send to client
	char result_to_client[MAX_BUFF_LEN];
	bool bLocalTcpFlag = false;
	int localTcpFd = -1;
	bool bLocalSocketFlag = false;

	int count = 0;

	byte buff_data[MAX_BUFF_LEN];

	pthread_mutex_lock(&pro_tcp_receive.lock);
	pro_tcp_receive.bSockExit = false;
	pthread_mutex_unlock(&pro_tcp_receive.lock);

	DEBUG("\ncircle_send start!\n");

	while(true)
	{
		pthread_mutex_lock(&pro_tcp_receive.lock);
		bLocalSocketFlag = pro_tcp_receive.bSockExit;
		pthread_mutex_unlock(&pro_tcp_receive.lock);

		if (bLocalSocketFlag)
		{
			DEBUG("[%d]:circle_send quit from circle_receive socket closed!\n\n",count);

			pthread_mutex_lock(&pro_tcp_receive.lock);
			pro_tcp_receive.bSockExit = false;
			pthread_mutex_unlock(&pro_tcp_receive.lock);

			close(sockfd);
			tcp_send_thread_flag = false;
			pthread_exit(NULL);
		}

		pthread_mutex_lock(&pro_tcp_send.lock);
		bLocalTcpFlag = pro_tcp_send.bflag;

		if (pro_tcp_send.bflag == true)
		{
			localTcpFd = pro_tcp_send.fd_socket;

			DEBUG("\n[%d]:circle_send fd_socket=%d, pro_tcp_send=%s\n", count,pro_tcp_send.fd_socket, pro_tcp_send.data);

			memset(result_to_client,0,MAX_BUFF_LEN);
			memcpy(result_to_client, pro_tcp_send.data, MAX_BUFF_LEN);
			memset(pro_tcp_send.data,0,MAX_BUFF_LEN);

			pro_tcp_send.bflag = false;
		}
	    pthread_mutex_unlock(&pro_tcp_send.lock);

		usleep(20000);	//avoid of cpu over usage

		/*
		//no block handle
		if (bLocalTcpFlag)
		{
			int a = strlen(result_to_client);
			a = htonl(a);

			send(localTcpFd,(const void*)&a,HEADER_BUFF_LENGTH,0);
			send(localTcpFd,result_to_client,strlen(result_to_client),0);

			bLocalTcpFlag = false;
		}
		*/

		if (bLocalTcpFlag)
		{
			int i_send_header_length = 0;
			int i_send_data_length = 0;
			bool b_send_header_result = false;
			bool b_send_data_result = false;

			int a = strlen(result_to_client);
			a = htonl(a);

			//send header_length and string data
			while(!b_send_header_result)
			{
				i_send_header_length=send(localTcpFd,(const void*)&a,HEADER_BUFF_LENGTH,0);

				if (i_send_header_length > 0)
				{
					b_send_header_result = true;

					header_heart_beat_mission_count = 0;

					while(!b_send_data_result)
					{
						i_send_data_length = send(localTcpFd,result_to_client,strlen(result_to_client),0);

						if (i_send_data_length > 0)
						{
							b_send_data_result = true;

							data_heart_beat_mission_count = 0;

							DEBUG("circle_send i_send_data_length=%d, errno=%d\n\n", i_send_data_length, errno);
						}
						else if ((i_send_data_length < 0)
									&& (errno == EINTR
										|| errno == EWOULDBLOCK
										|| errno == EAGAIN))
						{
							DEBUG("circle_send i_send_data_length=%d, errno=%d\n\n", i_send_data_length, errno);

							data_heart_beat_mission_count++;

							if (data_heart_beat_mission_count >= HEART_BEAT_OVER_COUNT)
							{
								DEBUG("circle_send quit from data_heart_beat_count over times!\n\n");

								close(sockfd);
								tcp_send_thread_flag = false;
	    						pthread_exit(NULL);
							}
							else
							{
								continue;
							}
						}
						else
						{
							DEBUG("circle_send return i_send_data_length=%d, errno=%d\n", i_send_data_length, errno);
							DEBUG("circle_send quit from send_data_socket closed!\n\n");

							close(sockfd);
							tcp_send_thread_flag = false;
							pthread_exit(NULL);
						}
					}
				}
				else if ((i_send_header_length < 0)
							&& (errno == EINTR
								|| errno == EWOULDBLOCK
								|| errno == EAGAIN))
				{
					DEBUG("circle_send i_send_header_length=%d, errno=%d\n\n", i_send_header_length, errno);

					header_heart_beat_mission_count++;

					if (header_heart_beat_mission_count >= HEART_BEAT_OVER_COUNT)
					{
						DEBUG("circle_send quit from header_heart_beat_count over times!\n\n");

						close(sockfd);
						tcp_send_thread_flag = false;
						pthread_exit(NULL);
					}
					else
					{
						continue;
					}
				}
				else
				{
					DEBUG("circle_send return i_send_header_length=%d, errno=%d\n", i_send_header_length, errno);
					DEBUG("circle_send quit from send_header_socket closed!\n\n");

					close(sockfd);
					tcp_send_thread_flag = false;
					pthread_exit(NULL);
				}
			}

			bLocalTcpFlag = false;
		}

		count++;
	}

	DEBUG("circle_send quit from while break!\n\n");

	close(sockfd);
	tcp_send_thread_flag = false;
    pthread_exit(NULL);
}

void* snav_handler(void* arg)
{
	char udp_receive_data[MAX_BUFF_LEN];
	char tcp_receive_data[MAX_BUFF_LEN];
	char result_to_client[MAX_BUFF_LEN];

	bool bLocalTcpFlag = false;
	int localTcpFd = -1;

	bool bLocalUdpFlag = false;

	//people detect cuiyc begin
	bool face_mission=false;
	bool body_mission=false;

	// Desired takeoff altitude
	float kDesTakeoffAlt = 1.2;	//1.5;  // m

	float fTrarilHeight = 1.8;	// m

	// Fixed takeoff and landing speed
	const float kLandingSpeed = -0.75;  // m/s
	const float kTakeoffSpeed = 0.75;   // m/s
	float distance_to_home;
	bool circle_mission=false;
	static bool calcCirclePoint = false;

	bool trail_navigation_mission = false;
	bool gps_waypiont_mission = false;
	bool gps_point_collect_mission = false;
	bool followme_mission=false;

	//for return_home_mission
	bool return_mission=false;
	bool fly_home=false;
	bool gohome_mission = false;
	float gohome_x_vel_des = 0;
	float gohome_y_vel_des = 0;
	float gohome_z_vel_des = 0;
	float gohome_yaw_vel_des = 0;

	bool wp_goal=false;
	float yaw_target_home, distance_home_squared;
	float distance_home_squared_threshold = 1;
	FlatVars output_vel;

	// Position at startup
	static float x_est_startup = 0;
	static float y_est_startup = 0;
	static float z_est_startup = 0;
	static float yaw_est_startup = 0;

	// Mission State Machine
	static size_t current_position = 0;

	// Time to loiter
  	const float kLoiterTime = 3;

	//gps params
	GpsPosition posLast;
	GpsPosition posGpsCurrent;
	GpsPosition posGpsDestination;
	float destyaw = 0;
	float speed = 0;
	float distance_to_dest=0;
	SnRcCommandType curSendMode=SN_RC_OPTIC_FLOW_POS_HOLD_CMD;

	//gps postion fly
	vector<GpsPosition> gps_positions;

	vector<NavigationPosition> trail_navigation_positions;

	//circle fly
	vector<Position> circle_positions;
	float radius = 2.5f;//meter

	float vel_target = 0.75;	 //m/sec
	int point_count = 72;
	float angle_per = 2*M_PI/point_count;

	int clockwise= 1;// anticlockwise = -1

	DroneState drone_state = DroneState::NORMAL;
	MissionState state = MissionState::UNKNOWN;
	int loop_counter = 0;

	static bool task_take_off_in_progress = true;

	DEBUG("snav_handler start!\n");

	SnavCachedData* snav_data = NULL;
	if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
	{
		DEBUG("\nFailed to get flight data pointer!\n");
		return NULL;
	}

	//udp for cmd
	int server_udp_sockfd;
	int server_udp_len;
    struct sockaddr_in server_udp_address;

    server_udp_address.sin_family=AF_INET;
    server_udp_address.sin_addr.s_addr=htonl(INADDR_ANY);
    server_udp_address.sin_port=htons(SERVER_UDP_PORT);
    server_udp_len=sizeof(server_udp_address);

	server_udp_sockfd=socket(AF_INET,SOCK_DGRAM,0);

    int bind_result = bind(server_udp_sockfd,(struct sockaddr*)&server_udp_address,server_udp_len);

	setsockopt(server_udp_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout_udp, sizeof(struct timeval));
	setsockopt(server_udp_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout_udp, sizeof(struct timeval));

	// Begin loop
	while (true)
	{
		int length = 0;
		struct sockaddr_in remote_addr;
		int sin_size;
	    char udp_buff_data[MAX_BUFF_LEN];
		char last_udp_buff_data[MAX_BUFF_LEN];

		sin_size=sizeof(struct sockaddr_in);

		//receive the udp data
		length=recvfrom(server_udp_sockfd,udp_buff_data,MAX_BUFF_LEN-1,0, (struct sockaddr *)&remote_addr,(socklen_t*)&sin_size);

		if (length>0)
		{
			struct timeval time_val;
			gettimeofday(&time_val, NULL);
			double time_now = time_val.tv_sec + time_val.tv_usec * 1e-6;

			//DEBUG("\nudp recvfrom received packet from %s:\n",inet_ntoa(remote_addr.sin_addr));
			udp_buff_data[length]='\0';
			DEBUG("\nudp recvfrom get data udp_buff_data=%s, time_now=%lf\n",udp_buff_data, time_now);

			bLocalUdpFlag = true;
		}
		else
		{
			struct timeval time_val;
			gettimeofday(&time_val, NULL);
			double time_now = time_val.tv_sec + time_val.tv_usec * 1e-6;

			DEBUG("\nudp recvfrom return length=%d, errno=%d, time_now=%lf\n", length, errno, time_now);

			bLocalUdpFlag = false;
		}

		//pro_tcp_receive for task operation
	    pthread_mutex_lock(&pro_tcp_receive.lock);
		bLocalTcpFlag = pro_tcp_receive.bflag;
		if (pro_tcp_receive.bflag == true)
		{
			localTcpFd = pro_tcp_receive.fd_socket;

			struct timeval time_val;
			gettimeofday(&time_val, NULL);
			double time_now = time_val.tv_sec + time_val.tv_usec * 1e-6;

			DEBUG("\nhandler prodcons fd_socket=%d, pro_tcp_receive=%s, time_now=%lf\n",
					pro_tcp_receive.fd_socket, pro_tcp_receive.data, time_now);

			memset(tcp_receive_data,0,MAX_BUFF_LEN);
			memcpy(tcp_receive_data, pro_tcp_receive.data, MAX_BUFF_LEN);

			pro_tcp_receive.bflag = false;
		}
	    pthread_mutex_unlock(&pro_tcp_receive.lock);


		// Always need to call this
		if (sn_update_data() != 0)
		{
			DEBUG("sn_update_data failed\n");
		}
		else
		{
			// Get the current mode
			SnMode mode;
			mode = (SnMode)snav_data->general_status.current_mode;

			// Get the current state of the propellers
			SnPropsState props_state;
			props_state = (SnPropsState) snav_data->general_status.props_state;

			// Get the source of the RC input (spektrum vs API) here
      		SnRcCommandSource rc_cmd_source = (SnRcCommandSource)(snav_data->rc_active.source);

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

			//check the drone status
			drone_state = DroneState::NORMAL;

			CpuStats cpu_status = snav_data->cpu_stats;
			for(int j = 0; j < 10; j++)
			{
				if (cpu_status.temp[j] >= 78)
				{
					drone_state = DroneState::CPU_OVER_HEAT;
				}
				break;
			}

			if (props_state == SN_PROPS_STATE_UNKNOWN)
			{
				drone_state = DroneState::MOTOR_ERROR;
			}

			SnDataStatus imu_status = (SnDataStatus) snav_data->data_status.imu_0_status;
			SnDataStatus baro_status = (SnDataStatus) snav_data->data_status.baro_0_status;
			SnDataStatus mag_status = (SnDataStatus) snav_data->data_status.mag_0_status;
			SnDataStatus gps_status = (SnDataStatus) snav_data->data_status.gps_0_status;
			SnDataStatus sonar_status = (SnDataStatus) snav_data->data_status.sonar_0_status;
			SnDataStatus optic_flow_status = (SnDataStatus) snav_data->data_status.optic_flow_0_status;

			/*
			if (mag_status != SN_DATA_VALID)
			{
				drone_state = DroneState::MAG_ERROR;
			}

			if (gps_status != SN_DATA_VALID)
			{
				drone_state = DroneState::GPS_ERROR;
			}

			if (baro_status != SN_DATA_VALID)
			{
				drone_state = DroneState::BARO_ERROR;
			}
			*/

			if (sonar_status != SN_DATA_VALID)
			{
				drone_state = DroneState::SONAR_ERROR;
			}

			if (optic_flow_status != SN_DATA_VALID)
			{
				drone_state = DroneState::OPTIC_FLOW_ERROR;
			}

			if (imu_status != SN_DATA_VALID)
			{
				drone_state = DroneState::IMU_ERROR;
			}
			//check end

			string recv_tcp_cmd, recv_udp_cmd;
			vector<string> gpsparams_tcp, gpsparams_udp;

			//get the data from tcp socket(task data) and udp socket(control data)
			if (bLocalTcpFlag)
			{
				recv_tcp_cmd = tcp_receive_data;
				gpsparams_tcp = split(recv_tcp_cmd,STR_SEPARATOR);
				DEBUG("tcp task operation:%s\n\n",tcp_receive_data);
			}

			if (bLocalUdpFlag)
			{
				recv_udp_cmd = udp_buff_data;	//udp_receive_data;
				gpsparams_udp = split(recv_udp_cmd,STR_SEPARATOR);
				DEBUG("udp control operation:%s\n",udp_buff_data);
			}

			DEBUG("task_take_off_in_progress:%d\n",task_take_off_in_progress);

			//udp control operation
			if (/*!task_take_off_in_progress
				&& */bLocalUdpFlag
				&& (gpsparams_udp.size() >= 6)
				&& (gpsparams_udp[0].compare(SNAV_CMD_CONROL)==0))
			{
				if (bLocalTcpFlag
					&& (gpsparams_tcp.size() >= 1)
					&& ((gpsparams_tcp[0].compare(SNAV_CMD_TAKE_OFF)==0)
					    || (gpsparams_tcp[0].compare(SNAV_CMD_LAND)==0)
					    /*|| (gpsparams_tcp[0].compare(SNAV_TASK_GET_INFO)==0)*/))
				{
					if (((props_state == SN_PROPS_STATE_NOT_SPINNING) && (gpsparams_tcp[0].compare(SNAV_CMD_TAKE_OFF)==0))
						|| ((props_state == SN_PROPS_STATE_SPINNING) && (gpsparams_tcp[0].compare(SNAV_CMD_LAND)==0)))
					{
						task_take_off_in_progress = true;
					}
					//leave the data to task module.
				}
				else if ((gpsparams_udp[1].compare("0")==0)
						&& (gpsparams_udp[2].compare("0")==0)
						&& (gpsparams_udp[3].compare("0")==0)
						&& (gpsparams_udp[4].compare("500")==0)
						&& (gpsparams_udp[5].compare("0")==0))
				{
					//discard the data. go to the task handle
				}
				else
				{
					//for snav control
					int rolli=-1;
		      		int pitchi=-1;
		      		int yawi=-1;
		      		int thrusti=-1;
		      		int buttons=-1;
					static bool landing_in_progress = false;

					const float kMin = -1;
					const float kMax = 1;
					float cmd0 = 0;
					float cmd1 = 0;
					float cmd2 = 0;
					float cmd3 = 0;

					// Use type to control how the commands get interpreted.
		      		SnRcCommandType type = SN_RC_OPTIC_FLOW_POS_HOLD_CMD;

					DEBUG("Control operation! props_state=%d\n", props_state);

					rolli = atoi(gpsparams_udp[1].c_str());
					pitchi = atoi(gpsparams_udp[2].c_str());
					yawi = atoi(gpsparams_udp[3].c_str());
					thrusti = atoi(gpsparams_udp[4].c_str());
					buttons = atoi(gpsparams_udp[5].c_str());

					DEBUG("SNAV_CMD_CONROL rolli,pitchi,yawi,thrusti,buttons:%d,%d,%d,%d,%d\n",
										rolli,pitchi,yawi,thrusti,buttons);

					if (props_state == SN_PROPS_STATE_SPINNING)
					{
						if (buttons == 1)	//landing
						{
							landing_in_progress = true;
						}
						else
						{
							landing_in_progress = false;
						}

						cmd0 = -((float)(pitchi+441)*(kMax-kMin)/882.+ kMin);
						cmd1 = -((float)(rolli+441)*(kMax-kMin)/882.+ kMin);
						cmd2 = (float)(thrusti)*(kMax-kMin)/1000.+ kMin;
						cmd3 = -((float)(yawi+250)*(kMax-kMin)/500.+ kMin);

						if (landing_in_progress)
				        {
				          // If user touches roll/pitch stick, stop landing
				          if (fabs(cmd0) > 1e-4 || fabs(cmd1) > 1e-4)
				          {
				            landing_in_progress = false;
				          }
				          else
				          {
				            cmd0 = 0;
				            cmd1 = 0;
				            cmd2 = -0.5;	//-0.75;
				            cmd3 = 0;
				          }
				        }

						DEBUG("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
						DEBUG("UDP SNAV_SEND_CMD cmd0,cmd1,cmd2,cmd3:%f,%f,%f,%f\n",
										cmd0,cmd1,cmd2,cmd3);
						DEBUG("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n");

						// Send the commands to Snapdragon Navigator with default RC options
		        		sn_send_rc_command(type, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);

						//udp control when takoff/landing/follow, need change the state
						if ((state == MissionState::TAKEOFF)
							|| (state == MissionState::LANDING)
							|| (state == MissionState::TRAJECTORY_FOLLOW))
						{
							if (state == MissionState::TRAJECTORY_FOLLOW)
							{
								current_position =0;
								circle_mission=false;
								calcCirclePoint = false;
								trail_navigation_mission = false;

								//people detect cuiyc begin
								face_mission=false;
								body_mission=false;
							}

							state = MissionState::LOITER;
						}
					}

					//for heartbeat send back
					if (bLocalTcpFlag)
					{
						pthread_mutex_lock(&pro_tcp_send.lock);

						//set the flag and data for the snav_handler-thread check
						pro_tcp_send.bflag = true;
						pro_tcp_send.fd_socket = localTcpFd;

						if (strlen(pro_tcp_send.data) > 0)
						{
							usleep(20000);
						}

						memset(pro_tcp_send.data,0,MAX_BUFF_LEN);

						DEBUG("[%d]:prepare pro_tcp_send to send back to client\n",loop_counter);

						if (strcmp(tcp_receive_data, SNAV_HEART_BEAT) == 0)	/*tcp heartbeat*/
						{
							memcpy(pro_tcp_send.data, tcp_receive_data, MAX_BUFF_LEN);
						}
						else /*Other tcp task*/
						{
							memset(result_to_client,0,MAX_BUFF_LEN);

							/*
							sprintf(result_to_client,"ignore tcp task(%s) when received udp control command!",tcp_receive_data);
							memcpy(pro_tcp_send.data, result_to_client, MAX_BUFF_LEN);
							*/

							//tcp 8001 task for get status and data information when udp control
							if ((gpsparams_tcp.size() >= 1) && (gpsparams_tcp[0].compare(SNAV_TASK_GET_INFO) == 0))
							{
								sprintf(result_to_client,"%s",SNAV_TASK_GET_INFO_RESULT);

								char battery_info[MAX_BUFF_LEN];
								char rpm_info[MAX_BUFF_LEN];
								char sonar_info[MAX_BUFF_LEN];
								char gps_info[MAX_BUFF_LEN];
								char xyz_info[MAX_BUFF_LEN];
								char rpy_info[MAX_BUFF_LEN];
								char flight_state_info[MAX_BUFF_LEN];
								char drone_state_info[MAX_BUFF_LEN];

								memset(battery_info,0,MAX_BUFF_LEN);
								memset(rpm_info,0,MAX_BUFF_LEN);
								memset(sonar_info,0,MAX_BUFF_LEN);
								memset(gps_info,0,MAX_BUFF_LEN);
								memset(xyz_info,0,MAX_BUFF_LEN);
								memset(rpy_info,0,MAX_BUFF_LEN);
								memset(flight_state_info,0,MAX_BUFF_LEN);
								memset(drone_state_info,0,MAX_BUFF_LEN);


								sprintf(battery_info,"battery_info:%f",snav_data->general_status.voltage);
								DEBUG("battery_info=%s\n",battery_info);


								sprintf(rpm_info,"rpm_info:%d:%d:%d:%d",snav_data->esc_raw.rpm[0], snav_data->esc_raw.rpm[1]
																		   ,snav_data->esc_raw.rpm[2], snav_data->esc_raw.rpm[3]);
								DEBUG("rpm_info=%s\n",rpm_info);

								sprintf(sonar_info,"sonar_info:%f",snav_data->sonar_0_raw.range);
								DEBUG("sonar_info=%s\n",sonar_info);


								int gps_enabled;
								sn_is_gps_enabled(&gps_enabled);

								if(gps_enabled != 1)
								{
									sprintf(gps_info, "gps_info:gps is not enabled!");
								}
								else
								{
									SnDataStatus gps_status = (SnDataStatus) snav_data->data_status.gps_0_status;
									if (gps_status != SN_DATA_VALID)
									{
										sprintf(gps_info, "gps_info:can not get gps location!");
									}
									else
									{
										sprintf(gps_info, "gps_info:%d:%d",snav_data->gps_0_raw.longitude, snav_data->gps_0_raw.latitude);
									}
								}
								DEBUG("gps_info=%s\n",gps_info);

								if (state == MissionState::ON_GROUND)
								{
									sprintf(xyz_info, "xyz_info:%f:%f:%f",0,0,0);
								}
								else
								{
									sprintf(xyz_info, "xyz_info:%f:%f:%f",(snav_data->high_level_control_data.position_estimated[0]-x_est_startup)
																	 ,(snav_data->high_level_control_data.position_estimated[1]-y_est_startup)
																	 ,snav_data->high_level_control_data.position_estimated[2]);
								}
								DEBUG("xyz_info=%s\n",xyz_info);

								sprintf(rpy_info, "rpy_info:%f:%f:%f",snav_data->attitude_estimate.roll
																	 ,snav_data->attitude_estimate.pitch
																	 ,snav_data->attitude_estimate.yaw);
								DEBUG("rpy_info=%s\n",rpy_info);

								sprintf(flight_state_info, "flight_state_info:%d",state);
								DEBUG("flight_state_info=%s\n",flight_state_info);

								sprintf(drone_state_info, "drone_state_info:%d",drone_state);
								DEBUG("drone_state_info=%s\n",drone_state_info);


								strcat(result_to_client, STR_SEPARATOR);
								strcat(result_to_client, battery_info);
								strcat(result_to_client, STR_SEPARATOR);
								strcat(result_to_client, rpm_info);
								strcat(result_to_client, STR_SEPARATOR);
								strcat(result_to_client, sonar_info);
								strcat(result_to_client, STR_SEPARATOR);
								strcat(result_to_client, gps_info);
								strcat(result_to_client, STR_SEPARATOR);
								strcat(result_to_client, xyz_info);
								strcat(result_to_client, STR_SEPARATOR);
								strcat(result_to_client, rpy_info);
								strcat(result_to_client, STR_SEPARATOR);
								strcat(result_to_client, flight_state_info);
								strcat(result_to_client, STR_SEPARATOR);
								strcat(result_to_client, drone_state_info);

								DEBUG("rpy_info=%s\n",result_to_client);

								memcpy(pro_tcp_send.data, result_to_client, MAX_BUFF_LEN);
							}
							else if ((gpsparams_tcp.size() >= 1) && (gpsparams_tcp[0].compare(SNAV_TASK_GET_VERSION) == 0))
							{
								sprintf(result_to_client,"%s",SNAV_TASK_GET_VERSION_RESULT);

								strcat(result_to_client, STR_SEPARATOR);
								strcat(result_to_client, VERSION_NUM);
								memcpy(pro_tcp_send.data, result_to_client, MAX_BUFF_LEN);
							}
							else
							{
								sprintf(result_to_client,"ignore tcp task(%s) when received udp control command!",tcp_receive_data);
								memcpy(pro_tcp_send.data, result_to_client, MAX_BUFF_LEN);
							}
						}

						pthread_mutex_unlock(&pro_tcp_send.lock);

						bLocalTcpFlag = false;
					}

					bLocalUdpFlag = false;

					loop_counter++;

					continue;
				}
			}

			//check gps status to do gps task
			if(gps_waypiont_mission || gps_point_collect_mission)
			{
				int gps_enabled;
				sn_is_gps_enabled(&gps_enabled);

				if(gps_enabled != 1)
				{
					DEBUG("Error: GPS disabled. Desired state will be incorrect for optic flow modes\n");

					loop_counter++;

					continue;
				}

				SnDataStatus gps_status = (SnDataStatus) snav_data->data_status.gps_0_status;
				if (gps_status != SN_DATA_VALID)
				{
					DEBUG("cant get gps location \n");
				}
			}

			static bool mission_has_begun = false;

			if ((props_state == SN_PROPS_STATE_NOT_SPINNING) && (on_ground_flag == 1) && !mission_has_begun)
			{
				state = MissionState::ON_GROUND;
			}

			struct timeval tv;
			gettimeofday(&tv, NULL);
			double t_now = tv.tv_sec + tv.tv_usec * 1e-6;

			// Desired velocity in vehicle world frame
			float x_vel_des = 0;
			float y_vel_des = 0;
			float z_vel_des = 0;
			float yaw_vel_des = 0;

			distance_home_squared = (x_est_startup-x_est)*(x_est_startup-x_est)+(y_est_startup-y_est)*(y_est_startup-y_est);
      		yaw_target_home = atan2(y_est_startup-y_est,x_est_startup-x_est);

			DEBUG("[%d]:distance_home_squared, yaw_target_home:[%f, %f]\n",
					loop_counter,distance_home_squared,yaw_target_home);

			if ((gpsparams_tcp.size() >= 2)
				&& (gpsparams_tcp[0].compare(SNAV_CMD_MODIFY_SSID_PWD) == 0)
				&& (props_state == SN_PROPS_STATE_NOT_SPINNING)
				&& (on_ground_flag == 1))
			{
				char ssid[MAX_BUFF_LEN];
				char pwd[MAX_BUFF_LEN];
				char sed_str[MAX_BUFF_LEN];

				memset(ssid,0,MAX_BUFF_LEN);
				memset(pwd,0,MAX_BUFF_LEN);
				memset(sed_str,0,MAX_BUFF_LEN);

				memcpy(ssid, gpsparams_tcp[1].c_str(), MAX_BUFF_LEN);

				if (gpsparams_tcp.size() >= 3)
				{
					memcpy(pwd, gpsparams_tcp[2].c_str(), MAX_BUFF_LEN);
					sprintf(sed_str,
					"sed -i 's/^ssid=.*$/ssid=%s/; s/^wpa_passphrase=.*$/wpa_passphrase=%s/'  /etc/hostapd.conf",
					ssid, pwd);
				}
				else
				{
					sprintf(sed_str,
					"sed -i 's/^ssid=.*$/ssid=%s/' /etc/hostapd.conf",
					ssid);
				}
				system(sed_str);
				system("chmod 755 /etc/hostapd.conf");
				//system("ps -e |grep hostapd |awk '{print $1}'| xargs kill -9");
				system("pkill hostapd");
				sleep(10);	//10s
				system("hostapd -B /etc/hostapd.conf");
			}

			if ((gpsparams_tcp.size() >= 2)
				&& (gpsparams_tcp[0].compare(SNAV_CMD_FACE_FOLLOW) == 0))
			{
				char switcher[MAX_BUFF_LEN];

				memset(switcher,0,MAX_BUFF_LEN);
				memcpy(switcher, gpsparams_tcp[1].c_str(), MAX_BUFF_LEN);

				if (strcmp(switcher, "on") == 0)
				{
					face_follow_switch = true;
					body_follow_switch = false;
				}
				else
				{
					face_follow_switch = false;
				}
			}
			else if ((gpsparams_tcp.size() >= 2)
				&& (gpsparams_tcp[0].compare(SNAV_CMD_BODY_FOLLOW) == 0))
			{
				char switcher[MAX_BUFF_LEN];

				memset(switcher,0,MAX_BUFF_LEN);
				memcpy(switcher, gpsparams_tcp[1].c_str(), MAX_BUFF_LEN);

				if (strcmp(switcher, "on") == 0)
				{
					body_follow_switch = true;
					face_follow_switch = false;
				}
				else
				{
					body_follow_switch = true;
				}
			}

			if (state == MissionState::ON_GROUND)
			{
				// Send zero velocity while vehicle sits on ground
				x_vel_des = 0;
				y_vel_des = 0;
				z_vel_des = 0;
				yaw_vel_des = 0;

				if(gpsparams_tcp.size() >= 1)
				{
					if(gpsparams_tcp[0].compare(SNAV_CMD_TAKE_OFF) == 0)
					{
						mission_has_begun = true;
						state = MissionState::STARTING_PROPS;
					}
					else if(gpsparams_tcp[0].compare(SNAV_CMD_TRAIL_NAVIGATION) == 0)
					{
						int position_num = 0;
						int i=0;
						int lati, longi;

						position_num = atoi(gpsparams_tcp[1].c_str());

						DEBUG("Trail Navigation position_num:%d\n", position_num);

						if ((position_num >= MIN_GPS_POSITION_NUM) && (position_num >= MAX_GPS_POSITION_NUM))
						{
							continue;
						}

						for (i=0; i<2*position_num; i+=2)
						{
							lati = atoi(gpsparams_tcp[2+i].c_str());
							longi = atoi(gpsparams_tcp[2+i+1].c_str());

							DEBUG("Trail Navigation [%d]-lati,logi:%d\n", i/2, lati, longi);

							NavigationPosition pos;
							pos.latitude = pos.latitude;
							pos.longitude = posGpsCurrent.longitude;

							if(pos.latitude !=0 && pos.longitude !=0)
							{
								trail_navigation_positions.push_back(pos);
							}
						}

						mission_has_begun = true;
						state = MissionState::STARTING_PROPS;

						trail_navigation_mission = true;
					}
					/*
					else if(gpsparams_tcp[0].compare("gps_waypiont") == 0)
					{
						state = MissionState::STARTING_PROPS;

						if(gps_positions.size()>2)
						{
							gps_waypiont_mission = true;
						}

						//clockwise =1; //anticlockwise = -1

						curSendMode =SN_RC_GPS_POS_HOLD_CMD;
					}
					else if(gpsparams_tcp[0].compare(SNAV_CMD_GPS_FOLLOW) == 0)
					{
						//state = MissionState::STARTING_PROPS;
						//curSendMode =SN_RC_GPS_POS_HOLD_CMD;
					}
					else if(gpsparams_tcp[0].compare("gps_point_collect") == 0)
					{
						state = MissionState::ON_GROUND;
						gps_point_collect_mission = true;
						curSendMode =SN_RC_GPS_POS_HOLD_CMD;
					}
					else if(gpsparams_tcp[0].compare("gps_point_collect_finish") == 0)
					{
						state = MissionState::ON_GROUND;
						DEBUG("gps_point_collect_finish.\n");
						gps_point_collect_mission = false;

						for(int k=0;k<gps_positions.size();k++)
						{
							DEBUG("gps position collect finished #%u: [%d,%d,%d,%f] ",
									k,gps_positions[k].latitude,gps_positions[k].longitude,
									gps_positions[k].altitude,gps_positions[k].yaw);
						}
					}
					*/
					else
					{
						state = MissionState::ON_GROUND;
					}

					/*
					if(gps_point_collect_mission)
					{
						GpsPosition pos;
						pos.latitude = posGpsCurrent.latitude;
						pos.longitude = posGpsCurrent.longitude;
						pos.altitude = posGpsCurrent.altitude;
						pos.yaw = posGpsCurrent.yaw;

						if(posGpsDestination.latitude !=0 && posGpsDestination.longitude !=0)
						{
							gps_positions.push_back(pos);
						}

						gps_point_collect_mission = false;
					}
					*/
				}
			}
			else if (state == MissionState::STARTING_PROPS)
			{
				x_vel_des = 0;
				y_vel_des = 0;
				z_vel_des = 0;
				yaw_vel_des = 0;

				if(gpsparams_tcp.size() >= 1 && (gpsparams_tcp[0].compare(SNAV_CMD_LAND) ==0))
				{
					state = MissionState::LANDING;
					loop_counter++;
					continue;
				}

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
				if(gpsparams_tcp.size() >= 1 && (gpsparams_tcp[0].compare(SNAV_CMD_LAND) ==0))
				{
					state = MissionState::LANDING;
					loop_counter++;
					continue;
				}

				if (props_state == SN_PROPS_STATE_SPINNING)
				{
					// Command constant positive z velocity during takeoff
					x_vel_des = 0;
					y_vel_des = 0;
					z_vel_des = kTakeoffSpeed;
					yaw_vel_des = 0;

					if (trail_navigation_mission)
					{
						if (z_des - z_est_startup > fTrarilHeight)
						{
							task_take_off_in_progress = false;
							state = MissionState::LOITER;
						}
						else if(z_des - z_est_startup > 0.8f*fTrarilHeight)
						{
							z_vel_des = kTakeoffSpeed*0.5f;
						}
					}
					else
					{
						if (z_des - z_est_startup > kDesTakeoffAlt)
						{
							task_take_off_in_progress = false;
							state = MissionState::LOITER;
						}
						else if(z_des - z_est_startup > 0.8f*kDesTakeoffAlt)
						{
							z_vel_des = kTakeoffSpeed*0.5f;
						}
					}
				}
			}
			else if (state == MissionState::LANDING)
			{
				task_take_off_in_progress = true;

				if (props_state == SN_PROPS_STATE_SPINNING)
				{
					// Command constant negative z velocity during landing
					x_vel_des = 0;
					y_vel_des = 0;
					z_vel_des = kLandingSpeed;
					yaw_vel_des = 0;

					//smoothly landing start
					//float distance_to_ground = z_des - z_est_startup;
					DEBUG("[%d] [Landing distance_to_ground]: [%f]\n",
							loop_counter,snav_data->sonar_0_raw.range);

					if (snav_data->sonar_0_raw.range <= 2.5 )
					{
						z_vel_des = -0.45;
						//z_vel_des = -0.58;	//-0.56
					}

					if (snav_data->sonar_0_raw.range <= 0.3)
					{
						z_vel_des = -1;
					}

					//smoothly landing end

					if (props_state == SN_PROPS_STATE_SPINNING && on_ground_flag == 1)
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
					circle_mission=false;
					calcCirclePoint = false;
					gps_waypiont_mission = false;
					gps_point_collect_mission = false;
					followme_mission = false;
					trail_navigation_mission = false;

					//people detect cuiyc begin
					face_mission=false;
					body_mission=false;

					return_mission = false;
				}
			}
			else if(state == MissionState::TRAJECTORY_FOLLOW)
			{
				float accel_max  = 1.5;   //m/sec
				float stopping_accel = 1.0; //m/sec

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

				if(gpsparams_tcp.size() >= 1 && (gpsparams_tcp[0].compare(SNAV_CMD_LAND) ==0))
				{
					state = MissionState::LANDING;
					loop_counter++;
					continue;
				}

				/*if(gps_waypiont_mission && gps_positions.size()>2)
				{
					command_diff_x = CalcAxisDistance((float)(gps_positions[current_position].longitude)/1e7,
																(float)posGpsCurrent.longitude/1e7);

					command_diff_y = CalcAxisDistance((float)(gps_positions[current_position].latitude)/1e7,
																(float)posGpsCurrent.latitude/1e7);

					distance_to_dest =CalcDistance(
													(float)(gps_positions[current_position].latitude)/1e7,
													(float)(gps_positions[current_position].longitude)/1e7,
													(float)posGpsCurrent.latitude/1e7,
													(float)posGpsCurrent.longitude/1e7);

					if(distance_to_dest<0.01)
					{
						distance_to_dest = 0.01;
					}  //to prevent dividing by zero

					vel_x_target = command_diff_x/distance_to_dest * vel_target;
					vel_y_target = command_diff_y/distance_to_dest * vel_target;
					vel_z_target = command_diff_z/distance_to_dest * vel_target;

					if(distance_to_dest < 2.5 ) //about 150*150   x,y 1.5m
					{
						//if it is on the last waypoint then slow down before stopping
						float stopping_vel = sqrt(2*stopping_accel*distance_to_dest);
						if(stopping_vel<vel_target)
						{
							vel_target = stopping_vel;
						}
					}

					if(distance_to_dest<1) // 1m
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
				else */if(circle_mission)
				{
					command_diff_x = circle_positions[current_position].x - (x_des-x_est_startup);
					command_diff_y = circle_positions[current_position].y - (y_des-y_est_startup);
					command_diff_z = circle_positions[current_position].z - (z_des-z_est_startup);
					command_diff_yaw = circle_positions[current_position].yaw - yaw_des;


					DEBUG("[%d] [circle_mission x_des y_des z_des]: [%f %f %f]\n",
							loop_counter,x_des,y_des,z_des);

					if (command_diff_yaw > M_PI)
					{
						command_diff_yaw = command_diff_yaw - 2*M_PI;
					}
					else if (command_diff_yaw < -M_PI)
					{
						command_diff_yaw = command_diff_yaw+ 2*M_PI;
					}

					if (abs(command_diff_yaw)>M_PI*0.25f)
					{
						state = MissionState::LOITER;
						current_position =0;
						circle_mission=false;
						calcCirclePoint = false;
						continue;
					}

					distance_to_dest = sqrt(command_diff_x*command_diff_x +
										 command_diff_y*command_diff_y +
										 command_diff_z*command_diff_z);

					if(distance_to_dest<0.01)
					{
						distance_to_dest = 0.01;
					}  //to prevent dividing by zero

					//if(distance_to_dest<0.04)	//&& abs(command_diff_yaw) <angle_per
					if(distance_to_dest<0.06)
					{
						// Close enough, move on
						current_position++;
						if (current_position >= circle_positions.size())
						{
							// No more circle_positions, so land
							state = MissionState::LOITER;
							current_position =0;
							circle_mission=false;
							calcCirclePoint = false;
						}
					}

					vel_x_target = command_diff_x/distance_to_dest * vel_target;
					vel_y_target = command_diff_y/distance_to_dest * vel_target;
					vel_z_target = command_diff_z/distance_to_dest * vel_target;
					vel_yaw_target = command_diff_yaw/angle_per*vel_target;

					DEBUG("[%d] [circle_mission vel_x_target vel_y_target vel_z_target vel_yaw_target]: [%f %f %f %f]\n",
							loop_counter,vel_x_target,vel_y_target,vel_z_target,vel_yaw_target);

					DEBUG("[%d] [distance_to_dest command_diff_x command_diff_y command_diff_z command_diff_yaw]: [%f %f %f %f %f]\n",
							loop_counter,distance_to_dest,command_diff_x,command_diff_y,command_diff_z,command_diff_yaw);

					//if (current_position >= 0.9*circle_positions.size()) //slow down
					//	vel_yaw_target =0;
					//if(vel_yaw_target>0.8f)vel_yaw_target=0.8f;
				}
				else if (trail_navigation_mission)
				{
					command_diff_x = CalcAxisDistance(trail_navigation_positions[current_position].longitude/1e7,
													  (float)posGpsCurrent.longitude/1e7);

					command_diff_y = CalcAxisDistance(trail_navigation_positions[current_position].latitude/1e7,
													  (float)posGpsCurrent.latitude/1e7);

					distance_to_dest = CalcDistance(
													trail_navigation_positions[current_position].latitude/1e7,
													trail_navigation_positions[current_position].longitude/1e7,
													(float)posGpsCurrent.latitude/1e7,
													(float)posGpsCurrent.longitude/1e7);

					if(distance_to_dest<0.01)
					{
						distance_to_dest = 0.01;
					}  //to prevent dividing by zero

					vel_x_target = command_diff_x/distance_to_dest * vel_target;
					vel_y_target = command_diff_y/distance_to_dest * vel_target;
					vel_z_target = command_diff_z/distance_to_dest * vel_target;

					if(distance_to_dest < 2.5 ) //about 150*150   x,y 1.5m
					{
						//if it is on the last waypoint then slow down before stopping
						float stopping_vel = sqrt(2*stopping_accel*distance_to_dest);
						if(stopping_vel<vel_target)
						{
							vel_target = stopping_vel;
						}
					}

					if(distance_to_dest<1) // 1m
					{
						// Close enough, move on
						current_position++;
						if (current_position >= trail_navigation_positions.size())
						{
						    state = MissionState::LOITER;
							trail_navigation_mission = false;
						}
					}
				}
				/*
				else if(followme_mission)	// && gpsparams_tcp[0].compare(SNAV_CMD_GPS_FOLLOW) == 0)
				{
					//curSendMode =SN_RC_GPS_POS_HOLD_CMD;
					float yaw_diff ;

					//destyaw = (float)atof(gpsparams_tcp[1].c_str());
					//speed = (float)atof(gpsparams_tcp[2].c_str());
					yaw_diff = destyaw - yaw_est;

					if(speed == 0 && abs(yaw_diff)<0.05f)
					{
						state = MissionState::LOITER;
						followme_mission = false;
					}

					if(yaw_diff > M_PI)
					{
						yaw_diff = yaw_diff -2*M_PI;
					}
					else if (yaw_diff < -M_PI)
					{
						yaw_diff = yaw_diff + 2*M_PI;
					}

					vel_yaw_target = (yaw_diff)*vel_target;

					command_diff_z = kDesTakeoffAlt - (z_des-z_est_startup);

					if(speed == 0)
					{
						vel_x_target =0;
						vel_y_target = 0;
					}
					else
					{
						vel_x_target = speed*cos(destyaw);
						vel_y_target = speed*sin(destyaw);
					}

					vel_z_target = command_diff_z/kDesTakeoffAlt * vel_target;

					DEBUG("[%d] [vel_x_target vel_y_target vel_z_target vel_yaw_target]: [%f %f %f %f]\n",
									loop_counter,vel_x_target,vel_y_target,vel_z_target,vel_yaw_target);
				}
				*/
				//cuiyc add face detect begin
				else if(face_mission)
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

						DEBUG("[%d] face_mission newP [forword_dis parallel_dis distance_to_dest ]: [%f %f %f]\n",
								loop_counter,forword_dis,parallel_dis,distance_to_dest);

						distance_remain_x = cos(yaw_est)*forword_dis-sin(yaw_est)*parallel_dis;
						distance_remain_y = sin(yaw_est)*forword_dis+cos(yaw_est)*parallel_dis;
						cur_body.newP = false;
					}

					if(((abs(angle_face_offset))< min_angle_offset && abs(distance_to_dest) <0.05f)
						|| !cur_body.have_face)
					{
						state = MissionState::LOITER;
						DEBUG(" face_mission follow face-> LOITER\n" );
						face_mission = false;
						continue;
					}
					vel_yaw_target = 0;

					//for test rotate
					distance_remain_x = distance_remain_x - vel_x_des_sent*0.02f; //20ms a tick
					distance_remain_y = distance_remain_y - vel_y_des_sent*0.02f;

					distance_to_dest = sqrt(distance_remain_x*distance_remain_x +
											 distance_remain_y*distance_remain_y);

					DEBUG("[%d] face_mission [distance_remain_x distance_remain_y]: [%f %f]\n",
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

					DEBUG("[%d] face_mission [vel_x_target vel_y_target distance_to_dest vel_yaw_target]: [%f %f %f %f]\n",
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

						DEBUG(" [%d] body_mission newP [forword_dis parallel_dis distance_to_dest ]: [%f %f %f]\n",
								loop_counter,forword_dis,parallel_dis,distance_to_dest);

						distance_remain_x = cos(yaw_est)*forword_dis-sin(yaw_est)*parallel_dis;
						distance_remain_y = sin(yaw_est)*forword_dis+cos(yaw_est)*parallel_dis;

						cur_body.newP = false;
					}

					if(((abs(angle_face_offset))< min_angle_offset && abs(distance_to_dest) <0.15f)
						|| (!cur_body.have_body) )
					{
						state = MissionState::LOITER;
						DEBUG("body_mission follow face-> LOITER\n" );
						face_mission = false;
						continue;
					}

					vel_yaw_target = 0;

					distance_remain_x = distance_remain_x - vel_x_des_sent*0.02f; //20ms a tick
					distance_remain_y = distance_remain_y - vel_y_des_sent*0.02f;

					distance_to_dest = sqrt(distance_remain_x*distance_remain_x +
											 distance_remain_y*distance_remain_y);

					DEBUG(" [%d] body_mission [distance_remain_x distance_remain_y]: [%f %f]\n",
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

					//command_diff_z = kDesTakeoffAlt - (z_des-z_est_startup);

					//if(abs(command_diff_z)>0.05)
					//	vel_z_target = command_diff_z/kDesTakeoffAlt * vel_target;
					//else
					//	vel_z_target =0;

					DEBUG(" [%d]  body_mission [vel_x_target vel_y_target distance_to_dest vel_yaw_target]: [%f %f %f %f]\n",
						loop_counter,vel_x_target,vel_y_target,distance_to_dest,vel_yaw_target);
				}//cuiyc add face detect end



				//return mission
				if (return_mission)
				{
					if (fly_home)
					{
						// Go to home waypoint
						goto_waypoint({x_des, y_des, z_des, yaw_des}, {x_est_startup, y_est_startup, z_des, yaw_des},
										{gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}, &output_vel, &wp_goal);
						gohome_x_vel_des = output_vel.x;
				        gohome_y_vel_des = output_vel.y;
				        gohome_z_vel_des = output_vel.z;
				        gohome_yaw_vel_des = output_vel.yaw;

						DEBUG("[%d]:return_mission direct fly_home wp_goal,gohome_x_vel_des,gohome_y_vel_des,gohome_z_vel_des,gohome_yaw_vel_des:[%f,%f,%f,%f]\n",
									loop_counter,gohome_x_vel_des,gohome_y_vel_des,gohome_z_vel_des,gohome_yaw_vel_des);

						if (wp_goal == true)
				        {
							// If home, enter landing
							wp_goal = false;
							state = MissionState::LANDING;
							return_mission = false;

							gohome_x_vel_des = 0;
					        gohome_y_vel_des = 0;
					        gohome_z_vel_des = 0;
					        gohome_yaw_vel_des = 0;

							fly_home = false;
				        }
					}
					else
					{
						goto_waypoint({x_des, y_des, z_des, yaw_des}, {x_des, y_des, z_des, yaw_target_home},
            							{gohome_x_vel_des, gohome_y_vel_des, gohome_z_vel_des, gohome_yaw_vel_des}, &output_vel, &wp_goal);
						gohome_x_vel_des = output_vel.x;
				        gohome_y_vel_des = output_vel.y;
				        gohome_z_vel_des = output_vel.z;
				        gohome_yaw_vel_des = output_vel.yaw;

						DEBUG("[%d]:return_mission fly_raw wp_goal,gohome_x_vel_des,gohome_y_vel_des,gohome_z_vel_des,gohome_yaw_vel_des:[%f,%f,%f,%f]\n",
									loop_counter,gohome_x_vel_des,gohome_y_vel_des,gohome_z_vel_des,gohome_yaw_vel_des);

						if (wp_goal == true)
				        {
							// If waypoint is reached (facing home), enter Fly_home
							wp_goal = false;

							fly_home = true;
						}
					}
				}
				else
				{
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

					DEBUG("[%d] [vel_x_diff,vel_y_diff,vel_z_diff,vel_yaw_diff]: [%f,%f,%f,%f] \n",
										loop_counter,vel_x_diff,vel_y_diff,vel_z_diff,vel_yaw_diff);

					float vel_diff_mag = sqrt(vel_x_diff*vel_x_diff +
					                  vel_y_diff*vel_y_diff +
					                  vel_z_diff*vel_z_diff);

					if(vel_diff_mag<0.01)
					{
						vel_diff_mag = 0.01;
					}

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
					{
						vel_yaw_des_sent = vel_yaw_target;
					}
					else
					{
						vel_yaw_des_sent += v_del_max;
					}

					distance_to_home = sqrt((x_est_startup-x_est)*(x_est_startup-x_est)+
									(y_est_startup-y_est)*(y_est_startup-y_est)+
									(z_est_startup-z_est)*(z_est_startup-z_est));
					//todo ...height and distance limited

					yaw_vel_des = vel_yaw_des_sent;

					x_vel_des = vel_x_des_sent;
					y_vel_des = vel_y_des_sent;
					z_vel_des = vel_z_des_sent;

					DEBUG("[%d] [x_vel_des,y_vel_des,z_vel_des,yaw_vel_des]: [%f,%f,%f,%f] \n",
										loop_counter,x_vel_des,y_vel_des,z_vel_des,yaw_vel_des);
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

					static bool entering_loiter = true;
					static double t_loiter_start = 0;

					if (entering_loiter)
					{
						t_loiter_start = t_now;
						entering_loiter = false;
					}

					if(gpsparams_tcp.size() >= 1 && (gpsparams_tcp[0].compare(SNAV_CMD_LAND) ==0))
					{
						state = MissionState::LANDING;
					}
					//circel task
					else if(gpsparams_tcp.size() >= 3
							&& gpsparams_tcp[0].compare(SNAV_CMD_CIRCLE) == 0
							&& !return_mission
							&& !trail_navigation_mission)
					{
						radius = atof(gpsparams_tcp[1].c_str());
						clockwise = atoi(gpsparams_tcp[2].c_str());

						curSendMode = SN_RC_OPTIC_FLOW_POS_HOLD_CMD;
						circle_mission = true;
						calcCirclePoint = true;

						if (radius < 1)
						{
							point_count = 36;
							vel_target = 0.5;	 //m/sec
						}
						else
						{
							point_count = ((int)radius)*36;
							vel_target = 0.5*radius;	 //m/sec
						}

						angle_per = 2*M_PI/point_count;

						DEBUG("LOITER circle: radius,clockwise,point_count,vel_target,angle_per:%f,%d,%d,%f,%f\n",
								radius,clockwise,point_count,vel_target,angle_per);
					}
					//return task
					else if(gpsparams_tcp.size() >= 1
							&& gpsparams_tcp[0].compare(SNAV_CMD_RETURN) == 0
							&& !circle_mission
							&& !trail_navigation_mission)
					{
						curSendMode =SN_RC_OPTIC_FLOW_POS_HOLD_CMD;
						return_mission = true;

						state = MissionState::TRAJECTORY_FOLLOW;
						entering_loiter = true;

						DEBUG("[%d]:return_mission distance_home_squared, yaw_target_home:[%f,%f]\n",
										loop_counter,distance_home_squared,yaw_target_home);

						if (distance_home_squared > distance_home_squared_threshold)
						{
							fly_home = false;
						}
						else
						{
							fly_home = true;
						}

						gohome_x_vel_des = 0;
				        gohome_y_vel_des = 0;
				        gohome_z_vel_des = 0;
				        gohome_yaw_vel_des = 0;

						DEBUG("LOITER return enter TRAJECTORY_FOLLOW\n");
					}
					//trail_navigation task
					else if(gpsparams_tcp.size() >= 1
							&& gpsparams_tcp[0].compare(SNAV_CMD_TRAIL_NAVIGATION) == 0
							&& !circle_mission
							&& !return_mission)
					{
						int position_num = 0;
						int i=0;
						int lati, longi;

						position_num = atoi(gpsparams_tcp[1].c_str());

						DEBUG("Trail Navigation position_num:%d\n", position_num);

						if ((position_num >= MIN_GPS_POSITION_NUM) && (position_num >= MAX_GPS_POSITION_NUM))
						{
							continue;
						}

						for (i=0; i<2*position_num; i+=2)
						{
							lati = atoi(gpsparams_tcp[2+i].c_str());
							longi = atoi(gpsparams_tcp[2+i+1].c_str());

							DEBUG("Trail Navigation [%d]-lati,logi:%d\n", i/2, lati, longi);

							NavigationPosition pos;
							pos.latitude = pos.latitude;
							pos.longitude = posGpsCurrent.longitude;

							if(pos.latitude !=0 && pos.longitude !=0)
							{
								trail_navigation_positions.push_back(pos);
							}
						}

						trail_navigation_mission = true;

						state = MissionState::TRAJECTORY_FOLLOW;
						entering_loiter = true;

						DEBUG("LOITER return enter TRAJECTORY_FOLLOW\n");
					}
					/*
					else if(gps_waypiont_mission && gps_positions.size()>2)
					{
						//posGpsDestination.latitude = atoi(gpsparams_tcp[2].c_str());
						//posGpsDestination.longitude = atoi(gpsparams_tcp[1].c_str());
						//destyaw = (float)atof(gpsparams_tcp[3].c_str());

						posGpsDestination.latitude = gps_positions[current_position].latitude;
						posGpsDestination.longitude = gps_positions[current_position].longitude;
						destyaw = gps_positions[current_position].yaw;

						distance_to_dest =CalcDistance(
							(float)posGpsDestination.latitude/1e7,
							(float)posGpsDestination.longitude/1e7,
							(float)posGpsCurrent.latitude/1e7,
							(float)posGpsCurrent.longitude/1e7);

						if(posGpsDestination.latitude !=0
							&& posGpsDestination.longitude !=0
							&& distance_to_dest>1.5f) //more than 0.5m
						{
							state = MissionState::TRAJECTORY_FOLLOW;
							posLast = posGpsCurrent;
							entering_loiter = true;
							DEBUG("get gps data,TRAJECTORY_FOLLOW\n");
						}
						else
						{
							state = MissionState::LOITER;
						}
					}
					else if((gpsparams_tcp.size() >= 3) && (gpsparams_tcp[0].compare(SNAV_CMD_GPS_FOLLOW) == 0))
					{
						//curSendMode =SN_RC_GPS_POS_HOLD_CMD;
						followme_mission = true;
						destyaw = (float)atof(gpsparams_tcp[1].c_str());
						speed = (float)atof(gpsparams_tcp[2].c_str());

						DEBUG("destyaw:%f  speed:%f\n",destyaw,speed);
						float yaw_diff ;
						yaw_diff = destyaw - yaw_est;

						if(abs(yaw_diff) >0.05f)
						{
							state = MissionState::TRAJECTORY_FOLLOW;
							entering_loiter = true;
						}
						DEBUG("followme yaw_diff:%f \n",yaw_diff);
					}
					*/
					//cuiyc add face detect begin
					else if(cur_body.have_face && face_follow_switch)
					{
						float face_offset ;
						face_offset = M_PI*cur_body.angle/180;
						DEBUG("followme have_face\n" );

						if(abs(face_offset) >0.055f || cur_body.distance>(safe_distance+0.15)
							|| cur_body.distance < (safe_distance-0.15))
						{
							face_mission = true;
							state = MissionState::TRAJECTORY_FOLLOW;
							entering_loiter = true;
							DEBUG("face_offset:%f	distance:%f\n",face_offset,cur_body.distance);
							DEBUG("followme have_face LOITER -> FACE_FOLLOW\n" );
						}
					}
					else if(cur_body.have_body && body_follow_switch)
					{
						float body_offset ;
						body_offset = M_PI*cur_body.angle/180;
						DEBUG("followme have_body\n" );

						if(abs(body_offset) >0.055f || cur_body.distance>(safe_distance+0.15)
							|| cur_body.distance < (safe_distance-0.15))
						{
							body_mission= true;
							state = MissionState::TRAJECTORY_FOLLOW;
							entering_loiter = true;
							DEBUG("face_offset:%f	distance:%f\n",body_offset,cur_body.distance);
							DEBUG("followme have_body LOITER -> FACE_FOLLOW\n" );
						}
					}
					//cuiyc add face detect end

					if(circle_mission)
					{
						if (t_now - t_loiter_start > kLoiterTime)
						{
							if(calcCirclePoint)
							{
								//circle center
								float circle_center_x;
								float circle_center_y;
								float yaw_t =0;

								circle_center_x = x_est-x_est_startup + radius*cos(yaw_est);
								circle_center_y = y_est-y_est_startup + radius*sin(yaw_est);

								if ((clockwise != 1) && (clockwise != -1))
								{
									clockwise = 1;
								}

								circle_positions.clear();//clear and recaculate.
								for(int k=0;k<=point_count;k++)	//the last position must be the same as the first position
								{
									Position pos;
									//pos.x = (1-cos(angle_per*k))*radius + x_est;
									//pos.y = -sin(angle_per*k)*radius + y_est;

									//yaw_t = yaw_est+angle_per*k*clockwise;
									yaw_t = yaw_est-angle_per*k*clockwise;

									if(yaw_t > M_PI)
									{
										yaw_t = yaw_t -2*M_PI;
									}
									else if (yaw_t < -M_PI)
									{
										yaw_t = yaw_t + 2*M_PI;
									}

									if(yaw_t>0)
									{
										pos.x = cos(yaw_t-M_PI)*radius + circle_center_x;
										pos.y = sin(yaw_t-M_PI)*radius + circle_center_y;
									}
									else
									{
										pos.x = cos(yaw_t+M_PI)*radius + circle_center_x;
										pos.y = sin(yaw_t+M_PI)*radius + circle_center_y;
									}
									pos.z = z_des - z_est_startup;	//kDesTakeoffAlt;
									pos.yaw = yaw_t;

									circle_positions.push_back(pos);
								}

								calcCirclePoint = false;

								DEBUG("@@@@circle_center_point [%f,%f]\n",
										circle_center_x,circle_center_y);

								for(int k=0;k<circle_positions.size();k++)
								{
									DEBUG("@@@@[%d] position #%u: [%f,%f,%f,%f]\n",k,
										k,circle_positions[k].x,circle_positions[k].y,
										circle_positions[k].z,circle_positions[k].yaw);
								}
							}

							state = MissionState::TRAJECTORY_FOLLOW;
							entering_loiter = true;
						}
					}
					else if (trail_navigation_mission)
					{
						if (t_now - t_loiter_start > kLoiterTime)
						{
							state = MissionState::TRAJECTORY_FOLLOW;
							entering_loiter = true;
						}
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

				if (props_state == SN_PROPS_STATE_SPINNING && on_ground_flag == 1)
				{
					sn_stop_props();
				}
			}


			if ((props_state == SN_PROPS_STATE_NOT_SPINNING)
					&& (on_ground_flag == 1)
					&& ((state == MissionState::TRAJECTORY_FOLLOW)
						|| (state == MissionState::LOITER)))
			{
				state = MissionState::ON_GROUND;

				circle_mission=false;
				calcCirclePoint = false;
				gps_waypiont_mission = false;
				gps_point_collect_mission = false;
				followme_mission = false;
			}

			// Rotate velocity by estimated yaw angle before sending
			// This puts velocity in body-relative Z-up frame
			float x_vel_des_yawed = x_vel_des*cos(-yaw_est) - y_vel_des*sin(-yaw_est);
			float y_vel_des_yawed = x_vel_des*sin(-yaw_est) + y_vel_des*cos(-yaw_est);

			float gohome_x_vel_des_yawed = gohome_x_vel_des*cos(-yaw_est) - gohome_y_vel_des*sin(-yaw_est);
			float gohome_y_vel_des_yawed = gohome_x_vel_des*sin(-yaw_est) + gohome_y_vel_des*cos(-yaw_est);

			float cmd0 = 0;
			float cmd1 = 0;
			float cmd2 = 0;
			float cmd3 = 0;

			// Go from the commands in real units computed above to the
			// dimensionless commands that the interface is expecting using a
			// linear mapping
			//sn_apply_cmd_mapping(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
			//sn_apply_cmd_mapping(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,//curSendMode

			if (return_mission)
			{
				DEBUG("[sn_apply_cmd_mapping gohome_x_vel_des_yawed, gohome_y_vel_des_yawed, gohome_z_vel_des, gohome_yaw_vel_des]: [%f,%f,%f,%f]\n",
	  											gohome_x_vel_des_yawed,gohome_y_vel_des_yawed,gohome_z_vel_des,gohome_yaw_vel_des);
				sn_apply_cmd_mapping(curSendMode, RC_OPT_LINEAR_MAPPING,
						gohome_x_vel_des_yawed, gohome_y_vel_des_yawed, gohome_z_vel_des, gohome_yaw_vel_des,
						&cmd0, &cmd1, &cmd2, &cmd3);
			}
			else
			{
				DEBUG("[sn_apply_cmd_mapping x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des]: [%f,%f,%f,%f]\n",
		  											x_vel_des_yawed,y_vel_des_yawed,z_vel_des,yaw_vel_des);
				sn_apply_cmd_mapping(curSendMode, RC_OPT_LINEAR_MAPPING,
						x_vel_des_yawed, y_vel_des_yawed, z_vel_des, yaw_vel_des,
						&cmd0, &cmd1, &cmd2, &cmd3);
			}

			// Send the commands if in the right mode.
			//sn_send_rc_command(SN_RC_GPS_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,
			//sn_send_rc_command(SN_RC_OPTIC_FLOW_POS_HOLD_CMD, RC_OPT_LINEAR_MAPPING,

			DEBUG("[sn_send_rc_command cmd0 cmd1 cmd2 cmd3]: [%f,%f,%f,%f]\n",cmd0,cmd1,cmd2,cmd3);
			sn_send_rc_command(curSendMode, RC_OPT_LINEAR_MAPPING,
			         cmd0, cmd1, cmd2, cmd3);


			// Print some information
			if(mode == SN_GPS_POS_HOLD_MODE){DEBUG("\n[%d] SN_GPS_POS_HOLD_MODE. ",loop_counter);}
			else if(mode == SN_SENSOR_ERROR_MODE){DEBUG("\n[%d] SENSOR ERROR MODE. ",loop_counter);}
			else if(mode == SN_OPTIC_FLOW_POS_HOLD_MODE){DEBUG("\n[%d] OPTIC FLOW VELOCITY MODE MODE. ",loop_counter);}
			else{DEBUG("\n[%d] UNDEFINED MODE :%d \n",loop_counter,mode);}

			if(props_state == SN_PROPS_STATE_NOT_SPINNING){DEBUG("Propellers NOT spinning\n");}
			else if(props_state == SN_PROPS_STATE_STARTING){DEBUG("Propellers attempting to spin\n");}
			else if (props_state == SN_PROPS_STATE_SPINNING){DEBUG("Propellers spinning\n");}
			else{DEBUG("Unknown propeller state\n");}

			DEBUG("[%d] commanded rates: [%f,%f,%f,%f]\n",loop_counter,x_vel_des_yawed,y_vel_des_yawed,z_vel_des,yaw_vel_des);
			DEBUG("[%d] battery_voltage: %f\n",loop_counter,voltage);
			DEBUG("[%d] [x_est,y_est,z_est,yaw_est]: [%f,%f,%f,%f]\n",loop_counter,x_est-x_est_startup,y_est-y_est_startup,z_est-z_est_startup,yaw_est);
			DEBUG("[%d] [x_des,y_des,z_des,yaw_des]: [%f,%f,%f,%f]\n",
					loop_counter,x_des-x_est_startup,y_des-y_est_startup,z_des-z_est_startup,yaw_des);

			DEBUG("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
			DEBUG("[%d] [x_est,y_est,z_est,yaw_est]: [%f,%f,%f,%f]\n",
					loop_counter,x_est,y_est,z_est,yaw_est);
			DEBUG("[%d] [x_des,y_des,z_des,yaw_des]: [%f,%f,%f,%f]\n",
					loop_counter,x_des,y_des,z_des,yaw_des);
			DEBUG("[%d] [x_est_startup,y_est_startup,z_est_startup,yaw_est_startup]: [%f,%f,%f,%f]\n",
					loop_counter,x_est_startup,y_est_startup,z_est_startup,yaw_est_startup);


			DEBUG("[%d] Current Sonar range: [%f]\n", loop_counter,snav_data->sonar_0_raw.range);

			if (state == MissionState::ON_GROUND)
				DEBUG("[%d] ON_GROUND\n",loop_counter);
			else if (state == MissionState::STARTING_PROPS)
				DEBUG("[%d] STARTING_PROPS\n",loop_counter);
			else if (state == MissionState::TAKEOFF)
			{
				DEBUG("[%d] TAKEOFF\n",loop_counter);
				DEBUG("[%d] position_est_startup: [%f,%f,%f]\n",loop_counter,x_est_startup,y_est_startup,z_est_startup);
			}
			else if (state == MissionState::TRAJECTORY_FOLLOW)
			{
				DEBUG("[%d] TRAJECTORY_FOLLOW\n",loop_counter);
				if(gps_waypiont_mission )
					DEBUG("[%d] [posGpsCurrent.latitude,posGpsCurrent.latitude]: [%d,%d]\n",
							loop_counter,posGpsCurrent.latitude,posGpsCurrent.longitude);

				if(circle_mission)
					DEBUG("[%d] circle_mission position #%u: [%f,%f,%f,%f]\n",loop_counter,
							current_position,circle_positions[current_position].x,
							circle_positions[current_position].y, circle_positions[current_position].z,
							circle_positions[current_position].yaw);
			}
			else if (state == MissionState::LOITER)
				DEBUG("[%d] LOITER\n",loop_counter);
			else if (state == MissionState::LANDING)
				DEBUG("[%d] LANDING\n",loop_counter);
			else
				DEBUG("[%d] STATE UNKNOWN\n",loop_counter);


			static double t_start = 0;
			//DEBUG("[%d] last comand time: %f\n",loop_counter,(t_now - t_start));
			t_start = t_now;

			//tcp 8001 task for get status and data information when tcp task
			if (bLocalTcpFlag)
			{
				pthread_mutex_lock(&pro_tcp_send.lock);

				//set the flag and data for the snav_handler-thread check
				pro_tcp_send.bflag = true;
				pro_tcp_send.fd_socket = localTcpFd;

				if (strlen(pro_tcp_send.data) > 0)
				{
					usleep(20000);
				}

				memset(pro_tcp_send.data,0,MAX_BUFF_LEN);

				if ((gpsparams_tcp.size() >= 1) && (gpsparams_tcp[0].compare(SNAV_TASK_GET_INFO) == 0))
				{
					memset(result_to_client,0,MAX_BUFF_LEN);
					sprintf(result_to_client,"%s",SNAV_TASK_GET_INFO_RESULT);

					char battery_info[MAX_BUFF_LEN];
					char rpm_info[MAX_BUFF_LEN];
					char sonar_info[MAX_BUFF_LEN];
					char gps_info[MAX_BUFF_LEN];
					char xyz_info[MAX_BUFF_LEN];
					char rpy_info[MAX_BUFF_LEN];
					char flight_state_info[MAX_BUFF_LEN];
					char drone_state_info[MAX_BUFF_LEN];

					memset(battery_info,0,MAX_BUFF_LEN);
					memset(rpm_info,0,MAX_BUFF_LEN);
					memset(sonar_info,0,MAX_BUFF_LEN);
					memset(gps_info,0,MAX_BUFF_LEN);
					memset(xyz_info,0,MAX_BUFF_LEN);
					memset(rpy_info,0,MAX_BUFF_LEN);
					memset(flight_state_info,0,MAX_BUFF_LEN);
					memset(drone_state_info,0,MAX_BUFF_LEN);


					sprintf(battery_info,"battery_info:%f",snav_data->general_status.voltage);
					DEBUG("battery_info=%s\n",battery_info);


					sprintf(rpm_info,"rpm_info:%d:%d:%d:%d",snav_data->esc_raw.rpm[0], snav_data->esc_raw.rpm[1]
															   ,snav_data->esc_raw.rpm[2], snav_data->esc_raw.rpm[3]);
					DEBUG("rpm_info=%s\n",rpm_info);

					sprintf(sonar_info,"sonar_info:%f",snav_data->sonar_0_raw.range);
					DEBUG("sonar_info=%s\n",sonar_info);


					int gps_enabled;
					sn_is_gps_enabled(&gps_enabled);

					if(gps_enabled != 1)
					{
						sprintf(gps_info, "gps_info:gps is not enabled!");
					}
					else
					{
						SnDataStatus gps_status = (SnDataStatus) snav_data->data_status.gps_0_status;
						if (gps_status != SN_DATA_VALID)
						{
							sprintf(gps_info, "gps_info:can not get gps location!");
						}
						else
						{
							sprintf(gps_info, "gps_info:%d:%d",snav_data->gps_0_raw.longitude, snav_data->gps_0_raw.latitude);
						}
					}
					DEBUG("gps_info=%s\n",gps_info);

					if (state == MissionState::ON_GROUND)
					{
						sprintf(xyz_info, "xyz_info:%f:%f:%f",0,0,0);
					}
					else
					{
						sprintf(xyz_info, "xyz_info:%f:%f:%f",(snav_data->high_level_control_data.position_estimated[0]-x_est_startup)
														 ,(snav_data->high_level_control_data.position_estimated[1]-y_est_startup)
														 ,snav_data->high_level_control_data.position_estimated[2]);
					}
					DEBUG("xyz_info=%s\n",xyz_info);

					sprintf(rpy_info, "rpy_info:%f:%f:%f",snav_data->attitude_estimate.roll
														 ,snav_data->attitude_estimate.pitch
														 ,snav_data->attitude_estimate.yaw);
					DEBUG("rpy_info=%s\n",rpy_info);

					sprintf(flight_state_info, "flight_state_info:%d",state);
					DEBUG("flight_state_info=%s\n",flight_state_info);

					sprintf(drone_state_info, "drone_state_info:%d",drone_state);
					DEBUG("drone_state_info=%s\n",drone_state_info);

					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, battery_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, rpm_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, sonar_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, gps_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, xyz_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, rpy_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, flight_state_info);
					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, drone_state_info);

					DEBUG("rpy_info=%s\n",result_to_client);

					memcpy(pro_tcp_send.data, result_to_client, MAX_BUFF_LEN);
				}
				else if ((gpsparams_tcp.size() >= 1) && (gpsparams_tcp[0].compare(SNAV_TASK_GET_VERSION) == 0))
				{
					memset(result_to_client,0,MAX_BUFF_LEN);
					sprintf(result_to_client,"%s",SNAV_TASK_GET_VERSION_RESULT);

					strcat(result_to_client, STR_SEPARATOR);
					strcat(result_to_client, VERSION_NUM);
					memcpy(pro_tcp_send.data, result_to_client, MAX_BUFF_LEN);
				}
				else
				{
					memcpy(pro_tcp_send.data, tcp_receive_data, MAX_BUFF_LEN);
				}

				pthread_mutex_unlock(&pro_tcp_send.lock);

				bLocalTcpFlag = false;
			}
		}
		loop_counter++;
		//usleep(20000);
	}

	DEBUG("out from the snav_handler thread\n\n");

    pthread_exit(NULL);
}

//cyc people detect begin
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

	struct sockaddr_un clt_addr;
	struct sockaddr_un srv_addr;
 	socklen_t len =sizeof(clt_addr);

	DEBUG("getVideoFaceFollowParam start");

	while(1)
	{
		//listen_fd=socket(AF_UNIX,SOCK_STREAM,0);
		listen_fd=socket(AF_UNIX,SOCK_DGRAM,0);
		if(listen_fd<0)
		{
			DEBUG("cannot create listening socket");
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
					DEBUG("cannot bind server socket");
					//close(listen_fd);
					unlink(UNIX_DOMAIN);
					break;
				}

				while(true)
				{
					recv_php_num = recvfrom(listen_fd, recv_php_buf, sizeof(recv_php_buf),
											0, (struct sockaddr *)&clt_addr, &len);
					DEBUG("\n=====face info=====\n");
					//0,flag 1,center-x 2,center-y,3,face_winth,4,_face_height,5,video_winth,6,video_height
					for(i=0;i<16;i++)
					{
						DEBUG("%d ",recv_php_buf[i]);
					}
					DEBUG("\n");

					if(recv_php_buf[0] == 1)
					{
						// normal  face 18cm && camera 83.5 degree
						float distance,angle;
						cur_body.have_face=true;

						if(recv_php_buf[6] == 720) //720p
						{
							distance = face_winth/tan((recv_php_buf[3]*camera_rad/1280));
						}
						else if(recv_php_buf[6] == 1080) //1080p
						{
							distance = face_winth/tan((recv_php_buf[3]*camera_rad/1920));
						}
						else if(recv_php_buf[5] != 0)
						{
							distance = face_winth/tan((recv_php_buf[3]*camera_rad/recv_php_buf[5])/2);
						}

						//dgree for window 72.7768
						angle = window_degree*((recv_php_buf[5]/2 - recv_php_buf[1])*1.0)/recv_php_buf[5];
						DEBUG("face distance :%f angle:%f \n",distance,angle);

						if((cur_body.angle != angle) || (cur_body.distance != distance))
						{
							cur_body.distance = distance;
							cur_body.angle 	= angle;
							cur_body.newP = true;
						}
						else
						{
							cur_body.newP = false;
						}

					}
					else
					{
						cur_body.have_face=false;
					}
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
			DEBUG("cannot create listening socket");
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
				DEBUG("cannot bind server socket");
				//close(listen_fd);
				unlink(UNIX_DOMAIN);
				break;
			   }
			   while(true)
			   {
				recv_php_num = recvfrom(listen_fd, recv_php_buf, sizeof(recv_php_buf),
					0, (struct sockaddr *)&clt_addr, &len);
				DEBUG("\n=====body info===== ");
				for(i=0;i<16;i++) //0,flag 1,center-x 2,center-y,3,body_winth,4,body_height,5,body flag 1000=halfbody 1001=fullbody
				DEBUG("%d ",recv_php_buf[i]);
				DEBUG("\n");

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
					DEBUG("body distance :%f angle:%f \n",distance,angle);

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
//cyc people detect end

int main(int argc, char* argv[])
{
	int server_tcp_sockfd;
    int server_tcp_len;
    struct sockaddr_in server_tcp_address;

	init_prodcon(&pro_tcp_receive);
	init_prodcon(&pro_tcp_send);

	//only keep 10 log files
	system("find /home/linaro/dev/examples/ -type f -name 'log_snav*'|xargs -r ls -lt|tail -n +5|awk '{print $9}'| xargs rm -rf");

	time_t now;
	struct tm *curTime;
	now = time(NULL);
	curTime = localtime(&now);

	char log_filename[256];
	sprintf(log_filename,"/home/linaro/dev/examples/log_snav_%04d-%02d-%02d-%02d-%02d-%02d",
							curTime->tm_year+1900,curTime->tm_mon+1,curTime->tm_mday,
							curTime->tm_hour,curTime->tm_min,curTime->tm_sec);
	DEBUG("log_filename=%s\n", log_filename);

	FILE *fp;
	if ((fp = fopen(log_filename, "w+")) != NULL)
	{
		fclose(fp);
	}

	freopen(log_filename, "a", stdout); setbuf(stdout, NULL);
	freopen(log_filename, "a", stderr); setbuf(stderr, NULL);

	//create the snav_handler process of snav
	bool handler_flag = false;
	while(!handler_flag)
	{
		pthread_t handler_thread;
	    pthread_attr_t thread_attr;
	    int result;

	    result = pthread_attr_init(&thread_attr);
	    if(result !=0)
	    {
	        perror("Attribute creation failed");
	        continue;
	    }

	    result = pthread_attr_setdetachstate(&thread_attr,PTHREAD_CREATE_DETACHED);
	    if(result !=0)
	    {
	        perror("Setting detached attribute failed");
			pthread_attr_destroy(&thread_attr);
	        continue;
	    }

		DEBUG("create snav_handler thread\n");

		result = pthread_create(&handler_thread,&thread_attr,snav_handler, NULL);
		if(result !=0)
	    {
	    	perror("Thread snav_handler creation failed");
	    	pthread_attr_destroy(&thread_attr);
	        continue;
	    }
		else
		{
			handler_flag = true;
		}

		pthread_attr_destroy(&thread_attr);
	}

	//create the face_body_follow process of snav
	bool face_body_follow_flag = false;
	while(!face_body_follow_flag)
	{
		pthread_t face_body_follow_thread;
	    pthread_attr_t thread_attr;
	    int result;

	    result = pthread_attr_init(&thread_attr);
	    if(result !=0)
	    {
	        perror("Attribute creation failed");
	        continue;
	    }

	    result = pthread_attr_setdetachstate(&thread_attr,PTHREAD_CREATE_DETACHED);
	    if(result !=0)
	    {
	        perror("Setting detached attribute failed");
	        pthread_attr_destroy(&thread_attr);
	        continue;
	    }

		DEBUG("create face_body_follow thread\n");

		result = pthread_create(&face_body_follow_thread,&thread_attr,getVideoFaceFollowParam, NULL);
		//result = pthread_create(&face_body_follow_thread,&thread_attr,getVideoBodyFollowParam, NULL);
	    if(result !=0)
	    {
	    	perror("Thread face_body_follow creation failed");
	    	pthread_attr_destroy(&thread_attr);
	        continue;
	    }
		else
		{
			face_body_follow_flag = true;
		}

		pthread_attr_destroy(&thread_attr);
	}


    server_tcp_sockfd=socket(AF_INET,SOCK_STREAM,0);

    server_tcp_address.sin_family=AF_INET;
    server_tcp_address.sin_addr.s_addr=htonl(INADDR_ANY);
    server_tcp_address.sin_port=htons(SERVER_TCP_PORT);
    server_tcp_len=sizeof(server_tcp_address);

    int bind_result = bind(server_tcp_sockfd,(struct sockaddr*)&server_tcp_address,server_tcp_len);
	DEBUG("bind_result=%d, errno=%d\n", bind_result,errno);

    int listen_result = listen(server_tcp_sockfd,MAX_SOCKET_CONNECTION);
	DEBUG("listen_result=%d, errno=%d\n", listen_result,errno);

    while (1)
    {
        int client_sockfd;
        struct sockaddr_in client_address;
        int client_len;
        char ip_address[16];
        client_arg* args;
        client_len = sizeof(client_address);

        client_sockfd = accept(server_tcp_sockfd,(struct sockaddr*)&client_address,(socklen_t*)&client_len);
		DEBUG("\naccept new socket\n");

		//tcp receive and send threads must be only one each alive
		DEBUG("tcp_receive_thread_flag=%d, tcp_send_thread_flag=%d\n", tcp_receive_thread_flag, tcp_send_thread_flag);
		if (tcp_receive_thread_flag || tcp_send_thread_flag)
		{
			close(client_sockfd);
            continue;
		}

        get_ip_address(ntohl(client_address.sin_addr.s_addr),ip_address);
        DEBUG("get connection from %s, client_sockfd=%d\n\n",ip_address,client_sockfd);

		args = (client_arg*)malloc(sizeof(client_arg));
        args->client_sockfd = client_sockfd;

        //////////////////////create tcp receive and send thread/////////////////////
        pthread_t receive_thread;
		pthread_t send_thread;
        pthread_attr_t thread_attr;
        int res;

        res=pthread_attr_init(&thread_attr);
        if(res !=0)
        {
            perror("Attribute creation failed");

            free(args);
            close(client_sockfd);
            continue;
        }

        res=pthread_attr_setdetachstate(&thread_attr,PTHREAD_CREATE_DETACHED);
        if(res !=0)
        {
            perror("Setting detached attribute failed");

            free(args);
            close(client_sockfd);
            continue;
        }

		DEBUG("create circle_receive thread\n");

        res=pthread_create(&receive_thread,&thread_attr,circle_receive, (void*)args);
        if(res !=0)
        {
        	perror("Thread circle_receive creation failed");

			tcp_receive_thread_flag = false;

        	pthread_attr_destroy(&thread_attr);
            free(args);
            close(client_sockfd);
            continue;
        }
		else
		{
			tcp_receive_thread_flag = true;
		}

		DEBUG("create circle_send thread\n");

		res=pthread_create(&send_thread,&thread_attr,circle_send, (void*)args);
		if(res !=0)
		{
			perror("Thread circle_send creation failed");

			pthread_cancel(receive_thread);
			tcp_receive_thread_flag = false;

			tcp_send_thread_flag = false;

			pthread_attr_destroy(&thread_attr);
			free(args);
			close(client_sockfd);
			continue;
		}
		else
		{
			tcp_send_thread_flag = true;
		}

		setsockopt(args->client_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout_tcp, sizeof(struct timeval));
		setsockopt(args->client_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout_tcp, sizeof(struct timeval));

        pthread_attr_destroy(&thread_attr);
    }

	fclose(stdout);
	fclose(stderr);

	return 0;
}

/*******************open/close qcamvid************************************/

/*
//Add to open/close qcamvid
int qcamvid_domain_socket_conn(const char *servername)
{
	int fd;

	// create a UNIX domain stream socket
	if ((fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0)
	{
		return(-1);
	}

	int len, rval;

	struct sockaddr_un client_un;
    memset(&client_un, 0, sizeof(client_un));

	client_un.sun_family = AF_UNIX;
	sprintf(client_un.sun_path, "%05d", getpid());

	len = offsetof(struct sockaddr_un, sun_path) + strlen(client_un.sun_path);
	unlink(client_un.sun_path); //in case it already exists

	if (bind(fd, (struct sockaddr *)&client_un, len) < 0)
	{
		rval=  -2;
	}
	else
	{
		//fill socket address structure with server's address
		memset(&client_un, 0, sizeof(client_un));
		client_un.sun_family = AF_UNIX;

		strcpy(client_un.sun_path, servername);
		len = offsetof(struct sockaddr_un, sun_path) + strlen(servername);
		if (connect(fd, (struct sockaddr *)&client_un, len) < 0)
		{
			rval= -4;
		}
		else
		{
			return (fd);
		}
	}

	int err;
	err = errno;
	close(fd);
	errno = err;
	return rval;
}

void qcamvid_domain_socket_close(int fd)
{
	close(fd);
}


void* sendFlagToQcamvid(bool bFlag)
{
	const char* UNIX_DOMAIN ="/tmp/qcam.domain";

	int connfd;
	connfd = qcamvid_domain_socket_conn(UNIX_DOMAIN);
	if(connfd<0)
	{
		DEBUG("Error[%d] when connecting...",errno);
		return 0;
	}

	DEBUG("Begin to recv/send...\n");
	int i,n,size;
	char buf[32];

	//=========·¢ËÍ======================
	memset(buf,0,32);
	strcpy(buf, bFlag?"on":"off");
	size = send(connfd, buf, 32, 0);

	if (size >= 0)
	{
		DEBUG("Data[%d] Sended:%s\n",size,buf);
	}
	else if (size == -1)
	{
		DEBUG("Error[%d] when Sending Data:%s.\n",errno,strerror(errno));
	}

#if 0
	//=========½ÓÊÕ=====================
	size = recv(connfd, rvbuf, 32, 0);   //MSG_DONTWAIT
	if(size>=0)
	{
		printf("Recieved Data[%d]:%c...%c\n",size,rvbuf[0],rvbuf[size-1]);
	}
	else if (size==-1)
	{
		printf("Error[%d] when recieving Data.\n",errno);
	}
#endif

	sleep(1);

	qcamvid_domain_socket_close(connfd);
	DEBUG("qcamvid_domain_socket client exited.\n");
}
*/





