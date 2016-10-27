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

// Waypoint utilities
#include "snav_waypoint_utils.hpp"
// Snapdragon Navigator
#include "snapdragon_navigator.h"

using namespace std;

//#define LOG_FILE "/usr/logs/snav_transfer_control.log"
#define LOG_FILE "/home/linaro/dev/examples/log_snav_transfer_control.txt"


#define MAX_SOCKET_CONNECTION 5
#define SOCKET_OVER_TIME 5
#define SERVER_TCP_PORT 14558
#define SERVER_UDP_PORT 14559

#define MAX_BUFF_LEN 512
#define HEADER_BUFF_LENGTH 4

#define STR_SEPARATOR  ","

#define HEART_BEAT_OVER_COUNT 6
#define SNAV_HEART_BEAT "heartbeat"

#define SNAV_TASK_SUCCEED 1
#define SNAV_TASK_FAILED 2

#define SNAV_CMD_CONROL "1000"

#define SNAV_CMD_TAKE_OFF "1001"
#define SNAV_CMD_LAND "1002"
#define SNAV_CMD_RETURN "1003"
#define SNAV_CMD_CIRCLE "1004"
#define SNAV_CMD_LINE_PLAN "1005"
#define SNAV_CMD_WITHOUT_HEAD "1006"
#define SNAV_CMD_GPS_FOLLOW "1007"
#define SNAV_CMD_SETTING_MODE "1008"
#define SNAV_CMD_FACE_CHECK "1009"
#define SNAV_CMD_HEIGHT_DISTANCE_LIMIT "1010"
#define SNAV_CMD_LIGHT_FRONT_SWITCH "1011"
#define SNAV_CMD_BRIGHTNESS "1012"
#define SNAV_CMD_SELF_CHECK "1013"
#define SNAV_CMD_FLOW_LOCATION_SWITCH "1014"
#define SNAV_CMD_PIC_SIZE "1015"
#define SNAV_CMD_VIDEO_SIZE "1016"
#define SNAV_CMD_PIC_FORMAT "1017"
#define SNAV_CMD_VIDEO_FORMAT "1018"
#define SNAV_CMD_MODE_CHECK "1019"
#define SNAV_CMD_ISO "1020"
#define SNAV_CMD_PIC_FILTER_STYLE "1021"
#define SNAV_CMD_VIDEO_FILTER_STYLE "1022"
#define SNAV_CMD_PIC_SNAP_MODE "1023"
#define SNAV_CMD_GET_FLY_PARAM "1024"

#define SNAV_CMD_MODIFY_SSID_PWD "1025"


#define SNAV_CMD_RETURN_TAKE_OFF "2001"
#define SNAV_CMD_RETURN_LAND "2002"
#define SNAV_CMD_RETURN_RETURN "2003"
#define SNAV_CMD_RETURN_CIRCLE "2004"
#define SNAV_CMD_RETURN_LINE_PLAN "2005"
#define SNAV_CMD_RETURN_WITHOUT_HEAD "2006"
#define SNAV_CMD_RETURN_GPS_FOLLOW "2007"
#define SNAV_CMD_RETURN_SETTING_MODE "2008"
#define SNAV_CMD_RETURN_FACE_CHECK "2009"
#define SNAV_CMD_RETURN_HEIGHT_DISTANCE_LIMIT "2010"
#define SNAV_CMD_RETURN_LIGHT_FRONT_SWITCH "2011"
#define SNAV_CMD_RETURN_BRIGHTNESS "2012"
#define SNAV_CMD_RETURN_SELF_CHECK "2013"
#define SNAV_CMD_RETURN_FLOW_LOCATION_SWITCH "2014"
#define SNAV_CMD_RETURN_PIC_SIZE "2015"
#define SNAV_CMD_RETURN_VIDEO_SIZE "2016"
#define SNAV_CMD_RETURN_PIC_FORMAT "2017"
#define SNAV_CMD_RETURN_VIDEO_FORMAT "2018"
#define SNAV_CMD_RETURN_MODE_CHECK "2019"
#define SNAV_CMD_RETURN_ISO "2020"
#define SNAV_CMD_RETURN_PIC_FILTER_STYLE "2021"
#define SNAV_CMD_RETURN_VIDEO_FILTER_STYLE "2022"
#define SNAV_CMD_RETURN_PIC_SNAP_MODE "2023"
#define SNAV_CMD_RETURN_GET_FLY_PARAM "2024"

#define SNAV_TASK_GET_INFO	"8001"

#define SNAV_TASK_GET_INFO_RESULT	"9001"


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
  OPTIC_FLOW_ERROR
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

typedef struct
{
	int client_sockfd;
}client_arg;

#define CRICLE_OVER -1

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


struct timeval timeout_tcp = {SOCKET_OVER_TIME,0};
struct timeval timeout_udp = {0,300000};


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
    client_arg* args=(client_arg*)arg;
    int sockfd=args->client_sockfd;

	int header_length = 0;
	byte buff_header[MAX_BUFF_LEN];

	int data_length = 0;
	byte buff_data[MAX_BUFF_LEN];

	int header_heart_beat_mission_count = 0;
	int data_heart_beat_mission_count = 0;

	int count = 0;

	printf("\ncircle_receive start!\n");

	pthread_mutex_lock(&pro_tcp_receive.lock);
	pro_tcp_receive.bSockExit = false;
	pthread_mutex_unlock(&pro_tcp_receive.lock);

	while(true)
	{
		//receive the header(length of the data)
		if ((header_length=recv(sockfd,buff_header,HEADER_BUFF_LENGTH,0))>0)
		{
			data_length = bytesToInt(buff_header, 0);

			printf("\ncircle_receive: header_length=%d, data_length=%d\n", header_length, data_length);
			fflush(stdout);

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

						//printf("circle_receive: i_recv_data_length=%d, buff_data=%s\n", i_recv_data_length, buff_data);
						//fflush(stdout);

						pthread_mutex_lock(&pro_tcp_receive.lock);
						//pthread_cond_wait(&pro_tcp_receive.flag_circle, &pro_tcp_receive.lock);

						//set the flag and data for the handler-thread check
						pro_tcp_receive.bflag = true;
						pro_tcp_receive.fd_socket = sockfd;
						memset(pro_tcp_receive.data,0,MAX_BUFF_LEN);
						memcpy(pro_tcp_receive.data, buff_data, MAX_BUFF_LEN);

						if (strcmp(pro_tcp_receive.data, SNAV_HEART_BEAT) != 0)
						{
							printf("******************data*******************\n\n");
							fflush(stdout);
						}

						printf("circle_receive prodcons fd_socket=%d, pro_tcp_receive=%s\n\n",
								pro_tcp_receive.fd_socket, pro_tcp_receive.data);
						fflush(stdout);

					    //pthread_cond_signal(&pro_tcp_receive.flag_handler);
					    pthread_mutex_unlock(&pro_tcp_receive.lock);
					}
					else if ((i_recv_data_length < 0)
							&& (errno == EINTR
								|| errno == EWOULDBLOCK
								|| errno == EAGAIN))
					{
						printf("circle_receive recv return i_recv_data_length=%d, errno=%d\n", i_recv_data_length, errno);
						fflush(stdout);

						data_heart_beat_mission_count++;

						if (data_heart_beat_mission_count >= HEART_BEAT_OVER_COUNT)
						{
							printf("circle_receive quit from data_heart_beat over times!\n\n");
							fflush(stdout);

							pthread_mutex_lock(&pro_tcp_receive.lock);
							pro_tcp_receive.bSockExit = true;
							pthread_mutex_unlock(&pro_tcp_receive.lock);

							close(sockfd);
    						pthread_exit(NULL);
						}
						else
						{
							continue;
						}
					}
					else
					{
						printf("circle_receive return i_recv_data_length=%d, errno=%d\n", i_recv_data_length, errno);
						printf("circle_receive quit from receive_data_socket closed!\n\n");
						fflush(stdout);

						pthread_mutex_lock(&pro_tcp_receive.lock);
						pro_tcp_receive.bSockExit = true;
						pthread_mutex_unlock(&pro_tcp_receive.lock);

						close(sockfd);
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
			printf("\ncircle_receive return header_length=%d, errno=%d\n", header_length, errno);
			fflush(stdout);

			header_heart_beat_mission_count++;

			if (header_heart_beat_mission_count >= HEART_BEAT_OVER_COUNT)
			{
				printf("circle_receive quit from header_heart_beat over times!\n\n");
				fflush(stdout);

				pthread_mutex_lock(&pro_tcp_receive.lock);
				pro_tcp_receive.bSockExit = true;
				pthread_mutex_unlock(&pro_tcp_receive.lock);

				close(sockfd);
				pthread_exit(NULL);
			}
			else
			{
				continue;
			}
		}
		else
		{
			printf("\ncircle_receive return header_length=%d, errno=%d\n", header_length, errno);
			printf("circle_receive quit from receive_header_socket closed!\n\n");
			fflush(stdout);

			pthread_mutex_lock(&pro_tcp_receive.lock);
			pro_tcp_receive.bSockExit = true;
			pthread_mutex_unlock(&pro_tcp_receive.lock);

			close(sockfd);
			pthread_exit(NULL);
		}

		count++;
	}

	printf("\ncircle_receive quit from while break!\n\n");
	fflush(stdout);

	pthread_mutex_lock(&pro_tcp_receive.lock);
	pro_tcp_receive.bSockExit = true;
	pthread_mutex_unlock(&pro_tcp_receive.lock);

	close(sockfd);
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

	byte buff_header[MAX_BUFF_LEN];
	byte buff_data[MAX_BUFF_LEN];

	int count = 0;

	printf("\ncircle_send start!\n");

	while(true)
	{
		pthread_mutex_lock(&pro_tcp_receive.lock);
		bLocalSocketFlag = pro_tcp_receive.bSockExit;
		pthread_mutex_unlock(&pro_tcp_receive.lock);

		if (bLocalSocketFlag)
		{
			printf("circle_send quit from circle_receive socket closed!\n\n");
			fflush(stdout);

			pthread_mutex_lock(&pro_tcp_receive.lock);
			pro_tcp_receive.bSockExit = false;
			pthread_mutex_unlock(&pro_tcp_receive.lock);

			close(sockfd);
			pthread_exit(NULL);
		}


		pthread_mutex_lock(&pro_tcp_send.lock);
		bLocalTcpFlag = pro_tcp_send.bflag;

		if (pro_tcp_send.bflag == true)
		{
			localTcpFd = pro_tcp_send.fd_socket;

			printf("\ncircle_send fd_socket=%d, pro_tcp_send=%s\n", pro_tcp_send.fd_socket, pro_tcp_send.data);
			fflush(stdout);

			memset(result_to_client,0,MAX_BUFF_LEN);
			memcpy(result_to_client, pro_tcp_send.data, MAX_BUFF_LEN);

			pro_tcp_send.bflag = false;
		}
	    pthread_mutex_unlock(&pro_tcp_send.lock);

		usleep(20000);	//avoid of cpu over usage

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

							printf("circle_send i_send_data_length=%d, errno=%d\n\n", i_send_data_length, errno);
							fflush(stdout);
						}
						else if ((i_send_data_length < 0)
									&& (errno == EINTR
										|| errno == EWOULDBLOCK
										|| errno == EAGAIN))
						{
							printf("circle_send i_send_data_length=%d, errno=%d\n\n", i_send_data_length, errno);
							fflush(stdout);

							data_heart_beat_mission_count++;

							if (data_heart_beat_mission_count >= HEART_BEAT_OVER_COUNT)
							{
								printf("circle_send quit from data_heart_beat_count over times!\n\n");
								fflush(stdout);

								close(sockfd);
	    						pthread_exit(NULL);
							}
							else
							{
								continue;
							}
						}
						else
						{
							printf("circle_send return i_send_data_length=%d, errno=%d\n", i_send_data_length, errno);
							printf("circle_send quit from send_data_socket closed!\n\n");
							fflush(stdout);

							close(sockfd);
							pthread_exit(NULL);
						}
					}
				}
				else if ((i_send_header_length < 0)
							&& (errno == EINTR
								|| errno == EWOULDBLOCK
								|| errno == EAGAIN))
				{
					printf("circle_send i_send_header_length=%d, errno=%d\n\n", i_send_header_length, errno);
					fflush(stdout);

					header_heart_beat_mission_count++;

					if (header_heart_beat_mission_count >= HEART_BEAT_OVER_COUNT)
					{
						printf("circle_send quit from header_heart_beat_count over times!\n\n");
						fflush(stdout);

						close(sockfd);
						pthread_exit(NULL);
					}
					else
					{
						continue;
					}
				}
				else
				{
					printf("circle_send return i_send_header_length=%d, errno=%d\n", i_send_header_length, errno);
					printf("circle_send quit from send_header_socket closed!\n\n");
					fflush(stdout);

					close(sockfd);
					pthread_exit(NULL);
				}
			}

			bLocalTcpFlag = false;
		}

		count++;
	}

	printf("circle_send quit from while break!\n\n");
	fflush(stdout);

	close(sockfd);
    pthread_exit(NULL);
}

void* handler(void* arg)
{
	char udp_receive_data[MAX_BUFF_LEN];
	char tcp_receive_data[MAX_BUFF_LEN];
	char result_to_client[MAX_BUFF_LEN];

	bool bLocalTcpFlag = false;
	int localTcpFd = -1;

	bool bLocalUdpFlag = false;

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

	//gps params
	GpsPosition posLast;
	GpsPosition posGpsCurrent;
	GpsPosition posGpsDestination;
	float destyaw = 0;
	float speed = 0;
	float distance_to_dest=0;
	SnRcCommandType curSendMode=SN_RC_OPTIC_FLOW_POS_HOLD_CMD;

	//gps postion fly
	std::vector<GpsPosition> gps_positions;

	//circle fly
	std::vector<Position> circle_positions;
	float radius = 2.5f;//meter

	float vel_target = 0.75;	 //m/sec
	int point_count = 72;
	float angle_per = 2*M_PI/point_count;

	int clockwise= 1;// anticlockwise = -1

	DroneState drone_state = DroneState::NORMAL;
	MissionState state = MissionState::UNKNOWN;
	int loop_counter = 0;

	static bool task_take_off_in_progress = true;

	printf("handler start!\n");

	SnavCachedData* snav_data = NULL;
	if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data) != 0)
	{
		printf("\nFailed to get flight data pointer!\n");
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

			//printf("\nudp recvfrom received packet from %s:\n",inet_ntoa(remote_addr.sin_addr));
			udp_buff_data[length]='\0';
			printf("\nudp recvfrom get data udp_buff_data=%s, time_now=%lf\n",udp_buff_data, time_now);

			bLocalUdpFlag = true;
		}
		else
		{
			printf("\nudp recvfrom return length=%d, errno=%d\n", length, errno);
			fflush(stdout);

			bLocalUdpFlag = false;
		}

		//pro_tcp_receive for task operation
	    pthread_mutex_lock(&pro_tcp_receive.lock);
		bLocalTcpFlag = pro_tcp_receive.bflag;
		if (pro_tcp_receive.bflag == true)
		{
			localTcpFd = pro_tcp_receive.fd_socket;

			printf("\nhandler prodcons fd_socket=%d, pro_tcp_receive=%s\n",
					pro_tcp_receive.fd_socket, pro_tcp_receive.data);
			fflush(stdout);

			memset(tcp_receive_data,0,MAX_BUFF_LEN);
			memcpy(tcp_receive_data, pro_tcp_receive.data, MAX_BUFF_LEN);

			pro_tcp_receive.bflag = false;
		}
	    pthread_mutex_unlock(&pro_tcp_receive.lock);


		// Always need to call this
		if (sn_update_data() != 0)
		{
			printf("sn_update_data failed\n");
		}
		else
		{
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

			std::string recv_tcp_cmd, recv_udp_cmd;
			std::vector<std::string> gpsparams_tcp, gpsparams_udp;

			//get the data from tcp socket(task data) and udp socket(control data)
			if (bLocalTcpFlag)
			{
				recv_tcp_cmd = tcp_receive_data;
				gpsparams_tcp = split(recv_tcp_cmd,STR_SEPARATOR);
				printf("tcp task operation:%s\n\n",tcp_receive_data);
				fflush(stdout);
			}

			if (bLocalUdpFlag)
			{
				recv_udp_cmd = udp_buff_data;	//udp_receive_data;
				gpsparams_udp = split(recv_udp_cmd,STR_SEPARATOR);
				printf("udp control operation:%s\n",udp_buff_data);
				fflush(stdout);
			}

			printf("task_take_off_in_progress:%d\n",task_take_off_in_progress);

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

					printf("Control operation! props_state=%d\n", props_state);
					fflush(stdout);

					rolli = atoi(gpsparams_udp[1].c_str());
					pitchi = atoi(gpsparams_udp[2].c_str());
					yawi = atoi(gpsparams_udp[3].c_str());
					thrusti = atoi(gpsparams_udp[4].c_str());
					buttons = atoi(gpsparams_udp[5].c_str());

					printf("SNAV_CMD_CONROL rolli,pitchi,yawi,thrusti,buttons:%d,%d,%d,%d,%d\n",
										rolli,pitchi,yawi,thrusti,buttons);
					fflush(stdout);

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

						printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
						printf("UDP SNAV_SEND_CMD cmd0,cmd1,cmd2,cmd3:%f,%f,%f,%f\n",
										cmd0,cmd1,cmd2,cmd3);
						printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n");
						fflush(stdout);

						// Send the commands to Snapdragon Navigator with default RC options
		        		sn_send_rc_command(type, RC_OPT_DEFAULT_RC, cmd0, cmd1, cmd2, cmd3);
					}

					//for heartbeat send back
					if (bLocalTcpFlag)
					{
						pthread_mutex_lock(&pro_tcp_send.lock);

						//set the flag and data for the handler-thread check
						pro_tcp_send.bflag = true;
						pro_tcp_send.fd_socket = localTcpFd;
						memset(pro_tcp_send.data,0,MAX_BUFF_LEN);

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
								printf("battery_info=%s\n",battery_info);


								sprintf(rpm_info,"rpm_info:%d:%d:%d:%d",snav_data->esc_raw.rpm[0], snav_data->esc_raw.rpm[1]
																		   ,snav_data->esc_raw.rpm[2], snav_data->esc_raw.rpm[3]);
								printf("rpm_info=%s\n",rpm_info);

								sprintf(sonar_info,"sonar_info:%f",snav_data->sonar_0_raw.range);
								printf("sonar_info=%s\n",sonar_info);


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
								printf("gps_info=%s\n",gps_info);

								sprintf(xyz_info, "xyz_info:%f:%f:%f",snav_data->high_level_control_data.position_estimated[0]
																	 ,snav_data->high_level_control_data.position_estimated[1]
																	 ,snav_data->high_level_control_data.position_estimated[2]);
								printf("xyz_info=%s\n",xyz_info);

								sprintf(rpy_info, "rpy_info:%f:%f:%f",snav_data->attitude_estimate.roll
																	 ,snav_data->attitude_estimate.pitch
																	 ,snav_data->attitude_estimate.yaw);
								printf("rpy_info=%s\n",rpy_info);

								sprintf(flight_state_info, "flight_state_info:%d",state);
								printf("flight_state_info=%s\n",flight_state_info);

								sprintf(drone_state_info, "drone_state_info:%d",drone_state);
								printf("drone_state_info=%s\n",drone_state_info);


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

								printf("rpy_info=%s\n",result_to_client);

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
					printf("Error: GPS disabled. Desired state will be incorrect for optic flow modes\n");

					loop_counter++;

					continue;
				}

				SnDataStatus gps_status = (SnDataStatus) snav_data->data_status.gps_0_status;
				if (gps_status != SN_DATA_VALID)
				{
					printf("cant get gps location \n");
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

			// Position at startup
			static float x_est_startup = 0;
			static float y_est_startup = 0;
			static float z_est_startup = 0;
			static float yaw_est_startup = 0;

			// Mission State Machine
			static size_t current_position = 0;

			if ((gpsparams_tcp.size() >= 2)
				&& (gpsparams_tcp[0].compare(SNAV_CMD_MODIFY_SSID_PWD) == 0))
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

				system("ps -e |grep hostapd |awk '{print $1}'| xargs kill -9");
				system(sed_str);
				system("hostapd -B /etc/hostapd.conf");
				system("hostapd -B /etc/hostapd.conf");
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
					else if(gpsparams_tcp[0].compare("gps_waypiont") == 0)
					{
						state = MissionState::STARTING_PROPS;

						if(gps_positions.size()>2)
						{
							gps_waypiont_mission = true;
						}

						clockwise =1; //anticlockwise = -1

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
						printf("gps_point_collect_finish.\n");
						gps_point_collect_mission = false;

						for(int k=0;k<gps_positions.size();k++)
						{
							printf("gps position collect finished #%u: [%d,%d,%d,%f] ",
							k,gps_positions[k].latitude,gps_positions[k].longitude,
							gps_positions[k].altitude,gps_positions[k].yaw);
						}
					}
					else
					{
						state = MissionState::ON_GROUND;
					}

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
				//system("ps -e |grep qcamvid |awk '{print $1}'| xargs kill -9");
				//system("/usr/bin/qcamvid");

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

					if (props_state == SN_PROPS_STATE_SPINNING && on_ground_flag == 1)
					{
						// Snapdragon Navigator has determined that vehicle is on ground,
						// so it is safe to kill the propellers
						sn_stop_props();
					}

					if (props_state == SN_PROPS_STATE_NOT_SPINNING)
					{
						state = MissionState::ON_GROUND;
						//system("ps -e |grep qcamvid |awk '{print $1}'| xargs kill -9");
					}
				}
				else
				{
					state = MissionState::ON_GROUND;
					//system("ps -e |grep qcamvid |awk '{print $1}'| xargs kill -9");

					circle_misson=false;
					calcCirclePoint = false;
					gps_waypiont_mission = false;
					gps_point_collect_mission = false;
					followme_mission = false;
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

				if(gpsparams_tcp.size() >= 1 && (gpsparams_tcp[0].compare(SNAV_CMD_LAND) ==0))
				{
					state = MissionState::LANDING;
					loop_counter++;
					continue;
				}

				if(gps_waypiont_mission && gps_positions.size()>2)
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
				else if(circle_misson)
				{
					command_diff_x = circle_positions[current_position].x - (x_des-x_est_startup);
					command_diff_y = circle_positions[current_position].y - (y_des-y_est_startup);
					command_diff_z = circle_positions[current_position].z - (z_des-z_est_startup);
					command_diff_yaw = circle_positions[current_position].yaw - yaw_des;


					printf("[%d] [circle_misson x_des y_des z_des]: [%f %f %f]\n",
							loop_counter,x_des,y_des,z_des);

					if (command_diff_yaw > M_PI)
					{
						command_diff_yaw = command_diff_yaw - 2*M_PI;
					}
					else if (command_diff_yaw < -M_PI)
					{
						command_diff_yaw = command_diff_yaw+ 2*M_PI;
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
							circle_misson=false;
							calcCirclePoint = false;
						}
					}

					vel_x_target = command_diff_x/distance_to_dest * vel_target;
					vel_y_target = command_diff_y/distance_to_dest * vel_target;
					vel_z_target = command_diff_z/distance_to_dest * vel_target;
					vel_yaw_target = command_diff_yaw/angle_per*vel_target;

					printf("[%d] [circle_misson vel_x_target vel_y_target vel_z_target vel_yaw_target]: [%f %f %f %f]\n",
							loop_counter,vel_x_target,vel_y_target,vel_z_target,vel_yaw_target);

					printf("[%d] [distance_to_dest command_diff_x command_diff_y command_diff_z command_diff_yaw]: [%f %f %f %f %f]\n",
							loop_counter,distance_to_dest,command_diff_x,command_diff_y,command_diff_z,command_diff_yaw);

					//if (current_position >= 0.9*circle_positions.size()) //slow down
					//	vel_yaw_target =0;
					//if(vel_yaw_target>0.8f)vel_yaw_target=0.8f;
				}
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

					printf("[%d] [vel_x_target vel_y_target vel_z_target vel_yaw_target]: [%f %f %f %f]\n",
									loop_counter,vel_x_target,vel_y_target,vel_z_target,vel_yaw_target);
				}


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

				printf("[%d] [vel_x_diff,vel_y_diff,vel_z_diff,vel_yaw_diff]: [%f,%f,%f,%f] \n",
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

					/*if(gpsparams_tcp.size() >= 4 && gpsparams_tcp[0].compare(SNAV_CMD_CIRCLE) == 0)*/
					if(gpsparams_tcp.size() >= 3 && gpsparams_tcp[0].compare(SNAV_CMD_CIRCLE) == 0)
					{
						curSendMode =SN_RC_OPTIC_FLOW_POS_HOLD_CMD;
						circle_misson = true;
						calcCirclePoint = true;
						/*kDesTakeoffAlt = atof(gpsparams_tcp[1].c_str());*/
						radius = atof(gpsparams_tcp[1].c_str());	//2
						/*vel_target = atof(gpsparams_tcp[2].c_str());	//3
						printf("LOITER circle: kDesTakeoffAlt,radius,vel_target:%f,%f,%f\n",
								kDesTakeoffAlt,radius,vel_target);*/

						if (radius < 1)
						{
							point_count = 36;
							vel_target = 0.5;	 //m/sec
						}
						else
						{
							point_count = (int)(radius*36);
							vel_target = 0.5*radius;	 //m/sec
						}

						angle_per = 2*M_PI/point_count;

						printf("LOITER circle: radius,point_count,vel_target,angle_per:%f,%d,%f,%f\n",
								radius,point_count,vel_target,angle_per);

					}

					if(gpsparams_tcp.size() >= 1 && (gpsparams_tcp[0].compare(SNAV_CMD_LAND) ==0))
					{
						state = MissionState::LANDING;
					}
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
							printf("get gps data,TRAJECTORY_FOLLOW\n");
						}
						else
						{
							state = MissionState::LOITER;
						}
					}
					else if(circle_misson)
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
								circle_center_y = y_est-yaw_est_startup + radius*sin(yaw_est);

								circle_positions.clear();//clear and recaculate.
								for(int k=0;k<point_count;k++)
								{
									Position pos;
									//pos.x = (1-cos(angle_per*k))*radius + x_est;
									//pos.y = -sin(angle_per*k)*radius + y_est;

									yaw_t = yaw_est+angle_per*k*clockwise;

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

								printf("@@@@circle_center_point [%f,%f]\n",
										circle_center_x,circle_center_y);

								for(int k=0;k<circle_positions.size();k++)
								{
									printf("@@@@[%d] position #%u: [%f,%f,%f,%f]\n",k,
										k,circle_positions[k].x,circle_positions[k].y,
										circle_positions[k].z,circle_positions[k].yaw);
								}
							}

							state = MissionState::TRAJECTORY_FOLLOW;
							entering_loiter = true;
						}
					}
					else if((gpsparams_tcp.size() >= 3) && (gpsparams_tcp[0].compare(SNAV_CMD_GPS_FOLLOW) == 0))
					{
						//curSendMode =SN_RC_GPS_POS_HOLD_CMD;
						followme_mission = true;
						destyaw = (float)atof(gpsparams_tcp[1].c_str());
						speed = (float)atof(gpsparams_tcp[2].c_str());

						printf("destyaw:%f  speed:%f\n",destyaw,speed);
						float yaw_diff ;
						yaw_diff = destyaw - yaw_est;

						if(abs(yaw_diff) >0.05f)
						{
							state = MissionState::TRAJECTORY_FOLLOW;
							entering_loiter = true;
						}
						printf("followme yaw_diff:%f \n",yaw_diff);
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

				circle_misson=false;
				calcCirclePoint = false;
				gps_waypiont_mission = false;
				gps_point_collect_mission = false;
				followme_mission = false;
			}

			// Rotate velocity by estimated yaw angle before sending
			// This puts velocity in body-relative Z-up frame
			float x_vel_des_yawed = x_vel_des*cos(-yaw_est) - y_vel_des*sin(-yaw_est);
			float y_vel_des_yawed = x_vel_des*sin(-yaw_est) + y_vel_des*cos(-yaw_est);

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
			printf("[%d] battery_voltage: %f\n",loop_counter,voltage);
			printf("[%d] [x_est,y_est,z_est,yaw_est]: [%f,%f,%f,%f]\n",loop_counter,x_est-x_est_startup,y_est-y_est_startup,z_est-z_est_startup,yaw_est);
			printf("[%d] [x_des,y_des,z_des,yaw_des]: [%f,%f,%f,%f]\n",
					loop_counter,x_des-x_est_startup,y_des-y_est_startup,z_des-z_est_startup,yaw_des);

			printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
			printf("[%d] [x_est,y_est,z_est,yaw_est]: [%f,%f,%f,%f]\n",
					loop_counter,x_est,y_est,z_est,yaw_est);
			printf("[%d] [x_des,y_des,z_des,yaw_des]: [%f,%f,%f,%f]\n",
					loop_counter,x_des,y_des,z_des,yaw_des);
			printf("[%d] [x_est_startup,y_est_startup,z_est_startup,yaw_est_startup]: [%f,%f,%f,%f]\n",
					loop_counter,x_est_startup,y_est_startup,z_est_startup,yaw_est_startup);

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
				if(gps_waypiont_mission )
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

			//tcp 8001 task for get status and data information when tcp task
			if (bLocalTcpFlag)
			{
				pthread_mutex_lock(&pro_tcp_send.lock);

				//set the flag and data for the handler-thread check
				pro_tcp_send.bflag = true;
				pro_tcp_send.fd_socket = localTcpFd;
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
					printf("battery_info=%s\n",battery_info);


					sprintf(rpm_info,"rpm_info:%d:%d:%d:%d",snav_data->esc_raw.rpm[0], snav_data->esc_raw.rpm[1]
															   ,snav_data->esc_raw.rpm[2], snav_data->esc_raw.rpm[3]);
					printf("rpm_info=%s\n",rpm_info);

					sprintf(sonar_info,"sonar_info:%f",snav_data->sonar_0_raw.range);
					printf("sonar_info=%s\n",sonar_info);


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
					printf("gps_info=%s\n",gps_info);

					sprintf(xyz_info, "xyz_info:%f:%f:%f",snav_data->high_level_control_data.position_estimated[0]
														 ,snav_data->high_level_control_data.position_estimated[1]
														 ,snav_data->high_level_control_data.position_estimated[2]);
					printf("xyz_info=%s\n",xyz_info);

					sprintf(rpy_info, "rpy_info:%f:%f:%f",snav_data->attitude_estimate.roll
														 ,snav_data->attitude_estimate.pitch
														 ,snav_data->attitude_estimate.yaw);
					printf("rpy_info=%s\n",rpy_info);

					sprintf(flight_state_info, "flight_state_info:%d",state);
					printf("flight_state_info=%s\n",flight_state_info);

					sprintf(drone_state_info, "drone_state_info:%d",drone_state);
					printf("drone_state_info=%s\n",drone_state_info);

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

					printf("rpy_info=%s\n",result_to_client);

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

	printf("out from the handler thread\n\n");

    pthread_exit(NULL);
}


int main(int argc, char* argv[])
{
	int server_tcp_sockfd;
    int server_tcp_len;
    struct sockaddr_in server_tcp_address;

	init_prodcon(&pro_tcp_receive);
	init_prodcon(&pro_tcp_send);

	FILE *fp;

	//clear the file first
	if ((fp = fopen(LOG_FILE, "w+")) != NULL)
	{
		fclose(fp);
	}

	freopen(LOG_FILE, "a", stdout); setbuf(stdout, NULL);
	freopen(LOG_FILE, "a", stderr); setbuf(stderr, NULL);

	//create the handler process of snav
	bool handler_flag = false;
	while(!handler_flag)
	{
		pthread_t handler_thread;
	    pthread_attr_t thread_attr;
	    int result;

	    result=pthread_attr_init(&thread_attr);
	    if(result !=0)
	    {
	        perror("Attribute creation failed");
	        continue;
	    }

	    result=pthread_attr_setdetachstate(&thread_attr,PTHREAD_CREATE_DETACHED);
	    if(result !=0)
	    {
	        perror("Setting detached attribute failed");
	        continue;
	    }


		printf("create handler thread\n");
		fflush(stdout);

		result=pthread_create(&handler_thread,&thread_attr,handler, NULL);

	    if(result !=0)
	    {
	        perror("Thread handler creation failed");
	        continue;
	    }
		else
		{
			handler_flag = true;
		}

		pthread_attr_destroy(&thread_attr);
	}

	//tcp socket for task
    server_tcp_sockfd=socket(AF_INET,SOCK_STREAM,0);

    server_tcp_address.sin_family=AF_INET;
    server_tcp_address.sin_addr.s_addr=htonl(INADDR_ANY);
    server_tcp_address.sin_port=htons(SERVER_TCP_PORT);
    server_tcp_len=sizeof(server_tcp_address);

    bind(server_tcp_sockfd,(struct sockaddr*)&server_tcp_address,server_tcp_len);

    listen(server_tcp_sockfd,MAX_SOCKET_CONNECTION);

    while (1)
    {
        int client_sockfd;
        struct sockaddr_in client_address;
        int client_len;
        char ip_address[16];
        client_arg* args;
        client_len=sizeof(client_address);

        client_sockfd=accept(server_tcp_sockfd,(struct sockaddr*)&client_address,(socklen_t*)&client_len);

		printf("\naccept new socket\n");

        args=(client_arg*)malloc(sizeof(client_arg));
        args->client_sockfd=client_sockfd;

        get_ip_address(ntohl(client_address.sin_addr.s_addr),ip_address);
        printf("get connection from %s\n\n",ip_address);


        //////////////////////create a thread to process the query/////////////////////
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

		printf("create circle_receive thread\n");

        res=pthread_create(&receive_thread,&thread_attr,circle_receive, (void*)args);
        if(res !=0)
        {
            perror("Thread circle_receive creation failed");
            free(args);
            close(client_sockfd);
            continue;
        }

		printf("create circle_send thread\n");

		res=pthread_create(&send_thread,&thread_attr,circle_send, (void*)args);
		if(res !=0)
		{
			pthread_cancel(receive_thread);

			perror("Thread circle_send creation failed");
			free(args);
			close(client_sockfd);
			continue;
		}

		setsockopt(args->client_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout_tcp, sizeof(struct timeval));
		setsockopt(args->client_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout_tcp, sizeof(struct timeval));

        pthread_attr_destroy(&thread_attr);
    }

	fclose(stdout);
	fclose(stderr);

	return 0;
}
