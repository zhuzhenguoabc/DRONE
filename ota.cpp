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

using namespace std;

#define OTA_UDP_PORT 						14888
#define MAX_BUFF_LEN 						512

#define OTA_LINARO_PATH 					"/tmp/update-linaro.zip"
#define OTA_SNAV_PATH             			"/tmp/update-snav.zip"

#define __DEBUG

#ifdef __DEBUG
#define DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define DEBUG(format,...)
#endif


struct timeval timeout_ota_udp = {0, 300000};			//300ms

int main(int argc, char* argv[])
{
	//udp
	int server_udp_sockfd;
	int server_udp_len;
	struct sockaddr_in server_udp_address;

	server_udp_address.sin_family = AF_INET;
	server_udp_address.sin_addr.s_addr = htonl(INADDR_ANY);
	server_udp_address.sin_port = htons(OTA_UDP_PORT);
	server_udp_len = sizeof(server_udp_address);

	server_udp_sockfd = socket(AF_INET,SOCK_DGRAM,0);

	int bind_result = bind(server_udp_sockfd, (struct sockaddr*)&server_udp_address, server_udp_len);

	//300MS avoid of udp missing data
	setsockopt(server_udp_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout_ota_udp, sizeof(struct timeval));
	setsockopt(server_udp_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout_ota_udp, sizeof(struct timeval));

	while (true)
	{
		int length = 0;
		struct sockaddr_in remote_addr;
		int sin_size = sizeof(struct sockaddr_in);
		char udp_buff_data[MAX_BUFF_LEN];

		//receive the udp data
		length = recvfrom(server_udp_sockfd,udp_buff_data,MAX_BUFF_LEN-1,0, (struct sockaddr *)&remote_addr,(socklen_t*)&sin_size);

		if (length > 0)
		{
			DEBUG("udp recvfrom received data from %s,%d:\n", inet_ntoa(remote_addr.sin_addr),  ntohs(remote_addr.sin_port));
			udp_buff_data[length] = '\0';
			DEBUG("udp recvfrom get data udp_buff_data=%s\n", udp_buff_data);

			if (strcmp(udp_buff_data, OTA_LINARO_PATH) == 0)
			{
				system("stop snav");
				system("install-update /tmp/update-linaro.zip");
			}
			else if (strcmp(udp_buff_data, OTA_SNAV_PATH) == 0)
			{
				system("rm -rf /tmp/update-snav/");
				system("chmod 777 /tmp/update-snav.zip");
				system("tar xvf /tmp/update-snav.zip -C /tmp/");	//zxvf
				system("chmod -R 777 /tmp/update-snav/");
				system("/tmp/update-snav/update.sh");
			}
		}
		else
		{
			DEBUG("udp recvfrom return length=%d, errno=%d\n", length, errno);
		}
	}

	return 0;
}

