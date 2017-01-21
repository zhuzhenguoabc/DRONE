# Copyright (c) 2015 Qualcomm Technologies, Inc.  All Rights Reserved.
# Qualcomm Technologies Proprietary and Confidential.


SNAV_DEV_LOCATION = /home/linaro/dev

CC = gcc
CXX = g++
CFLAGS = -Wall
CXXFLAGS = -Wall -std=c++0x
INCLUDES = -I${SNAV_DEV_LOCATION}/inc     # path to snav_interface.h
LIBS = -lsnav_arm -lm -lpthread

TARGET = ${SNAV_DEV_LOCATION}/examples/snav_send_esc_commands  \
	 ${SNAV_DEV_LOCATION}/examples/snav_calibration_manager \
	 ${SNAV_DEV_LOCATION}/examples/snav_test_receive_data  \
         ${SNAV_DEV_LOCATION}/examples/snav_dronecontroller_companion \
         ${SNAV_DEV_LOCATION}/examples/snav_go_home_optic_flow \
	 ${SNAV_DEV_LOCATION}/examples/snav_waypoint_follow \
	 ${SNAV_DEV_LOCATION}/examples/snav_send_led_colors \
	 ${SNAV_DEV_LOCATION}/examples/snav_test_receive_data_easy

all: $(TARGET)

% : %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LD_FLAGS) $< -o $@ $(LIBS)

% : %.c
	$(CC) $(CFLAGS) $(INCLUDES) $(LD_FLAGS) $< -o $@ $(LIBS)

clean:
	rm -f $(TARGET)
