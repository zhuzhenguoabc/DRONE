# Copyright (c) 2015 Qualcomm Technologies, Inc.  All Rights Reserved.
# Qualcomm Technologies Proprietary and Confidential.

CC = arm-linux-gnueabihf-gcc
CXX = arm-linux-gnueabihf-g++

CFLAGS = -Wall
CXXFLAGS = -Wall -std=c++0x

#INCLUDES = -I/mnt/workspace/snav_example/home/linaro/dev/inc     
#LDFLAGS=-L/mnt/workspace/snav_example/usr/local/lib
#LIBS = -lsnav_arm -ladsprpc -L/mnt/workspace/snav_example/usr/local/lib 

INCLUDES = -I./inc     
LDFLAGS=-L./lib
LIBS = -lsnav_arm -ladsprpc -L./lib 

TARGET = snav_transfer_control  

all: $(TARGET)

% : %.cpp 
	$(CXX) $(CXXFLAGS) $(INCLUDES) -lpthread $(LD_FLAGS) $< -o $@ $(LIBS)	
	
% : %.c 
	$(CC) $(CFLAGS) $(INCLUDES) -lpthread $(LD_FLAGS) $< -o $@ $(LIBS)

clean:
	rm -f $(TARGET)

