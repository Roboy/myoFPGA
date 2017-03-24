#
TARGET = spi_control
ALT_DEVICE_FAMILY ?= soc_cv_av
HWLIBS_ROOT = $(SOCEDS_DEST_ROOT)/ip/altera/hps/altera_hps/hwlib
POWERLINK_ROOT = /home/roboy/Downloads/openPowerLink
POWERLINK_LIBS = /home/roboy/workspace/myoFPGA/hps-c/lib/linux/armv7l

#
CONFIG_USE_PCAP = 1
CROSS_COMPILE = arm-linux-gnueabihf-
CFLAGS = -static -g -Wall -std=c++11 -I${HWLIBS_ROOT}/include -I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY) -D$(ALT_DEVICE_FAMILY) -Iinclude/ -Incurses_arm/include -I$(POWERLINK_ROOT)/stack/include -I$(POWERLINK_ROOT)/apps/common/src -I$(POWERLINK_ROOT)/contrib
LDFLAGS =  -g -Wall -Lncurses_arm/lib -lncurses -L$(POWERLINK_LIBS) -lpcap -loplkmn
CC = $(CROSS_COMPILE)gcc
CPP = $(CROSS_COMPILE)g++
ARCH= arm

CPP_FILES := $(wildcard src/*.cpp)
C_FILES := $(wildcard src/*.c)
CPP_OBJ_FILES := $(addprefix build/,$(notdir $(CPP_FILES:.cpp=.o)))
C_OBJ_FILES := $(addprefix build/,$(notdir $(CPP_FILES:.cpp=.o)))

OUTPUTDIR = ./build/
MKDIR = mkdir -p $(OUTPUTDIR)

.PHONY : all
all : 
	$(MKDIR)
	$(CC) $(C_FILES) $(CFLAGS) -o $(OUTPUTDIR)$(TARGET)
	$(CPP) $(CPP_FILES) $(TARGET).o $(CPP_FILES) -o $(OUTPUTDIR)$(TARGET)

.PHONY: clean
clean:
	rm -f $(TARGET) build/*.a build/*.o build/*~ 
