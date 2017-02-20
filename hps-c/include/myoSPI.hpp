#pragma once

#include <inttypes.h>
#include <stdint.h>
#include <unistd.h>

#include "spi.hpp"

#define WRITE_DATA      1
#define READ_DATA       0

struct SPISTREAM {
  union {
	struct {
	  uint16_t startOfFrame;
	  int16_t pwmRef;
	  uint16_t controlFlags1 : 16;
	  uint16_t controlFlags2 : 16;
	  uint16_t dummy : 16;
	  int32_t actualPosition : 32;
	  int16_t actualVelocity : 16;
	  int16_t actualCurrent : 16;
	  int16_t springDisplacement : 16;
	  int16_t sensor1 : 16;
	  int16_t sensor2 : 16;
	};
	uint8_t TxBuffer[24];
	uint16_t TxBuffer2[12];
  };
};

void swap(uint8_t *source, uint8_t *target);

void prepareData(SPISTREAM *spistream, int datatype);

void convertEndianess(SPISTREAM *frame_in, SPISTREAM *frame_out);

void exchangeFrame( uint32_t *base, uint32_t slave, SPISTREAM *frame);
