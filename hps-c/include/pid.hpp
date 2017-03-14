/*
 * pid.hpp
 *
 *  Created on: Mar 2, 2017
 *      Author: letrend
 */
#pragma once

#define IORD(base,reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base,reg,data) (*(((volatile int32_t*)base)+reg)=data)

#define PID_READ_result(base) IORD(base, 0)
#define PID_READ_Kp(base) IORD(base, 1)
#define PID_READ_Kd(base) IORD(base, 2)
#define PID_READ_Ki(base) IORD(base, 3)
#define PID_READ_sp(base) IORD(base, 4)
#define PID_READ_forwardGain(base) IORD(base, 5)
#define PID_READ_outputPosMax(base) IORD(base, 6)
#define PID_READ_outputNegMax(base) IORD(base, 7)
#define PID_READ_IntegralNegMax(base) IORD(base, 8)
#define PID_READ_IntegralPosMax(base) IORD(base, 9)
#define PID_READ_deadBand(base) IORD(base, 10)
#define PID_READ_position(base) IORD(base, 11)
#define PID_READ_velocity(base) IORD(base, 12)
#define PID_READ_displacement(base) IORD(base, 13)

#define PID_WRITE_Kp(base,data) IOWR(base, 1, data)
#define PID_WRITE_Kd(base,data) IOWR(base, 2, data)
#define PID_WRITE_Ki(base,data) IOWR(base, 3, data)
#define PID_WRITE_sp(base,data) IOWR(base, 4, data)
#define PID_WRITE_forwardGain(base,data) IOWR(base, 5, data)
#define PID_WRITE_outputPosMax(base,data) IOWR(base, 6, data)
#define PID_WRITE_outputNegMax(base,data) IOWR(base, 7, data)
#define PID_WRITE_IntegralNegMax(base,data) IOWR(base, 8, data)
#define PID_WRITE_IntegralPosMax(base,data) IOWR(base, 9, data)
#define PID_WRITE_deadBand(base,data) IOWR(base, 10, data)
