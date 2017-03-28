#pragma once

#include <vector>
#include <map>
#include <iostream>
#include <math.h>
#include <chrono>
#include <fstream>
#include "myoControlRegister.hpp"

#include "xap.h"

#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include <system/system.h>
#include <obdcreate/obdcreate.h>
#include <getopt/getopt.h>
#include <console/console.h>
#include <eventlog/eventlog.h>
#include <limits.h>

#if defined(CONFIG_USE_PCAP)
#include <pcap/pcap-console.h>
#endif

#include <stdio.h>
#include <limits.h>

#define CYCLE_LEN         50000
#define NODEID            1                   // could be changed by command param
#define IP_ADDR           0xc0a86401          // 192.168.100.1
#define DEFAULT_GATEWAY   0xC0A864FE          // 192.168.100.C_ADR_RT1_DEF_NODE_ID
#define SUBNET_MASK       0xFFFFFF00          // 255.255.255.0

using namespace std;
using namespace std::chrono;

typedef struct
{
    int32_t outputPosMax; /*!< maximum control output in the positive direction in counts, max 4000*/
    int32_t outputNegMax; /*!< maximum control output in the negative direction in counts, max -4000*/
    int32_t spPosMax;/*<!Positive limit for the set point.*/
    int32_t spNegMax;/*<!Negative limit for the set point.*/
	uint16_t Kp;/*!<Gain of the proportional component*/
	uint16_t Ki;/*!<Gain of the integral component*/
	uint16_t Kd;/*!<Gain of the differential component*/
	uint16_t forwardGain; /*!<Gain of  the feed-forward term*/
	uint16_t deadBand;/*!<Optional deadband threshold for the control response*/
	int16_t IntegralPosMax; /*!<Integral positive component maximum*/
	int16_t IntegralNegMax; /*!<Integral negative component maximum*/
	vector<float> polyPar; /*! polynomial fit from displacement (d)  to tendon force (f)
				 f=polyPar[0]+polyPar[1]*d +polyPar[2]*d^2+ +polyPar[3]*d^3 + ... */
	float radPerEncoderCount;
}control_Parameters_t;

typedef struct
{
    UINT32          nodeId;
    tEventlogFormat logFormat;
    UINT32          logLevel;
    UINT32          logCategory;
    char            devName[128];
} tOptions;

enum CONTROLMODE{
	POSITION,
	VELOCITY,
	DISPLACEMENT,
	FORCE
};

struct MotorStatus{
    // controlled node output/ managing node input
    signed pwmRef_I16:16;
    signed actualPosition_I32:32;
    signed actualVelocity_I16:16;
    signed actualCurrent_I16:16;
    signed springDisplacement_I16:16;
    signed sensor1_I16:16;
    signed sensor2_I16:16;
};
struct SetPoints{
    signed CN1_MotorCommand_setPoint_I32_1:32;
    signed CN1_MotorCommand_setPoint_I32_2:32;
    signed CN1_MotorCommand_setPoint_I32_3:32;
    signed CN1_MotorCommand_setPoint_I32_4:32;
    signed CN1_MotorCommand_setPoint_I32_5:32;
    signed CN1_MotorCommand_setPoint_I32_6:32;
    signed CN1_MotorCommand_setPoint_I32_7:32;
    signed CN1_MotorCommand_setPoint_I32_8:32;
};

class MyoControl{
public:
	MyoControl(vector<int32_t*> &myo_base, int argc, char* argv[]);
	~MyoControl();
	/**
	 * This is the main loop, receiving commands and sending motor status via powerlink
	 */
	void mainLoop();
	/**
	 * Changes the controller of a motor
	 * @param motor for this motor
	 * @param mode choose from Position, Velocity or Displacement
	 * @param params with these controller parameters
	 */
	void changeControl(int motor, int mode, control_Parameters_t &params);
	/**
	 * Changes the controller of a motor with the saved controller parameters
	 * @param motor for this motor
	 * @param mode choose from Position, Velocity or Displacement
	 */
	void changeControl(int motor, int mode);
	/**
	 * Changes the controller of ALL motors with the saved controller parameters
	 * @param mode choose from Position, Velocity or Displacement
	 */
	void changeControl(int mode);
	/**
	 * Toggles SPI transmission
	 * @return on/off
	 */
	bool toggleSPI();
	/**
	 * Resets all myo controllers
	 */
	void reset();
	/**
	 * Changes setpoint for position controller
	 * @param motor for this motor
	 * @param position the new setpoint
	 */
	void setPosition(int motor, int32_t position);
	/**
	 * Changes setpoint for velocity controller
	 * @param motor for this motor
	 * @param velocity the new setpoint
	 */
	void setVelocity(int motor, int32_t velocity);
	/**
	 * Changes setpoint for displacement controller
	 * @param motor for this motor
	 * @param displacement the new setpoint
	 */
	void setDisplacement(int motor, int32_t displacement);
	/**
	 * Get the parameters for the PID controller of a motor
	 * @param motor for this motor
	 */
	void getPIDcontrollerParams(int &Pgain, int &Igain, int &Dgain, int &forwardGain, int &deadband,
									int &setPoint, int &setPointMin, int &setPointMax, int motor);
	/**
	 * Gets the current control_mode of a motor
	 * @param motor for this motor
	 */
	uint16_t getControlMode(int motor);
	/**
	 * Gets the current pwm of a motor
	 * @param motor for this motor
	 */
    static int16_t getPWM(int motor);
	/**
	 * Gets the current position of a motor in radians
	 * @param motor for this motor
	 */
    static int32_t getPosition(int motor);
	/**
	 * Gets the current velocity of a motor in radians/seconds
	 * @param motor for this motor
	 */
    static int16_t getVelocity(int motor);
	/**
	 * Gets the displacement in encoder ticks
	 * @param motor for this motor
	 */
    static int16_t getDisplacement(int motor);
	/**
	 * Gets the current in Ampere
	 * @param motor for this motor
	 */
    static int16_t getCurrent(int motor);

	/**
	 * Fills the given params with default values for the corresponding control mode
	 * @param params pointer to control struct
	 * @param control_mode Position, Velocity, Force
	 */
	void getDefaultControlParams(control_Parameters_t *params, int control_mode);

	/**
	 * Changes the control mode for all motors to Position
	 * @param pos new setPoint
	 */
	void allToPosition(int32_t pos);
	/**
	 * Changes the control mode for all motors to Velocity
	 * @param pos new setPoint
	 */
	void allToVelocity(int32_t vel);
	/**
	 * Changes the control mode for all motors to Displacement
	 * @param force new setPoint
	 */
	void allToDisplacement(int32_t displacement);
	/**
	 * Zeros the current weight
	 */
	void zeroWeight();
	/**
	 * Returns the current weight according to adc_weight_parameters
	 */
	float getWeight();
	/**
	 * Estimates the spring parameters of a motor by pulling with variable forces
	 * keeping track of displacement and weight, it will either timeout or stop when the
	 * requested number of samples was reached
	 * @param motor for this motor
	 * @param timeout in milliseconds
	 * @param numberOfDataPoints how many samples do you wanne collect
	 */
	void estimateSpringParameters(int motor, int timeout, uint numberOfDataPoints);
	/**
	 * Performs polynomial regression (http://www.bragitoff.com/2015/09/c-program-for-polynomial-fit-least-squares/)
	 * @param degree (e.g. 2 -> a * x^0 + b * x^1 + c * x^2)
	 * @param coeffs the estimated coefficients
	 * @param X the x-data
	 * @param Y the y-data
	 */
	void polynomialRegression(int degree, vector<float> &x, vector<float> &y,
			vector<float> &coeffs);

	map<int,map<int,control_Parameters_t>> control_params;
	uint32_t *adc_base = nullptr;
	float weight_offset = 0;
	float adc_weight_parameters[2] = {830.7, -0.455};
	bool spi_active = false;
	uint numberOfMotors;
private:
    /**
     * This initializes the process image for openPowerLink
     * @return errorCode
     */
    tOplkError initProcessImage();
    /**
     * The function parses the supplied command line parameters and stores the
     * options at pOpts_p.
     * @param argc_p Argument count.
     * @param argv_p Pointer to arguments.
     * @param pOpts_p Pointer to store options
     * @return The function returns the parsing status.
     * @retval 0 Successfully parsed
     * @retval -1 Parsing error
    */
    int getOptions(int argc_p,
                   char* const argv_p[],
                   tOptions* pOpts_p);
    /**
     * The function initializes the openPOWERLINK stack.
     * @param cycleLen_p          Length of POWERLINK cycle.
     * @param devName_p           Device name string.
     * @param macAddr_p           MAC address to use for POWERLINK interface.
     * @param nodeId_p            POWERLINK node ID.
     * @return The function returns a tOplkError error code.
    */
    tOplkError initPowerlink(UINT32 cycleLen_p,
                             const char* devName_p,
                             const UINT8* macAddr_p,
                             UINT32 nodeId_p);
    /**
     * The function implements the synchronous data handler.
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processSync();
    /** Shuts down powerLink */
    void shutdownPowerlink();
    /**
     * The function processes error and warning events.
     * @param  EventType_p         Type of event
     * @param  pEventArg_p         Pointer to union which describes the event in detail
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processEvents(tOplkApiEventType EventType_p,
                             const tOplkApiEventArg* pEventArg_p,
                             void* pUserArg_p);
    /**
     * The function processes PDO change events.
     * @param  EventType_p         Type of event
     * @param  pEventArg_p         Pointer to union which describes the event in detail
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processPdoChangeEvent(tOplkApiEventType EventType_p,
                                     const tOplkApiEventArg* pEventArg_p,
                                     void* pUserArg_p);
    /**
     * The function processes error and warning events.
     * @param  EventType_p         Type of event
     * @param  pEventArg_p         Pointer to union which describes the event in detail
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processErrorWarningEvent(tOplkApiEventType EventType_p,
                                        const tOplkApiEventArg* pEventArg_p,
                                        void* pUserArg_p);
    /**
     * The function processes state change events.
     * @param  EventType_p         Type of event
     * @param  pEventArg_p         Pointer to union which describes the event in detail
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processStateChangeEvent(tOplkApiEventType EventType_p,
                                       const tOplkApiEventArg* pEventArg_p,
                                       void* pUserArg_p);
public:
    static vector<int32_t*> myo_base;
    static vector<SetPoints> setPoints;
    static vector<MotorStatus> motorStatus;
    static uint8_t motor_selecta;
    static PI_IN*   pProcessImageIn_l;
    static PI_OUT*  pProcessImageOut_l;
};
