#pragma once

#include "myoSPI.hpp"
#include "CommunicationData.h"
#include <algorithm>

struct pidController
{
	/*! \brief Constructor to instantiate a PID controller with the variables stored in a control_Parameters union.
	*
	*/
	pidController(control_Parameters_t*);

	/*! \brief Constructor to instantiate an uninitialised PID controller.
	*
	*/
	pidController();

	/*! \brief Calculate the control output.
	*
	* \param[in] pv The current process value of the system.
	* \param[in] sp The desired set point of the system.
	* \return The control output.
	*
	* \sa controllerControlLoop
	*/
	/*!< Calculate the control output when given the process variable and set point.*/
	int16_t outputCalc(float32 pv, float32 sp);

	float32 integral;/*!<Integral of the error*/
	float32 pgain;/*!<Gain of the proportional component*/
	float32 igain;/*!<Gain of the integral component*/
	float32 dgain;/*!<Gain of the differential component*/
	float32 forwardGain; /*!<Gain of  the feed-forward term*/
	float32 deadBand;/*!<Optional deadband threshold for the control response*/
	float32 lastError;/*!<Error in previous time-step, used to calculate the differential component*/
	float32 IntegralPosMax;/*<!Positive saturation limit for the integral term*/
	float32 IntegralNegMax;/*<!Negative saturation limit for the integral term*/


	int32_t outputPosMax; /*!< Maximum control output in the positive direction.*/
	int32_t outputNegMax; /*!< Maximum control output in the negative direction.*/
	float32 spPosMax;/*<!Positive limit for the set point.*/
	float32 spNegMax;/*<!Negative limit for the set point.*/
	float32 timePeriod;/*<!Update period of the controller in seconds*/
	uint32 tag; /*!< The control mode of the current controller, used for error checking.*/
	float32 radPerEncoderCount; /*!output shaft rotation (in rad) per encoder count */
	float32 polyPar[4]; /*! polynomial fit from displacement (d)  to tendon force (f) f=polyPar[0]+polyPar[1]*d +polyPar[2]*d^2+ +polyPar[3]*d^3+ +polyPar[4]*d^4 */
	float32 torqueConstant; /*!motor torque constant in Nm/A */
};
