#pragma once

#include "myoSPI.hpp"
#include "CommunicationData.h"
#include <algorithm>

#define controllerDISABLE 0
#define controllerENABLE 1

class pidController
{

public:

	/*! \brief Constructor to instantiate a PID controller with the variables stored in a control_Parameters union.
	*
	*/
	pidController(control_Parameters_t*);

	/*! \brief Constructor to instantiate an uninitialised PID controller.
	*
	*/
	pidController();

	/*! \brief
	*
	*	Set the integral term to a given value.
	*
	*	\param new_integ The value to which the integral term will be set.
	*/
	void pid_setinteg(float new_integ);

	/*! \brief Get the gain of the P element of the controller.
	*
	* \return The P gain of the controller.
	*/
	float get_pgain(){return pgain;}

	/*! \brief Get the gain of the I element of the controller.
	*
	* \return The I gain of the controller.
	*/
	float get_igain(){return igain;}

	/*! \brief Get the gain of the D element of the controller.
	*
	* \return The D gain of the controller.
	*/
	float get_dgain(){return dgain;}

	/*! \brief Set the P gain of the controller.
	*
	* \param gain The gain value to be set.
	* \return 0 on success, 1 if an illegal value is requested.
	*/
	int set_pgain(float gain);

	/*! \brief Set the I gain of the controller.
	*
	* \param gain The gain value to be set.
	* \return 0 on success, 1 if an illegal value is requested.
	*/
	int set_igain(float gain);

	/*! \brief Set the D gain of the controller.
	*
	* \param gain The gain value to be set.
	* \return 0 on success, 1 if an illegal value is requested.
	*/
	int set_dgain(float gain);

	/*! \brief Set the positive saturation limit for the integral term in the controller.
	*
	* \param limit The limit value to be set.
	* \return 0 on success, 1 if an illegal value is requested.
	*/
	int set_IntegralPosMax(float limit);

	/*! \brief Set the negative saturation limit for the integral term in the controller.
	*
	* \param limit The limit value to be set.
	* \return 0 on success, 1 if an illegal value is requested.
	*/
	int set_IntegralNegMax(float limit);

	/*! \brief Set all the parameters in the controller.
	*
	* All of the control parameters can be set with a single function call.
	* \param[in] parameters A structure that contains all the parameter data.
	* \return 0 on success.
	*/
	int setParams(control_Parameters_t* parameters);

	/*! \brief Get all the parameters in the controller.
	*
	* All of the control parameters can be returned with a single function call.
	* \param[out] parameters structure that will contain all the parameter data.
	* \return 0 on success.
	*/
	int getParams(control_Parameters_t* parameters);

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

	void setisEnabled(int enable);

private:
	float32 integral;/*!<Integral of the error*/
	float32 pgain;/*!<Gain of the proportional component*/
	float32 igain;/*!<Gain of the integral component*/
	float32 dgain;/*!<Gain of the differential component*/
	float32 forwardGain; /*!<Gain of  the feed-forward term*/
	float32 deadBand;/*!<Optional deadband threshold for the control response*/
	float32 lastError;/*!<Error in previous time-step, used to calculate the differential component*/
	float32 IntegralPosMax;/*<!Positive saturation limit for the integral term*/
	float32 IntegralNegMax;/*<!Negative saturation limit for the integral term*/

	enum pvDataType_t{
			INT16,
			INT32
		};

	int32_t outputPosMax; /*!< Maximum control output in the positive direction.*/
	int32_t outputNegMax; /*!< Maximum control output in the negative direction.*/
	float32 spPosMax;/*<!Positive limit for the set point.*/
	float32 spNegMax;/*<!Negative limit for the set point.*/
	int isEnabled;/*<!Whether or not the controller is enabled. 0 if disabled, 1 if enabled*/
	//uint32 timePeriod;/*<!Update period of the controller*/
	float32 timePeriod;/*<!Update period of the controller in seconds*/
	uint32 tag; /*!< The control mode of the current controller, used for error checking.*/
	float32 radPerEncoderCount; /*!output shaft rotation (in rad) per encoder count */
	float32 polyPar[4]; /*! polynomial fit from displacement (d)  to tendon force (f) f=polyPar[0]+polyPar[1]*d +polyPar[2]*d^2+ +polyPar[3]*d^3+ +polyPar[4]*d^4 */
	float32 torqueConstant; /*!motor torque constant in Nm/A */

};
