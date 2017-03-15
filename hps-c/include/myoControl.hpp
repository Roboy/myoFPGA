#pragma once

#include <vector>
#include <map>
#include <iostream>
#include <math.h>
#include <chrono>
#include <fstream>
#include "CommunicationData.h"
#include "myoSPI.hpp"
#include "pid.hpp"

using namespace std;
using namespace std::chrono;

//#define DEBUG

class MyoControl{
public:
	MyoControl(vector<int32_t*> &pid_base, uint motors = 1);
	~MyoControl();
	/**
	 * updates all motors
	 */
	void update();
	/**
	 * Changes the controller of a motor
	 * @param motor for this motor
	 * @param mode choose from Position, Velocity or Force
	 * @param params with these controller parameters
	 */
	void changeControl(int motor, int mode, control_Parameters_t &params);
	/**
	 * Changes the controller of a motor with the saved controller parameters
	 * @param motor for this motor
	 * @param mode choose from Position, Velocity or Force
	 */
	void changeControl(int motor, int mode);
	/**
	 * Changes setpoint for position controller
	 * @param motor for this motor
	 * @param position the new setpoint
	 */
	void setPosition(int motor, float position);
	/**
	 * Changes setpoint for velocity controller
	 * @param motor for this motor
	 * @param velocity the new setpoint
	 */
	void setVelocity(int motor, float velocity);
	/**
	 * Changes setpoint for force controller
	 * @param motor for this motor
	 * @param force the new setpoint
	 */
	void setForce(int motor, float force);
	/**
	 * Gets the current position of a motor in radians
	 * @param motor for this motor
	 */
	float getPosition(int motor);
	/**
	 * Gets the current velocity of a motor in radians/seconds
	 * @param motor for this motor
	 */
	float getVelocity(int motor);
	/**
	 * Gets the current force of a motor in Newton
	 * @param motor for this motor
	 */
	float getForce(int motor);
	/**
	 * Gets the displacement in encoder ticks
	 * @param motor for this motor
	 */
	float getDisplacement(int motor);
	/**
	 * Gets the current in Ampere
	 * @param motor for this motor
	 */
	float getCurrent(int motor);

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
	void allToPosition(float pos);
	/**
	 * Changes the control mode for all motors to Velocity
	 * @param pos new setPoint
	 */
	void allToVelocity(float vel);
	/**
	 * Changes the control mode for all motors to Force
	 * @param force new setPoint
	 */
	void allToForce(float force);
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

	SPISTREAM frame;
	vector<int32_t> pos;
	vector<int16_t> vel, force, displacement, current;
	vector<int32_t> pos_setPoint, vel_setPoint, force_setPoint;
	vector<int32_t> control_mode;
	map<int,vector<control_Parameters_t>> control_params;
	vector<int16_t> pwm_control;
	vector<vector<float>> polyPar;
	uint32_t *adc_base = nullptr;
	float weight_offset = 0;
	float adc_weight_parameters[2] = {830.7, -0.455};
private:
	vector<int32_t*> pid_base;
	uint numberOfMotors;
	float radPerEncoderCount = 2 * 3.14159265359 / (2000.0 * 53.0);
	int iter = 0;
};
