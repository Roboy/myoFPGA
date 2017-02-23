#pragma once

#include <vector>
#include <iostream>
#include <math.h>
#include "myoSPI.hpp"
#include "pidController.hpp"

using namespace std;

//#define DEBUG

class MyoControl{
public:
	MyoControl(uint32_t* spi_base = nullptr, uint motors = 1);
	~MyoControl();
	/**
	 * updates all motors
	 */
	void update();
	/**
	 * Changes the controller of a motor
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

	SPISTREAM frame;
	vector<float> pos, vel, force, displacement, current;
	vector<pidController> position_controller, velocity_controller, force_controller;
	vector<float> pos_setPoint, vel_setPoint, force_setPoint;
	vector<int> control_mode;
	float polyPar[4]= {0, 0.023,-0.000032,0};
	uint32_t* spi_base, *adc_base = nullptr;
	float weight_offset = 0;
	float adc_weight_parameters[2] = {83.7, -0.0455};
private:
	vector<int16_t> pwm_control;
	uint numberOfMotors;
	float radPerEncoderCount = 2 * 3.14159265359 / (2000.0 * 53.0);
	int iter = 0;
};
