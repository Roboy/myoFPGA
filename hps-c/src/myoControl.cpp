#include "myoControl.hpp"

MyoControl::MyoControl(uint motors, uint32_t* spi_base):
numberOfMotors(motors), spi_base(spi_base){
	// initialize all controllers with default values
	control_Parameters_t params;
	getDefaultControlParams(&params, Position);
	for(uint motor=0;motor<numberOfMotors;motor++){
		pidController controller(&params);
		position_controller.push_back(controller);
	}
	getDefaultControlParams(&params, Velocity);
	for(uint motor=0;motor<numberOfMotors;motor++){
		pidController controller(&params);
		velocity_controller.push_back(controller);
	}
	getDefaultControlParams(&params, Force);
	for(uint motor=0;motor<numberOfMotors;motor++){
		pidController controller(&params);
		force_controller.push_back(controller);
	}
	// initialize setpoints
	pos_setPoint.resize(numberOfMotors,0);
	vel_setPoint.resize(numberOfMotors,0);
	force_setPoint.resize(numberOfMotors,0);
	pwm_control.resize(numberOfMotors,0);
	// initialize control mode to be force
	control_mode.resize(numberOfMotors, Force);
	// initialize the actual values
	pos.resize(numberOfMotors,0);
	vel.resize(numberOfMotors,0);
	force.resize(numberOfMotors,0);
	displacement.resize(numberOfMotors,0);
	current.resize(numberOfMotors,0);
}

MyoControl::~MyoControl(){
	cout << "shutting down myoControl" << endl;
}

void MyoControl::update(){
	for(uint motor=0;motor<numberOfMotors;motor++){
		switch(control_mode[motor]){
		case Position:
			pwm_control[motor] = position_controller[motor].outputCalc(pos[motor],pos_setPoint[motor]);
			break;
		case Velocity:
			pwm_control[motor] = velocity_controller[motor].outputCalc(vel[motor],vel_setPoint[motor]);
			break;
		case Force:
			pwm_control[motor] = force_controller[motor].outputCalc(force[motor],force_setPoint[motor]);
			break;
		default:
			cout << "currently only supporting Position, Velocity or Force control" << endl;
		}
		for(uint i = 0; i<24;i++)
			frame.TxBuffer[i] = 0;
		frame.pwmRef = pwm_control[motor];

		prepareData(&frame, WRITE_DATA);
		exchangeFrame(spi_base, motor, &frame);
		prepareData(&frame, READ_DATA);

		pos[motor] = frame.actualPosition*radPerEncoderCount;
		vel[motor] = frame.actualVelocity*radPerEncoderCount;
		displacement[motor] = frame.springDisplacement;
		force[motor] = polyPar[0]+polyPar[1]*displacement[motor] +
				polyPar[2]*powf(displacement[motor],2.0f)+
				polyPar[3]*powf(displacement[motor],3.0f);
		current[motor] = frame.actualCurrent;

#ifdef DEBUG
		if(iter%1000==0 && (motor>=0 && motor<=7)){
		printf("============motor %d=============\n", motor);
					printf( "startOfFrame:         %d\n"
						  "pwmRef:               %d\n"
						  "controlFlags1:        %d\n"
						  "controlFlags2:        %d\n"
						  "dummy:                %d\n"
						  "actualPosition:       %f\n"
						   "actualVelocity:      %f\n"
						   "actualCurrent:       %d\n"
						   "springDisplacement:  %d\n"
						   "sensor1:             %d\n"
						   "sensor2:             %d\n"
						   "force:               %f\n",
						   frame.startOfFrame, pwm_control[motor], frame.controlFlags1, frame.controlFlags2, frame.dummy,
						   pos[motor], vel[motor], frame.actualCurrent, frame.springDisplacement, frame.sensor1, frame.sensor2, force[motor]);
		}
#endif
	}
	iter++;
}

void MyoControl::changeControl(int motor, int mode){
	control_mode[motor] = mode;
}

void MyoControl::setPosition(int motor, float position){
	pos_setPoint[motor] = position;
}

void MyoControl::setVelocity(int motor, float velocity){
	vel_setPoint[motor] = velocity;
}

void MyoControl::setForce(int motor, float force){
	force_setPoint[motor] = force;
}

float MyoControl::getPosition(int motor){
	return pos[motor];
}

float MyoControl::getVelocity(int motor){
	return vel[motor];
}

float MyoControl::getForce(int motor){
	return force[motor];
}

float MyoControl::getDisplacement(int motor){
	return displacement[motor];
}

float MyoControl::getCurrent(int motor){
	return current[motor];
}

void MyoControl::getDefaultControlParams(control_Parameters_t *params, int control_mode){
	params->tag = 0;              // sint32
	params->outputPosMax = 500;  // sint32
	params->outputNegMax = -500; // sint32
	params->timePeriod = 10; // float32      //in us set time period to avoid error case

	params->radPerEncoderCount =
			2 * 3.14159265359 / (2000.0 * 53.0);          // float32
	params->params.pidParameters.lastError = 0; // float32

	params->spPosMax = 1000; // float32
	params->spNegMax = -1000; // float32
switch(control_mode){
case Position:
	params->params.pidParameters.integral = 0;             // float32
	params->params.pidParameters.pgain = 1000.0;                   // float32
	params->params.pidParameters.igain = 0;                   // float32
	params->params.pidParameters.dgain = 0;                   // float32
	params->params.pidParameters.forwardGain = 0;       // float32
	params->params.pidParameters.deadBand = 2;             // float32
	params->params.pidParameters.IntegralPosMax = 100; // float32
	params->params.pidParameters.IntegralNegMax = -100; // float32
	params->params.pidParameters.lastError = 0;
	break;
case Velocity:
	params->params.pidParameters.integral = 0;             // float32
	params->params.pidParameters.pgain = 1000.0;                   // float32
	params->params.pidParameters.igain = 0;                   // float32
	params->params.pidParameters.dgain = 0;                   // float32
	params->params.pidParameters.forwardGain = 0;       // float32
	params->params.pidParameters.deadBand = 2;             // float32
	params->params.pidParameters.IntegralPosMax = 100; // float32
	params->params.pidParameters.IntegralNegMax = -100; // float32
	params->params.pidParameters.lastError = 0;
	break;
case Force:
	params->params.pidParameters.integral = 0;             // float32
	params->params.pidParameters.pgain = 1000.0;                   // float32
	params->params.pidParameters.igain = 0;                   // float32
	params->params.pidParameters.dgain = 0;                   // float32
	params->params.pidParameters.forwardGain = 0;       // float32
	params->params.pidParameters.deadBand = 2;             // float32
	params->params.pidParameters.IntegralPosMax = 100; // float32
	params->params.pidParameters.IntegralNegMax = -100; // float32
	params->params.pidParameters.lastError = 0;
	break;
default:
	cout << "unknown control mode" << endl;
	break;
}

}
