#include "myoControl.hpp"

MyoControl::MyoControl(vector<int32_t*> &myo_base):myo_base(myo_base){
	// initialize control mode
	numberOfMotors = myo_base.size()*MOTORS_PER_MYOCONTROL;
	// initialize all controllers with default values
	control_Parameters_t params;
	getDefaultControlParams(&params, Position);
	for(uint motor=0;motor<numberOfMotors;motor++){
		changeControl(motor, 0, params);
		control_params[0].push_back(params);
	}
	getDefaultControlParams(&params, Velocity);
	for(uint motor=0;motor<numberOfMotors;motor++){
		control_params[1].push_back(params);
	}
	getDefaultControlParams(&params, Force);
	for(uint motor=0;motor<numberOfMotors;motor++){
		control_params[2].push_back(params);
	}
	pwm_control.resize(numberOfMotors,0);
	// set the default spring parameters
	polyPar.resize(numberOfMotors);
	vector<float> coeffs(4);
	coeffs[0] = 0;
	coeffs[1] = 0.023;
	coeffs[2] = -0.000032;
	coeffs[3] = 0;
	for(uint motor=0; motor<numberOfMotors; motor++){
		polyPar[motor] = coeffs;
	}
}

MyoControl::~MyoControl(){
	cout << "shutting down myoControl" << endl;
}

void MyoControl::changeControl(int motor, int mode, control_Parameters_t &params){
	for(uint motor=0;motor<numberOfMotors;motor++){
		// set the current setpoint to the current measurement, which results in zero error
		int32_t current_measurement = MYO_READ_sp(myo_base[motor/MOTORS_PER_MYOCONTROL],motor);
		MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, current_measurement);
		MYO_WRITE_Kp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (uint16_t)params.params.pidParameters.pgain);
		MYO_WRITE_Kd(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (uint16_t)params.params.pidParameters.dgain);
		MYO_WRITE_Ki(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (uint16_t)params.params.pidParameters.igain);
		MYO_WRITE_forwardGain(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (int16_t)params.params.pidParameters.forwardGain);
		MYO_WRITE_deadBand(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (uint16_t)params.params.pidParameters.deadBand);
		MYO_WRITE_IntegralPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (int16_t)params.params.pidParameters.IntegralPosMax);
		MYO_WRITE_IntegralNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (int16_t)params.params.pidParameters.IntegralNegMax);
		MYO_WRITE_outputPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (int16_t)params.outputPosMax);
		MYO_WRITE_outputNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (int16_t)params.outputNegMax);
		MYO_WRITE_control(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, mode);
	}
}

void MyoControl::changeControl(int motor, int mode){
	for(uint motor=0;motor<numberOfMotors;motor++){
		// set the current setpoint to the current measurement, which results in zero error
		int current_measurement = MYO_READ_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor);
		MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, current_measurement);
		MYO_WRITE_Kp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (uint16_t)control_params[mode][motor].params.pidParameters.pgain);
		MYO_WRITE_Kd(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (uint16_t)control_params[mode][motor].params.pidParameters.dgain);
		MYO_WRITE_Ki(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (uint16_t)control_params[mode][motor].params.pidParameters.igain);
		MYO_WRITE_forwardGain(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (int16_t)control_params[mode][motor].params.pidParameters.forwardGain);
		MYO_WRITE_deadBand(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (uint16_t)(control_params[mode][motor].params.pidParameters.deadBand));
		MYO_WRITE_IntegralPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (int16_t)control_params[mode][motor].params.pidParameters.IntegralPosMax);
		MYO_WRITE_IntegralNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (int16_t)control_params[mode][motor].params.pidParameters.IntegralNegMax);
		MYO_WRITE_outputPosMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (int16_t)control_params[mode][motor].outputPosMax);
		MYO_WRITE_outputNegMax(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, (int16_t)control_params[mode][motor].outputNegMax);
		MYO_WRITE_control(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, mode);
	}
}

bool MyoControl::toggleSPI(){
	spi_active = !spi_active;
	for(uint i=0;i<myo_base.size();i++)
		MYO_WRITE_spi_activated(myo_base[i],spi_active);
	return spi_active;
}

void MyoControl::reset(){
	for(uint i=0;i<myo_base.size();i++){
		MYO_WRITE_reset_myo_control(myo_base[i],true);
		MYO_WRITE_reset_myo_control(myo_base[i],false);
	}
}

void MyoControl::setPosition(int motor, int32_t position){
	MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, position);
}

void MyoControl::setVelocity(int motor, int32_t velocity){
	MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, velocity);
}

void MyoControl::setDisplacement(int motor, int32_t displacement){
	MYO_WRITE_sp(myo_base[motor/MOTORS_PER_MYOCONTROL], motor, displacement);
}

void MyoControl::getPIDcontrollerParams(int &Pgain, int &Igain, int &Dgain, int &forwardGain, int &deadband,
									int &setPoint, int &setPointMin, int &setPointMax, int motor){
	Pgain = MYO_READ_Kp(myo_base[motor/MOTORS_PER_MYOCONTROL],motor);
	Igain = MYO_READ_Ki(myo_base[motor/MOTORS_PER_MYOCONTROL],motor);
	Dgain = MYO_READ_Kd(myo_base[motor/MOTORS_PER_MYOCONTROL],motor);
	forwardGain = MYO_READ_forwardGain(myo_base[motor/MOTORS_PER_MYOCONTROL],motor);
	deadband = MYO_READ_deadBand(myo_base[motor/MOTORS_PER_MYOCONTROL],motor);
	setPoint = MYO_READ_sp(myo_base[motor/MOTORS_PER_MYOCONTROL],motor);
	setPointMin = 0;
	setPointMax = 0;
}

int16_t MyoControl::getPWM(int motor){
	return MYO_READ_pwmRef(myo_base[motor/MOTORS_PER_MYOCONTROL],motor);
}

int32_t MyoControl::getPosition(int motor){
	return MYO_READ_position(myo_base[motor/MOTORS_PER_MYOCONTROL],motor);
}

int16_t MyoControl::getVelocity(int motor){
	return MYO_READ_velocity(myo_base[motor/MOTORS_PER_MYOCONTROL],motor);
}

int16_t MyoControl::getDisplacement(int motor){
	return MYO_READ_displacement(myo_base[motor/MOTORS_PER_MYOCONTROL],motor);
}

int16_t MyoControl::getCurrent(int motor){
	return MYO_READ_current(myo_base[motor/MOTORS_PER_MYOCONTROL],motor);
}

void MyoControl::getDefaultControlParams(control_Parameters_t *params, int control_mode){
	params->tag = 0;              // sint32
	params->outputPosMax = 1000;  // sint32
	params->outputNegMax = -1000; // sint32
	params->timePeriod = 10; // float32

	params->radPerEncoderCount =
			2 * 3.14159265359 / (2000.0 * 53.0);          // float32
	params->params.pidParameters.lastError = 0; // float32

	params->spPosMax = 10000000; // float32
	params->spNegMax = -10000000; // float32
switch(control_mode){
case Position:
	params->params.pidParameters.integral = 0;             // float32
	params->params.pidParameters.pgain = 1.0;                   // float32
	params->params.pidParameters.igain = 0;                   // float32
	params->params.pidParameters.dgain = 0;                   // float32
	params->params.pidParameters.forwardGain = 0;       // float32
	params->params.pidParameters.deadBand = 0;             // float32
	params->params.pidParameters.IntegralPosMax = 100; // float32
	params->params.pidParameters.IntegralNegMax = -100; // float32
	params->params.pidParameters.lastError = 0;
	break;
case Velocity:
	params->params.pidParameters.integral = 0;             // float32
	params->params.pidParameters.pgain = 1.0;                   // float32
	params->params.pidParameters.igain = 0;                   // float32
	params->params.pidParameters.dgain = 0;                   // float32
	params->params.pidParameters.forwardGain = 0;       // float32
	params->params.pidParameters.deadBand = 0;             // float32
	params->params.pidParameters.IntegralPosMax = 100; // float32
	params->params.pidParameters.IntegralNegMax = -100; // float32
	params->params.pidParameters.lastError = 0;
	break;
case Force:
	params->params.pidParameters.integral = 0;             // float32
	params->params.pidParameters.pgain = 1.0;                   // float32
	params->params.pidParameters.igain = 0;                   // float32
	params->params.pidParameters.dgain = 0;                   // float32
	params->params.pidParameters.forwardGain = 0;       // float32
	params->params.pidParameters.deadBand = 0;             // float32
	params->params.pidParameters.IntegralPosMax = 100; // float32
	params->params.pidParameters.IntegralNegMax = -100; // float32
	params->params.pidParameters.lastError = 0;
	break;
default:
	cout << "unknown control mode" << endl;
	break;
}

}

void MyoControl::allToPosition(int32_t pos){
	for(uint motor=0; motor<numberOfMotors; motor++){

	}
}

void MyoControl::allToVelocity(int32_t vel){
	for(uint motor=0; motor<numberOfMotors; motor++){

	}
}

void MyoControl::allToDisplacement(int32_t displacement){
	for(uint motor=0; motor<numberOfMotors; motor++){

	}
}

void MyoControl::zeroWeight(){
	weight_offset = -getWeight();
}

float MyoControl::getWeight(){
	float weight = 0;
	uint32_t adc_value = 0;
	if(adc_base!=nullptr){
		*adc_base = 0;
		adc_value = *adc_base;
		weight = (adc_weight_parameters[0]+weight_offset+adc_weight_parameters[1]*adc_value);
	}
	return weight;
}

void MyoControl::estimateSpringParameters(int motor, int timeout,  uint numberOfDataPoints){
	vector<float> weight, displacement, coeffs;
	setDisplacement(motor,0);
	changeControl(motor,Force);
	float force_min = 0, force_max = 4.0;
	milliseconds ms_start = duration_cast< milliseconds >(system_clock::now().time_since_epoch()), ms_stop, t0, t1;
	ofstream outfile;
	outfile.open ("springParameters_calibration.csv");
	do{
		float f = (rand()/(float)RAND_MAX)*(force_max-force_min)+force_min;
		setDisplacement(motor, f);
		t0 = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
		do{// wait a bit until force is applied
			// update control
			t1 = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
		}while((t1-t0).count()<500);

		// note the weight
		weight.push_back(getWeight());
		// note the force
		displacement.push_back(getDisplacement(motor));
		outfile << displacement.back() << ", " << weight.back() << endl;
		ms_stop = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
	}while((ms_stop-ms_start).count()<timeout && weight.size()<numberOfDataPoints);
	polynomialRegression(3, displacement, weight, coeffs);
	polyPar[motor] = coeffs;
	outfile.close();
}

void MyoControl::polynomialRegression(int degree, vector<float> &x, vector<float> &y,
			vector<float> &coeffs){
		int N = x.size(), i, j, k;
	    double X[2*degree+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	    for (i=0;i<2*degree+1;i++)
	    {
	        X[i]=0;
	        for (j=0;j<N;j++)
	            X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	    }
	    double B[degree+1][degree+2],a[degree+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
	    for (i=0;i<=degree;i++)
	        for (j=0;j<=degree;j++)
	            B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
	    double Y[degree+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^degree*yi)
	    for (i=0;i<degree+1;i++)
	    {
	        Y[i]=0;
	        for (j=0;j<N;j++)
	        Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	    }
	    for (i=0;i<=degree;i++)
	        B[i][degree+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
	    degree=degree+1;                //degree is made degree+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
	    for (i=0;i<degree;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
	        for (k=i+1;k<degree;k++)
	            if (B[i][i]<B[k][i])
	                for (j=0;j<=degree;j++)
	                {
	                    double temp=B[i][j];
	                    B[i][j]=B[k][j];
	                    B[k][j]=temp;
	                }

	    for (i=0;i<degree-1;i++)            //loop to perform the gauss elimination
	        for (k=i+1;k<degree;k++)
	            {
	                double t=B[k][i]/B[i][i];
	                for (j=0;j<=degree;j++)
	                    B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
	            }
	    for (i=degree-1;i>=0;i--)                //back-substitution
	    {                        //x is an array whose values correspond to the values of x,y,z..
	        a[i]=B[i][degree];                //make the variable to be calculated equal to the rhs of the last equation
	        for (j=0;j<degree;j++)
	            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
	                a[i]=a[i]-B[i][j]*a[j];
	        a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
	    }
	    for (i=0;i<degree;i++)
	        coeffs.push_back(a[i]);	//the values of x^0,x^1,x^2,x^3,....
}
