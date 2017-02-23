#include "myoControl.hpp"

MyoControl::MyoControl(uint32_t* spi_base, uint motors):
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
		force[motor] = polyPar[motor][0]+polyPar[motor][1]*displacement[motor] +
				polyPar[motor][2]*powf(displacement[motor],2.0f)+
				polyPar[motor][3]*powf(displacement[motor],3.0f);
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
	params->outputPosMax = 1000;  // sint32
	params->outputNegMax = -1000; // sint32
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
	params->params.pidParameters.deadBand = 0;             // float32
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
	params->params.pidParameters.deadBand = 0;             // float32
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

void MyoControl::allToPosition(float pos){
	for(uint motor=0; motor<numberOfMotors; motor++){
		control_mode[motor] = Position;
		pos_setPoint[motor] = pos;
	}
}

void MyoControl::allToVelocity(float vel){
	for(uint motor=0; motor<numberOfMotors; motor++){
		control_mode[motor] = Velocity;
		vel_setPoint[motor] = vel;
	}
}

void MyoControl::allToForce(float force){
	for(uint motor=0; motor<numberOfMotors; motor++){
		control_mode[motor] = Force;
		force_setPoint[motor] = force;
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
	setForce(motor,0);
	changeControl(motor,Force);
	update();
	int sample = 0;
	float force_min = 0, force_max = 4.0;
	milliseconds ms_start = duration_cast< milliseconds >(system_clock::now().time_since_epoch()), ms_stop, t0, t1;
	ofstream outfile;
	outfile.open ("springParameters_calibration.csv");
	do{
		float f = (rand()/(float)RAND_MAX)*(force_max-force_min)+force_min;
		setForce(motor, f);
		t0 = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
		do{// wait a bit until force is applied
			// update control
			update();
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
