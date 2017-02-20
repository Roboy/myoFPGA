#include <pidController.hpp>

pidController::pidController()
{

}

pidController::pidController(control_Parameters_t *parameters)
{
	  tag = parameters->tag;
	  pgain = parameters->params.pidParameters.pgain;
	  igain = parameters->params.pidParameters.igain;
	  dgain = parameters->params.pidParameters.dgain;
	  deadBand = parameters->params.pidParameters.deadBand;
	  timePeriod = parameters->timePeriod;
	  lastError = 0;
	  integral = 0;
	  IntegralPosMax = parameters->params.pidParameters.IntegralPosMax;
	  IntegralNegMax = parameters->params.pidParameters.IntegralNegMax;
	  outputPosMax = parameters->outputPosMax;
	  outputNegMax = parameters->outputNegMax;
	  spPosMax = parameters->spPosMax;
	  spNegMax = parameters->spNegMax;
	  isEnabled = 0;//controller is initialised disabled
}

int pidController::set_pgain(float gain)
{
  pgain = gain;

  return 0;
}

int pidController::set_igain(float gain)
{
  igain = gain;

  return 0;
}

int pidController::set_dgain(float gain)
{
  dgain = gain;

  return 0;
}

void pidController::pid_setinteg(float new_integ)
{
	integral = new_integ;
	lastError = 0;
}

int pidController::set_IntegralPosMax(float max)
{
	if(max<=outputPosMax)
		IntegralPosMax = max;
	else
		return 1;//return an error if the value is illegal

	return 0;
}

int pidController::set_IntegralNegMax(float max)
{
	if(max<=outputNegMax)
		IntegralNegMax = max;
	else
		return 1;//return an error if the value is illegal

	return 0;
}

int pidController::setParams(control_Parameters_t *parameters)
{
	  pgain = parameters->params.pidParameters.pgain;
	  igain = parameters->params.pidParameters.igain;
	  dgain = parameters->params.pidParameters.dgain;
	  forwardGain = parameters->params.pidParameters.forwardGain;
	  deadBand = parameters->params.pidParameters.deadBand;
	  timePeriod = parameters->timePeriod/1000000.0;//convert the time from microseconds to seconds
	  lastError = 0;
	  IntegralPosMax = parameters->params.pidParameters.IntegralPosMax;
	  IntegralNegMax = parameters->params.pidParameters.IntegralNegMax;
	  outputPosMax = parameters->outputPosMax;
	  outputNegMax = parameters->outputNegMax;
	  spPosMax = parameters->spPosMax;
	  spNegMax = parameters->spNegMax;
	  radPerEncoderCount = parameters->radPerEncoderCount;
	  std::copy(parameters->polyPar, parameters->polyPar + 4, polyPar);
	  torqueConstant = parameters->torqueConstant;
	  //isEnabled = 1;

	  return 0;
}

int pidController::getParams(control_Parameters_t* parameters)
{
	parameters->tag = tag;
	parameters->params.pidParameters.pgain = pgain;
	parameters->params.pidParameters.igain = igain;
	parameters->params.pidParameters.dgain = dgain;
	parameters->params.pidParameters.forwardGain = forwardGain;
    parameters->params.pidParameters.deadBand = deadBand;
    parameters->timePeriod = timePeriod;
    parameters->params.pidParameters.integral = integral;
    parameters->outputPosMax = outputPosMax;
    parameters->outputNegMax = outputNegMax;
    parameters->params.pidParameters.IntegralPosMax = IntegralPosMax;
    parameters->params.pidParameters.IntegralNegMax = IntegralNegMax;
    parameters->spPosMax = spPosMax;
    parameters->spNegMax = spNegMax;
    parameters->radPerEncoderCount = radPerEncoderCount;
    std::copy(polyPar, polyPar + 4, parameters->polyPar);
    parameters->torqueConstant = torqueConstant;

	return 0;
}

int16_t pidController::outputCalc(float pv, float sp)
{
	float pterm, dterm, result, err, ffterm;


	err = sp - pv;
	if ((err > deadBand) || (err < -1*deadBand))
	{
		pterm = pgain * err;
		if ((pterm < outputPosMax) || (pterm > outputNegMax)) //if the proportional term is not maxed
		{
			integral += (igain * err * timePeriod); //add to the integral
			if (integral > IntegralPosMax)
				integral = IntegralPosMax;
			else if (integral < IntegralNegMax)
				integral = IntegralNegMax;
		}

		dterm = ((err - lastError)/timePeriod) * dgain;

		ffterm = forwardGain * sp;
		result = ffterm + pterm + integral + dterm;
		if(result<outputNegMax)
			result = (float)outputNegMax;
		else if(result>outputPosMax)
			result=(float)outputPosMax;
	}
	else
		result = integral;

	lastError = err;

	 //check for control limits
	//controllerGetParams(motorID, &control_parameters);//load controller parameters...MJP: why?
	if(result>4000)//check limit using raw parameters as all controllers will contain the elements specified
		result = 4000;
	else if(result<-4000)
		result = -4000;

	return (int16_t)result;

}

void pidController::setisEnabled(int enable)
{
	//spiREG5->PCDOUT = spiREG5->PCDOUT^LED6;
	if((this->isEnabled == controllerDISABLE) && (enable == controllerENABLE))
		this->integral = 0;//set the integral to zero when a controller is enabled/disabled
	this->isEnabled = enable;
}
