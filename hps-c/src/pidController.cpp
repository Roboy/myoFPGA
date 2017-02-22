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
	  radPerEncoderCount = 2 * 3.14159265359 / (2000.0 * 53.0);
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
