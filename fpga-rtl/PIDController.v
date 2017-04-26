// PID controller myoRobotics style
`timescale 1ns/10ps

module PIDController (
	input clock,
	input reset,
	input unsigned [15:0] Kp,
	input unsigned [15:0] Kd,
	input unsigned [15:0] Ki,
	input signed [31:0] sp,
	input signed [15:0] forwardGain,
	input signed [15:0] outputPosMax,
	input signed [15:0] outputNegMax,
	input signed [15:0] IntegralNegMax,
	input signed [15:0] IntegralPosMax,
	input unsigned [15:0] deadBand,
	input unsigned [1:0] controller, // position velocity displacement
	input signed [31:0] position,
	input signed [15:0] velocity,
	input signed [15:0] displacement,
	input update_controller,
	output reg signed [31:0] result
	);

always @(posedge clock, posedge reset) begin: PID_CONTROLLER_PID_CONTROLLERLOGIC
	reg signed [31:0] pv;
	reg signed [31:0] integral;
	reg signed [31:0] lastError;
	reg signed [31:0] err;
	reg signed [31:0] pterm;
	reg signed [31:0] dterm;
	reg signed [31:0] ffterm;
	reg update_controller_prev;
	reg signed [15:0] displacement_offset;
	
	if (reset == 1) begin
		pv <= 0;
		integral <= 0;
		lastError <= 0;
		result <= 0;
		err <=0;
		result <= 0;
		update_controller_prev <= 0;
		displacement_offset <= 0;
	end else begin
		update_controller_prev <= update_controller;
		if(update_controller_prev==0 && update_controller==1) begin
			if(controller==0) 
				err = (sp - position); 
			else if(controller==1) 
				err = (sp - velocity);
			else if(controller==2) begin
				if(displacement<0) // this should not happen, unless the muscle was in tension when power was turned on
					err = 0;
				else 
					err = (sp - displacement);
			end else
				err = 0;
			
			if (((err > deadBand) || (err < ((-1) * deadBand)))) begin
				pterm = (Kp * err);
				if ((pterm < outputPosMax) || (pterm > outputNegMax)) begin  //if the proportional term is not maxed
					integral = integral + (Ki * err); //add to the integral
					if (integral > IntegralPosMax) 
						integral = IntegralPosMax;
					else if (integral < IntegralNegMax) 
						integral = IntegralNegMax;
				end
				dterm = ((err - lastError) * Kd);
				ffterm = (forwardGain * sp);
				result = (((ffterm + pterm) + integral) + dterm);
				if ((result < outputNegMax)) 
					 result = outputNegMax;
				else if ((result > outputPosMax)) 
					 result = outputPosMax;
			end else 
				result = integral;
			lastError = err;
		end
	end 
end


endmodule

