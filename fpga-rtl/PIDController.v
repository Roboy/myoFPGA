// PID controller myoRobotics style
`timescale 1ns/10ps

module PIDController (
	input clock,
	input reset,
	input unsigned [15:0] Kp,
	input unsigned [15:0] Kd,
	input unsigned [15:0] Ki,
	input signed [31:0] sp,
	input unsigned [15:0] forwardGain,
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
	output reg signed [15:0] result
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
	
	if (reset == 1) begin
		pv <= 0;
		integral <= 0;
		lastError <= 0;
		result <= 0;
		err <=0;
		result <= 0;
		update_controller_prev <= 0;
	end else begin
		update_controller_prev <= update_controller;
		if(update_controller_prev==0 && update_controller==1) begin
			case(controller)
				0: err = (sp - position);
				1: err = (sp - velocity);
				2: err = (sp - displacement);
				default: err = 0;
			endcase
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

