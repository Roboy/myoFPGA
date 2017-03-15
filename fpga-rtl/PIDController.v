// PID controller myoRobotics style
`timescale 1ns/10ps

module PIDController (
	input clock,
	input reset,
	input signed [31:0] Kp,
	input signed [31:0] Kd,
	input signed [31:0] Ki,
	input signed [31:0] sp,
	input signed [31:0] forwardGain,
	input signed [31:0] outputPosMax,
	input signed [31:0] outputNegMax,
	input signed [31:0] IntegralNegMax,
	input signed [31:0] IntegralPosMax,
	input signed [31:0] deadBand,
	input [1:0] controller, // position velocity force
	input signed [31:0] position,
	input signed [31:0] velocity,
	input signed [31:0] displacement,
	input controller_update,
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
	reg signed [31:0] tmp_result;
	reg controller_update_prev;
	
	if (reset == 1) begin
		pv <= 0;
		integral <= 0;
		lastError <= 0;
		result <= 0;
		err <=0;
		result <= 0;
		tmp_result <= 0;
		controller_update_prev <= 0;
	end else begin
		controller_update_prev <= controller_update;
		if(controller_update_prev==0 && controller_update) begin
			case(controller)
				0: pv = position;
				1: pv = velocity;
				2: pv = displacement;
				default: pv = 0;
			endcase
			
			err = (sp - pv);
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
				tmp_result = (((ffterm + pterm) + integral) + dterm);
				if ((tmp_result < outputNegMax)) 
					 tmp_result = outputNegMax;
				else if ((tmp_result > outputPosMax)) 
					 tmp_result = outputPosMax;
			end else 
				tmp_result = integral;
			lastError = err;
			result = tmp_result;
		end
	end 
end


endmodule

