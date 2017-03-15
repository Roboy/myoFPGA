// PID controller myoRobotics style
// you can read out the registers via avalon bus in the following way:
// #define IORD(base,reg) (*(((volatile uint32_t*)base)+reg))
// #define IOWR(base,reg,data) (*(((volatile uint32_t*)base)+reg)=data)
// where reg corresponds to the address of the avalon slave


`timescale 1ns/10ps

module pid_controller (
	input clock,
	input reset,
	// this is for the avalon interface
	input [3:0] address,
	input write,
	input signed [31:0] writedata,
	input read,
	input signed [0:31] position,
	input signed [0:15] velocity,
	input signed [0:15] displacement,
	input measurement_update,
	input [1:0] controller, // position velocity force
	output signed [31:0] readdata,
	output reg signed [31:0] result_o,
	output waitrequest
);

// here go the gains and shit
reg signed [31:0] Kp;
reg signed [31:0] Kd;
reg signed [31:0] Ki;
reg signed [31:0] sp;
reg signed [31:0] pv;
reg signed [31:0] integral;
reg signed [31:0] end_result;
reg signed [31:0] lastError;
reg signed [31:0] forwardGain;
reg signed [31:0] outputPosMax;
reg signed [31:0] outputNegMax;
reg signed [31:0] IntegralNegMax;
reg signed [31:0] IntegralPosMax;
reg signed [31:0] deadBand;
reg signed [31:0] result;
reg data_ready;
reg controller_update;
reg signed [31:0] actualPosition;
reg signed [31:0] actualVelocity;
reg signed [31:0] actualDisplacement;

assign waitrequest = ~data_ready;

assign readdata = ((address == 0))? result :
	((address == 1))? Kp :
	((address == 2))? Kd :
	((address == 3))? Ki :
	((address == 4))? sp :
	((address == 5))? forwardGain :
	((address == 6))? outputPosMax :
	((address == 7))? outputNegMax :
	((address == 8))? IntegralNegMax :
	((address == 9))? IntegralPosMax :
	((address == 10))? deadBand :
	((address == 11))? actualPosition :
	((address == 12))? actualVelocity :
	((address == 13))? actualDisplacement :
	32'hDEAD_BEEF;

always @(posedge clock, posedge reset) begin: PID_CONTROLLER_PID_CONTROLLERLOGIC
	reg signed [31:0] err;
	reg signed [31:0] pterm;
	reg signed [31:0] dterm;
	reg signed [31:0] ffterm;
	if (reset == 1) begin
		integral <= 0;
		lastError <= 0;
		result <= 0;
		err <=0;
		result <= 0;
		data_ready <= 0;
		Kp <= 1;
		Kd <= 0;
		Ki <= 0;
		sp <= 0;
		forwardGain <= 0;
		outputPosMax <= 2000;
		outputNegMax <= -2000;
		IntegralPosMax <= 100;
		IntegralNegMax <= -100;
		deadBand <= 0;
	end else begin
		if(measurement_update || controller_update) begin
			case(controller)
				0: pv = position;
				1: pv = velocity;
				2: pv = displacement;
				default: pv = 0;
			endcase
			data_ready = 0;
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
				result = (((ffterm + pterm) + integral) + dterm);
				if ((result < outputNegMax)) 
					 result = outputNegMax;
				else if ((result > outputPosMax)) 
					 result = outputPosMax;
			end else 
				result = integral;
			result_o = result;
			lastError = err;
			controller_update = 0;
		end
		data_ready = 1;

		if(measurement_update) begin
			actualPosition[31:0] <= position[0:31];
			actualVelocity[15:0] <= velocity[0:15];
			actualDisplacement[15:0] <= displacement[0:15];
		end

		if(write && ~waitrequest) begin
			case(address)
				1: Kp <= writedata[31:0];
				2: Kd <= writedata[31:0];
				3: Ki <= writedata[31:0];
				4: sp <= writedata[31:0];
				5: forwardGain <= writedata[31:0];
				6: outputPosMax <= writedata[31:0];
				7: outputNegMax <= writedata[31:0];
				8: IntegralNegMax <= writedata[31:0];
				9: IntegralPosMax <= writedata[31:0];
				10: deadBand <= writedata[31:0];
			endcase 
			controller_update <= 1;
		end
    	end 
end


endmodule


