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
	output signed [31:0] readdata,
	output signed [31:0] o_output,
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
reg process;

assign waitrequest = ~data_ready;

assign readdata = ((address == 0))? result :
	((address == 1))? Kp :
	((address == 2))? Kd :
	((address == 3))? Ki :
	((address == 4))? sp :
	((address == 5))? pv :
	((address == 6))? forwardGain :
	((address == 7))? outputPosMax :
	((address == 8))? outputNegMax :
	((address == 9))? IntegralNegMax :
	((address == 10))? IntegralPosMax :
	((address == 11))? deadBand :
	32'hDEAD_BEEF;

always @(posedge clock, posedge reset) begin: PID_CONTROLLER_PID_CONTROLLERLOGIC
	// local variables, scope limited to this process
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
		process <= 1;
		Kp <= 1;
		Kd <= 0;
		Ki <= 0;
		sp <= 0;
		pv <= 0;
		forwardGain <= 0;
		outputPosMax <= 4000;
		outputNegMax <= -4000;
		IntegralPosMax <= 100;
		IntegralNegMax <= -100;
		deadBand <= 0;
	end else begin
		if(process) begin
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
			lastError = err;
			process = 0;
		end
		data_ready = 1;

		if(write && ~waitrequest) begin
			case(address)
				1: Kp <= writedata[31:0];
				2: Kd <= writedata[31:0];
				3: Ki <= writedata[31:0];
				4: sp <= writedata[31:0];
				5: pv <= writedata[31:0];
				6: forwardGain <= writedata[31:0];
				7: outputPosMax <= writedata[31:0];
				8: outputNegMax <= writedata[31:0];
				9: IntegralNegMax <= writedata[31:0];
				10: IntegralPosMax <= writedata[31:0];
				11: deadBand <= writedata[31:0];
			endcase 
			process <= 1;
		end
    	end 
end


endmodule




