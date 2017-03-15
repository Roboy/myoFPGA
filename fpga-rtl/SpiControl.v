`timescale 1ns/10ps

module SpiControl (
   input clock,
	input reset_n,
	input di_req,
	input write_ack,
	input data_read_valid,
	input [0:15] data_read,
	input start,
	input wire ss_n,
	input signed [0:15] pwmRef,
	output wire [7:0] ss_n_o,
	output reg [0:15] Word,
	output reg wren,
	output spi_done,
	output reg signed[0:31] position,
	output reg signed[0:15] velocity,
	output reg signed[0:15] current,
	output reg signed[0:15] displacement,
	output reg [7:0] motor_switch
);

reg [7:0] numberOfWordsTransmitted;
reg [7:0] numberOfWordsReceived;
reg write_ack_prev;
reg next_value;
reg start_frame;
reg data_read_valid_prev;

reg unsigned [0:15] startOfFrame;

reg unsigned[0:15] controlFlags1;
reg unsigned[0:15] controlFlags2;
reg unsigned[0:15] dummy;
reg signed[0:15] actualCurrent;
reg signed[0:15] sensor1;
reg signed[0:15] sensor2;
reg [5:0] delay_counter;

//`define ENABLE_DELAY

assign spi_done = numberOfWordsTransmitted>=12;

always @(posedge clock, negedge reset_n) begin: SPICONTROL_SPILOGIC
	if (reset_n == 0) begin
		numberOfWordsTransmitted <= 12;
		wren <= 0;
		write_ack_prev <= 0;
		start_frame <= 0;
		
		startOfFrame <= 16'h8000;
		controlFlags1 <= 0;
		controlFlags2 <= 0;
		dummy <= 0;
		delay_counter <= 0;
		motor_switch <= 8'b00000001;
	end else begin
		write_ack_prev <= write_ack;
		if( write_ack_prev==0 && write_ack == 1) begin
			wren <= 0;
			numberOfWordsTransmitted <= numberOfWordsTransmitted + 1;
			next_value <= 1;
		end
		
		if( (di_req || start_frame) && numberOfWordsTransmitted<12 && next_value==1) begin
			case(numberOfWordsTransmitted)
				0: Word <= startOfFrame;
				1: Word <= pwmRef & 16'h7fff;
				2: Word <= controlFlags1;
				3: Word <= controlFlags2;
				4: Word <= dummy;
				default: Word <= 0;
			endcase
`ifdef ENABLE_DELAY
			delay_counter <= 1;
`else
			wren <= 1;
`endif
			next_value <= 0;
			
			if(start_frame)
				start_frame <= 0;
		end
		
`ifdef ENABLE_DELAY
		if(wren==0 && next_value==0) begin // this adds a delay of 64/50 approx. 1.28us
			if(delay_counter==0)
				wren <= 1;
			else if (delay_counter>0)
				delay_counter <= delay_counter + 1;
		end
`endif /*ENABLE_DELAY*/
			
		data_read_valid_prev <= data_read_valid;
		if( data_read_valid_prev==1 && data_read_valid==0 ) begin
			case(numberOfWordsReceived)
				5: position[0:15] <= data_read;
				6: position[16:31] <= data_read;
				7: velocity <= data_read;
				8: current <= data_read;
				9: displacement <= data_read;
				10: sensor1 <= data_read;
				11: sensor2 <= data_read;
			endcase
			numberOfWordsReceived <= numberOfWordsReceived + 1;
		end
		
		if ( numberOfWordsTransmitted>=12 && ss_n==1 ) begin			
			if ( start ) begin
				numberOfWordsTransmitted<= 0;
				numberOfWordsReceived <= 0;
				start_frame <= 1;
				next_value <= 1;
				if(motor_switch[0])
					motor_switch <= 8'b00000000;
				else
					motor_switch <= 8'b00000001;
			end
		end 
	end
end

assign ss_n_o[0] = (motor_switch==1?ss_n:1);
assign ss_n_o[1] = (motor_switch==2?ss_n:1);
assign ss_n_o[2] = (motor_switch==4?ss_n:1);
assign ss_n_o[3] = (motor_switch==8?ss_n:1);
assign ss_n_o[4] = (motor_switch==16?ss_n:1);
assign ss_n_o[5] = (motor_switch==32?ss_n:1);
assign ss_n_o[6] = (motor_switch==64?ss_n:1);
assign ss_n_o[7] = (motor_switch==128?ss_n:1);

endmodule
