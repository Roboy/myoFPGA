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
	output wire [9:0] ss_n_o,
	output reg [0:15] Word,
	output reg wren,
	output reg spi_done
);

reg [7:0] numberOfWordsTransmitted;
reg write_ack_prev;
reg next_value;
reg start_frame;

reg unsigned [0:15] startOfFrame;
reg signed [0:15] pwmRef;
reg unsigned[0:15] controlFlags1;
reg unsigned[0:15] controlFlags2;
reg unsigned[0:15] dummy;
reg signed[0:31] actualPosition;
reg signed[0:15] actualVelocity;
reg signed[0:15] actualCurrent;
reg signed[0:15] springDisplacement;
reg signed[0:15] sensor1;
reg signed[0:15] sensor2;
reg [5:0] delay_counter;
reg [2:0]pid_mux;

always @(posedge clock, negedge reset_n) begin: SPICONTROL_SPILOGIC
	if (reset_n == 0) begin
		numberOfWordsTransmitted <= 12;
		wren <= 0;
		write_ack_prev <= 0;
		start_frame <= 0;
		
		startOfFrame <= 16'h8000;
		pwmRef <= 500;
		controlFlags1 <= 0;
		controlFlags2 <= 0;
		dummy <= 0;
		delay_counter <= 0;
		pid_mux <= 0;
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
				1: Word <= pwmRef;
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
			
		
		if( data_read_valid && numberOfWordsTransmitted>=5 && numberOfWordsTransmitted< 12 ) begin
			case(numberOfWordsTransmitted)
				5: actualPosition[0:15] <= data_read;
				6: actualPosition[16:31] <= data_read;
				7: actualVelocity <= data_read;
				8: actualCurrent <= data_read;
				9: springDisplacement <= data_read;
				10: sensor1 <= data_read;
				11: sensor2 <= data_read;
			endcase
		end
		
		if ( numberOfWordsTransmitted>=12 && ss_n==1 ) begin
			spi_done <= 1;
			if (start==1) begin
				spi_done <= 0;
				numberOfWordsTransmitted<= 0;
				start_frame <= 1;
				next_value <= 1;
				pid_mux <= pid_mux + 1;
			end
		end 
	end
end

assign ss_n_o[0] = (pid_mux==0?ss_n:1);
assign ss_n_o[1] = (pid_mux==1?ss_n:1);
assign ss_n_o[2] = (pid_mux==2?ss_n:1);
assign ss_n_o[3] = (pid_mux==3?ss_n:1);
assign ss_n_o[4] = (pid_mux==4?ss_n:1);
assign ss_n_o[5] = (pid_mux==5?ss_n:1);
assign ss_n_o[6] = (pid_mux==6?ss_n:1);
assign ss_n_o[7] = (pid_mux==7?ss_n:1);

endmodule
