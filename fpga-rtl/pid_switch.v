`timescale 1ns/10ps

module pid_switch (
   input clock,
	input reset_n,
	input spi_done,
	output reg [3:0]pid_mux
);

reg spi_done_prev;

always @(posedge clock, negedge reset_n) begin: PID_SWITCH
	if (reset_n == 0) begin
		pid_mux <= 0;
	end else begin
		spi_done_prev <= spi_done;
		if(spi_done_prev==1 && spi_done==0)
			pid_mux <= pid_mux + 1;
	end
end

endmodule
