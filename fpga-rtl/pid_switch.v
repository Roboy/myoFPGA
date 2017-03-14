`timescale 1ns/10ps

module pid_switch (
   input clock,
	input reset_n,
	input spi_done,
	input wire ss_n,
	output wire [9:0] ss_n_o
);

reg spi_done_prev;
reg [3:0]pid_mux;

always @(posedge clock, negedge reset_n) begin: PID_SWITCH
	if (reset_n == 0) begin
		pid_mux <= 0;
	end else begin
		spi_done_prev <= spi_done;
		if(spi_done_prev==1 && spi_done==0) begin
			if(pid_mux<9)
				pid_mux <= pid_mux + 1;
			else 
				pid_mux <= 0;
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
assign ss_n_o[8] = (pid_mux==8?ss_n:1);
assign ss_n_o[9] = (pid_mux==9?ss_n:1);


endmodule
