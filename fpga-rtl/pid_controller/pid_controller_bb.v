
module pid_controller (
	address,
	write,
	writedata,
	read,
	readdata,
	waitrequest,
	clock,
	o_output,
	reset);	

	input	[3:0]	address;
	input		write;
	input	[31:0]	writedata;
	input		read;
	output	[31:0]	readdata;
	output		waitrequest;
	input		clock;
	output	[31:0]	o_output;
	input		reset;
endmodule
