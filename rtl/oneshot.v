module oneshot (
	 input clk,
	 input [width-1:0] edge_sig,
	 output [width-1:0] level_sig
	);
	
parameter width = 4;

reg [width-1:0] cur_value;
reg [width-1:0] last_value;

assign level_sig = ~cur_value & last_value;

always @(posedge clk) begin
    cur_value <= edge_sig;
    last_value <= cur_value;
end

endmodule