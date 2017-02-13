module delay_ctrl (
    input clk,
    input reset,

    input faster,
    input slower,
    output [7:0] delay,

    input write,
    input [7:0] writedata
);

reg [7:0] delay_intern = 8'b00001000;

assign delay = delay_intern;

always @(posedge clk) begin
    if (reset)
        delay_intern <= 8'b00001000;
    else if (write)
        delay_intern <= writedata[7:0];
    else if (faster && delay_intern != 4'b1000)
        delay_intern <= delay_intern - 1'b1;
    else if (slower && delay_intern != 4'b1111)
        delay_intern <= delay_intern + 1'b1;
end

endmodule