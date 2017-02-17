module spi_test (
    input CLOCK_50,
    input [3:0] KEY,
    output [7:0] LED,
	 
	 output [14:0] hps_memory_mem_a,
    output [2:0]  hps_memory_mem_ba,
    output        hps_memory_mem_ck,
    output        hps_memory_mem_ck_n,
    output        hps_memory_mem_cke,
    output        hps_memory_mem_cs_n,
    output        hps_memory_mem_ras_n,
    output        hps_memory_mem_cas_n,
    output        hps_memory_mem_we_n,
    output        hps_memory_mem_reset_n,
    inout  [39:0] hps_memory_mem_dq,
    inout  [4:0]  hps_memory_mem_dqs,
    inout  [4:0]  hps_memory_mem_dqs_n,
    output        hps_memory_mem_odt,
    output [4:0]  hps_memory_mem_dm,
    input         hps_memory_oct_rzqin
);

wire [3:0] key_os;
wire [7:0] delay;
wire main_clk = CLOCK_50;

oneshot os (
    .clk (main_clk),
    .edge_sig (KEY),
    .level_sig (key_os)
);

blinker b (
    .clk (main_clk),
    .delay (delay[3:0]),
    .led (LED),
    .reset (key_os[3]),
    .pause (key_os[2])
);

spi_master spi (
	.sclk_i(main_clk),
	.pclk_i(main_clk),
	.rst_i(reset_p),
	.spi_ssel_o(SS_n),
   .spi_sck_o(SCLK),
   .spi_mosi_o(MOSI),
   .spi_miso_i(MISO),
	.di_req_o(LED[7]),
	.di_i(delay),
	.wren_i(KEY[2]),
	.wr_ack_o(LED[6]),
	.do_valid_o(LED[5]),
	.do_o(data_out)
);

endmodule