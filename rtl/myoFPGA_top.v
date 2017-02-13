module myoFPGA_top (
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
    input         hps_memory_oct_rzqin,
	 
	 output SS_n,
	 output MOSI,
	 input MISO,
	 output SCLK
);

wire [7:0] delay;
wire main_clk = CLOCK_50;
wire [7:0] data_out;
wire [3:0] key = KEY[3:0];
wire [7:0] led = LED[7:0];

soc_system soc (
    .delay_ctrl_delay (delay),
    .delay_ctrl_faster (key[0]),
    .delay_ctrl_slower (key[1]),
    .memory_mem_a        (hps_memory_mem_a),
    .memory_mem_ba       (hps_memory_mem_ba),
    .memory_mem_ck       (hps_memory_mem_ck),
    .memory_mem_ck_n     (hps_memory_mem_ck_n),
    .memory_mem_cke      (hps_memory_mem_cke),
    .memory_mem_cs_n     (hps_memory_mem_cs_n),
    .memory_mem_ras_n    (hps_memory_mem_ras_n),
    .memory_mem_cas_n    (hps_memory_mem_cas_n),
    .memory_mem_we_n     (hps_memory_mem_we_n),
    .memory_mem_reset_n  (hps_memory_mem_reset_n),
    .memory_mem_dq       (hps_memory_mem_dq),
    .memory_mem_dqs      (hps_memory_mem_dqs),
    .memory_mem_dqs_n    (hps_memory_mem_dqs_n),
    .memory_mem_odt      (hps_memory_mem_odt),
    .memory_mem_dm       (hps_memory_mem_dm),
    .memory_oct_rzqin    (hps_memory_oct_rzqin),

    .clk_clk (main_clk),
    .reset_reset_n (!key[3])
);

blinker b (
    .clk (main_clk),
    .delay (delay[3:0]),
    .led (led[3:0]),
    .reset (key[3]),
    .pause (key[2])
);

spi_master spi (
	.sclk_i(main_clk),
	.pclk_i(main_clk),
	.rst_i(reset_p),
	.spi_ssel_o(SS_n),
   .spi_sck_o(SCLK),
   .spi_mosi_o(MOSI),
   .spi_miso_i(MISO),
	.di_req_o(led[7]),
	.di_i(delay),
	.wren_i(key[2]),
	.wr_ack_o(led[6]),
	.do_valid_o(led[5]),
	.do_o(data_out)
);

endmodule