module tb_pid_controller;

reg [0:0] clock;
reg reset;
reg [31:0] Kp;
reg [31:0] Kd;
reg [31:0] Ki;
reg [31:0] sp;
reg [31:0] pv;
wire [31:0] output;
reg [31:0] forwardGain;
reg [31:0] outputPosMax;
reg [31:0] outputNegMax;
reg [31:0] timePeriod;
reg [31:0] IntegralNegMax;
reg [31:0] IntegralPosMax;
reg [31:0] deadBand;

initial begin
    $from_myhdl(
        clock,
        reset,
        Kp,
        Kd,
        Ki,
        sp,
        pv,
        forwardGain,
        outputPosMax,
        outputNegMax,
        timePeriod,
        IntegralNegMax,
        IntegralPosMax,
        deadBand
    );
    $to_myhdl(
        output
    );
end

pid_controller dut(
    clock,
    reset,
    Kp,
    Kd,
    Ki,
    sp,
    pv,
    output,
    forwardGain,
    outputPosMax,
    outputNegMax,
    timePeriod,
    IntegralNegMax,
    IntegralPosMax,
    deadBand
);

endmodule
