from myhdl import *

ACTIVE_LOW, INACTIVE_HIGH = 0, 1

def pid_controller(clock, reset, Kp, Kd, Ki, sp, pv, output, forwardGain, outputPosMax, outputNegMax, timePeriod, IntegralNegMax, IntegralPosMax, deadBand):  
    
    err, pterm, dterm, err, ffterm, integral, lastError, result = [Signal(intbv(0, min=-2147483647, max=2147483647)) for i in range(8)]
    
    @always_seq(clock.posedge, reset=reset)
    def pid_controllerLogic():
        err.next = sp - pv
        if err > deadBand or err < -1*deadBand:
            pterm.next = Kp * err
            if ((pterm < outputPosMax) or (pterm > outputNegMax)): #if the proportional term is not maxed
                integral.next = integral + (Ki * err * timePeriod) #add to the integral
                if integral > IntegralPosMax:
                    integral.next = IntegralPosMax
                elif integral < IntegralNegMax:
                    integral.next = IntegralNegMax
    
            dterm.next = (err - lastError) * Kd;
    
            ffterm.next = forwardGain * sp
            result.next = ffterm + pterm + integral + dterm
            if(result<outputNegMax):
                result.next  = outputNegMax
            elif(result>outputPosMax):
                result.next = outputPosMax
        else:
            result.next = integral
    
        lastError.next = err;
    
#          //check for control limits
        if(result>4000):#//check limit using raw parameters as all controllers will contain the elements specified
            result.next = 4000;
        elif(result<-4000):
            result.next = -4000;
            
        output.next = result
                
    return pid_controllerLogic
    

from random import randrange

def testbench_pid_controller():
    Kp, Kd, Ki, sp, pv, output, forwardGain, outputPosMax, outputNegMax, timePeriod, IntegralNegMax, IntegralPosMax, deadBand = [Signal(intbv(0, min=-2147483647, max=2147483647)) for i in range(13)]
    clock = Signal(intbv(0))
    reset = ResetSignal(0, active=ACTIVE_LOW, async=True)

    pid = pid_controller(clock, reset, Kp, Kd, Ki, sp, pv, output, forwardGain, outputPosMax, outputNegMax, timePeriod, IntegralNegMax, IntegralPosMax, deadBand)

    HALF_PERIOD = delay(10)

    @always(HALF_PERIOD)
    def clockGen():
        clock.next = not clock

    @instance
    def stimulus():
        reset.next = ACTIVE_LOW
        yield clock.negedge
        reset.next = INACTIVE_HIGH
        Kp.next = 100
        Kd.next = 0
        Ki.next = 0
        sp.next = 1
        pv.next = 0
        forwardGain.next = 0
        outputPosMax.next = 100
        outputNegMax.next = -100
        timePeriod.next = 1
        IntegralNegMax.next = -100
        IntegralPosMax.next = 100
        yield clock.negedge
        for i in range(10):
            yield clock.negedge
        raise StopSimulation

    @instance
    def monitor():
        print "testbench pid_controller" 
        yield reset.posedge
        while 1:
            print "sp: %s    result: %s"% (sp, output)
            yield clock.posedge
            yield delay(1)


    return clockGen, stimulus, pid, monitor


tb0 = testbench_pid_controller()
Simulation(tb0).run()
 
def convert_pid_controller():
    Kp, Kd, Ki, sp, pv, result, forwardGain, outputPosMax, outputNegMax, timePeriod, IntegralNegMax, IntegralPosMax, deadBand = [Signal(intbv(0, min=-2147483647, max=2147483647)) for i in range(13)]
    clock = Signal(intbv(0,min=0,max=1))
    reset = ResetSignal(0, active=ACTIVE_LOW, async=True)
    toVerilog(pid_controller, clock, reset, Kp, Kd, Ki, sp, pv, result, forwardGain, outputPosMax, outputNegMax, timePeriod, IntegralNegMax, IntegralPosMax, deadBand)
 
convert_pid_controller()
