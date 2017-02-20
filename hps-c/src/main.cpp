#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <pidController.hpp>
#include <pidController.hpp>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"

#include "hps_0.h"
#include "myoSPI.hpp"
#include "pidController.hpp"

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int main() {

	void *virtual_base;
	int fd;
	int loop_count;
	int led_direction;
	int led_mask;
	void *h2p_lw_led_addr, *h2p_lw_spi_addr;

	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span

	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	
	h2p_lw_led_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_LED_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_spi_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SPI_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	

	// toggle the LEDs a bit

	loop_count = 0;
	led_mask = 0x01;
	led_direction = 0; // 0: left to right direction

	SPISTREAM frame;

	control_Parameters_t controlparams;
	controlparams.tag = 0;              // sint32
	controlparams.outputPosMax = 1000;  // sint32
	controlparams.outputNegMax = -1000; // sint32
	controlparams.timePeriod = 100; // float32      //in us set time period to avoid error case

	controlparams.radPerEncoderCount =
			2 * 3.14159265359 / (2000.0 * 53.0);          // float32
	controlparams.params.pidParameters.lastError = 0; // float32

	controlparams.spPosMax = 1000; // float32
	controlparams.spNegMax = -1000; // float32

	controlparams.params.pidParameters.integral = 0;             // float32
	controlparams.params.pidParameters.pgain = 10.0;                   // float32
	controlparams.params.pidParameters.igain = 0;                   // float32
	controlparams.params.pidParameters.dgain = 0;                   // float32
	controlparams.params.pidParameters.forwardGain = 0;       // float32
	controlparams.params.pidParameters.deadBand = 0;             // float32
	controlparams.params.pidParameters.IntegralPosMax = 0; // float32
	controlparams.params.pidParameters.IntegralNegMax = 0; // float32

	pidController controller(&controlparams);
	float setPoint = 1;
	float pos = setPoint, vel = setPoint;
	short control;

	int iter = 0;

	while( loop_count < 60 ) {
		for(uint i = 0; i<24;i++)
			frame.TxBuffer[i] = 0;
		control = controller.outputCalc(pos,setPoint);

		frame.pwmRef = control;

		prepareData(&frame, WRITE_DATA);
		exchangeFrame((uint32_t*)h2p_lw_spi_addr, 0, &frame);

		prepareData(&frame, READ_DATA);

		  pos = frame.actualPosition*controlparams.radPerEncoderCount;
		  vel = frame.actualVelocity*controlparams.radPerEncoderCount;

		  if((iter++)%100==0){
			// control led
			*(uint32_t *)h2p_lw_led_addr = ~led_mask;

			// update led mask
			if (led_direction == 0){
				led_mask <<= 1;
				if (led_mask == (0x01 << (PIO_LED_DATA_WIDTH-1)))
					 led_direction = 1;
			}else{
				led_mask >>= 1;
				if (led_mask == 0x01){
					led_direction = 0;
					loop_count++;
				}
			}

			printf("===================================\n");
			printf( "startOfFrame:         %d\n"
				  "pwmRef:               %d\n"
				  "controlFlags1:        %d\n"
				  "controlFlags2:        %d\n"
				  "dummy:                %d\n"
				  "actualPosition:       %f\n"
				   "actualVelocity:      %f\n"
				   "actualCurrent:       %d\n"
				   "springDisplacement:  %d\n"
				   "sensor1:             %d\n"
				   "sensor2:             %d\n",
				   frame.startOfFrame, control, frame.controlFlags1, frame.controlFlags2, frame.dummy,
				   pos, vel, frame.actualCurrent, frame.springDisplacement, frame.sensor1, frame.sensor2);
		  }



	} // while
	

	// clean up our memory mapping and exit
	
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}
