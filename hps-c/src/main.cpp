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
#include "myoControl.hpp"

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int main() {

	void *virtual_base;
	int fd;
	int loop_count;
	int led_direction = 0;
	int led_mask = 0x01;
	void *h2p_lw_led_addr, *h2p_lw_spi_addr, *h2p_lw_adc_addr;

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
	h2p_lw_adc_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	
	MyoControl myoControl(7,(uint32_t*)h2p_lw_spi_addr);
	myoControl.changeControl(1,Position);
	myoControl.changeControl(2,Position);
	myoControl.changeControl(3,Position);
	myoControl.changeControl(4,Position);
	myoControl.changeControl(5,Position);
	myoControl.changeControl(6,Position);
	myoControl.changeControl(7,Position);
	myoControl.setPosition(1,10);
	myoControl.setPosition(2,10);
	myoControl.setPosition(3,10);
	myoControl.setPosition(4,10);
	myoControl.setPosition(5,10);
	myoControl.setPosition(6,10);
	myoControl.setPosition(7,10);
	int iter = 0;
	while( true ) {
		myoControl.update();

		// Toggling the LEDs to show off
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
		}

	}
	
	// clean up our memory mapping and exit
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}
