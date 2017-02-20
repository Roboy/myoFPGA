#include "myoSPI.hpp"

void swap(uint8_t *source, uint8_t *target)
{
	uint8_t buffer = 0;
	buffer = *target;
	*target = *source;
	*source = buffer;
}

void prepareData(SPISTREAM *spistream, int datatype)
{

  if(datatype == WRITE_DATA)
	{
	spistream->TxBuffer[0] = 0x80;
	spistream->TxBuffer[1] = 0x00;
	spistream->pwmRef &= 0x7FFF;
//	int i = 0;
//	for( i =2; i< 8; i = i+2)
//	  spistream->TxBuffer[i] &= ~(1<<7);
  }
  else if(datatype == READ_DATA)
  {
	swap(&spistream->TxBuffer[10],&spistream->TxBuffer[13]);
	swap(&spistream->TxBuffer[11],&spistream->TxBuffer[12]);

	int i = 0;
	for( i = 14; i< 24; i= i+2)
	  swap(&spistream->TxBuffer[i],&spistream->TxBuffer[i+1]);
  }
}

void convertEndianess(SPISTREAM *frame_in, SPISTREAM *frame_out){
	frame_out->startOfFrame = 		(frame_in->TxBuffer[0] << 8) | frame_in->TxBuffer[1];
	frame_out->pwmRef = 			(frame_in->TxBuffer[3] << 8) | frame_in->TxBuffer[2];
	frame_out->controlFlags1 = 		(frame_in->TxBuffer[4] << 8) | frame_in->TxBuffer[5];
	frame_out->controlFlags2 = 		(frame_in->TxBuffer[6] << 8) | frame_in->TxBuffer[7];
	frame_out->dummy = 				(frame_in->TxBuffer[8] << 8) | frame_in->TxBuffer[9];
	frame_out->actualPosition = 	(frame_in->TxBuffer[10] << 24) | (frame_in->TxBuffer[11] << 16) |
									(frame_in->TxBuffer[12] << 8) | frame_in->TxBuffer[13];
	frame_out->actualVelocity = 		(frame_in->TxBuffer[14] << 8) | frame_in->TxBuffer[15];
	frame_out->actualCurrent = 			(frame_in->TxBuffer[16] << 8) | frame_in->TxBuffer[17];
	frame_out->springDisplacement = 	(frame_in->TxBuffer[18] << 8) | frame_in->TxBuffer[19];
	frame_out->sensor1 = 				(frame_in->TxBuffer[20] << 8) | frame_in->TxBuffer[21];
	frame_out->sensor2 = 				(frame_in->TxBuffer[22] << 8) | frame_in->TxBuffer[23];
}

void exchangeFrame( uint32_t *base, uint32_t slave, SPISTREAM *frame){
	SPISTREAM spi_in, frame_out;

	convertEndianess(frame, &frame_out);

	uint16_t *write_data = frame_out.TxBuffer2, *write_end = frame_out.TxBuffer2 + sizeof(frame_out.TxBuffer2)/2;
	uint16_t *read_data = spi_in.TxBuffer2, *read_end = spi_in.TxBuffer2 + sizeof(spi_in.TxBuffer2)/2;

	uint32_t status;

//	IOWR_ALTERA_AVALON_SPI_CONTROL(base, ALTERA_AVALON_SPI_CONTROL_SSO_MSK);
	/* Keep clocking until all the data has been processed. */
	for ( ;; )
	{
		// wait until tx and rx are ready
		do{
			status = IORD_ALTERA_AVALON_SPI_STATUS(base);
		}while((status & ALTERA_AVALON_SPI_STATUS_TRDY_MSK) == 0 &&
				(status & ALTERA_AVALON_SPI_STATUS_RRDY_MSK) == 0 );
//		IOWR_ALTERA_AVALON_SPI_SLAVE_SEL(base, 1 << slave);
		if (write_data < write_end)
			IOWR_ALTERA_AVALON_SPI_TXDATA(base, *write_data++);

		uint32_t rxdata = IORD_ALTERA_AVALON_SPI_RXDATA(base);
		if(read_data < read_end)
			*read_data++ = (uint16_t)rxdata;
		else
			break;

		IOWR_ALTERA_AVALON_SPI_CONTROL(base, 0);
		usleep(1);
	}

	IOWR_ALTERA_AVALON_SPI_CONTROL(base, 0);

	/* Wait until the interface has finished transmitting */
	do{
		status = IORD_ALTERA_AVALON_SPI_STATUS(base);
	}while ((status & ALTERA_AVALON_SPI_STATUS_TMT_MSK) == 0);

	/* Clear SSO (release chipselect) */
	IOWR_ALTERA_AVALON_SPI_CONTROL(base, 0);
	convertEndianess(&spi_in, frame);
}


