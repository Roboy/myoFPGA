/*
 * spi.h
 *
 *  Created on: Feb 19, 2017
 *      Author: letrend
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

#define IORD(base,reg) (*(((volatile uint32_t*)base)+reg))
#define IOWR(base,reg,data) (*(((volatile uint32_t*)base)+reg)=data)

#define ALTERA_AVALON_SPI_RXDATA_REG                  0
#define IORD_ALTERA_AVALON_SPI_RXDATA(base)           IORD(base, ALTERA_AVALON_SPI_RXDATA_REG)
#define IOWR_ALTERA_AVALON_SPI_RXDATA(base, data)     IOWR(base, ALTERA_AVALON_SPI_RXDATA_REG, data)

#define ALTERA_AVALON_SPI_TXDATA_REG                  1
#define IORD_ALTERA_AVALON_SPI_TXDATA(base)           IORD(base, ALTERA_AVALON_SPI_TXDATA_REG)
#define IOWR_ALTERA_AVALON_SPI_TXDATA(base, data)     IOWR(base, ALTERA_AVALON_SPI_TXDATA_REG, data)

#define ALTERA_AVALON_SPI_STATUS_REG                  2
#define IORD_ALTERA_AVALON_SPI_STATUS(base)           IORD(base, ALTERA_AVALON_SPI_STATUS_REG)
#define IOWR_ALTERA_AVALON_SPI_STATUS(base, data)     IOWR(base, ALTERA_AVALON_SPI_STATUS_REG, data)

#define ALTERA_AVALON_SPI_STATUS_ROE_MSK              (0x8)
#define ALTERA_AVALON_SPI_STATUS_ROE_OFST             (3)
#define ALTERA_AVALON_SPI_STATUS_TOE_MSK              (0x10)
#define ALTERA_AVALON_SPI_STATUS_TOE_OFST             (4)
#define ALTERA_AVALON_SPI_STATUS_TMT_MSK              (0x20)
#define ALTERA_AVALON_SPI_STATUS_TMT_OFST             (5)
#define ALTERA_AVALON_SPI_STATUS_TRDY_MSK             (0x40)
#define ALTERA_AVALON_SPI_STATUS_TRDY_OFST            (6)
#define ALTERA_AVALON_SPI_STATUS_RRDY_MSK             (0x80)
#define ALTERA_AVALON_SPI_STATUS_RRDY_OFST            (7)
#define ALTERA_AVALON_SPI_STATUS_E_MSK                (0x100)
#define ALTERA_AVALON_SPI_STATUS_E_OFST               (8)

#define ALTERA_AVALON_SPI_CONTROL_REG                 3
#define IORD_ALTERA_AVALON_SPI_CONTROL(base)          IORD(base, ALTERA_AVALON_SPI_CONTROL_REG)
#define IOWR_ALTERA_AVALON_SPI_CONTROL(base, data)    IOWR(base, ALTERA_AVALON_SPI_CONTROL_REG, data)

#define ALTERA_AVALON_SPI_CONTROL_IROE_MSK            (0x8)
#define ALTERA_AVALON_SPI_CONTROL_IROE_OFST           (3)
#define ALTERA_AVALON_SPI_CONTROL_ITOE_MSK            (0x10)
#define ALTERA_AVALON_SPI_CONTROL_ITOE_OFST           (4)
#define ALTERA_AVALON_SPI_CONTROL_ITRDY_MSK           (0x40)
#define ALTERA_AVALON_SPI_CONTROL_ITRDY_OFS           (6)
#define ALTERA_AVALON_SPI_CONTROL_IRRDY_MSK           (0x80)
#define ALTERA_AVALON_SPI_CONTROL_IRRDY_OFS           (7)
#define ALTERA_AVALON_SPI_CONTROL_IE_MSK              (0x100)
#define ALTERA_AVALON_SPI_CONTROL_IE_OFST             (8)
#define ALTERA_AVALON_SPI_CONTROL_SSO_MSK             (0x400)
#define ALTERA_AVALON_SPI_CONTROL_SSO_OFST            (10)

#define ALTERA_AVALON_SPI_SLAVE_SEL_REG               5
#define IORD_ALTERA_AVALON_SPI_SLAVE_SEL(base)        IORD(base, ALTERA_AVALON_SPI_SLAVE_SEL_REG)
#define IOWR_ALTERA_AVALON_SPI_SLAVE_SEL(base, data)  IOWR(base, ALTERA_AVALON_SPI_SLAVE_SEL_REG, data)



#endif /* SPI_H_ */
