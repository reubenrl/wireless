/*
 * uartswconf.h
 *
 *  Created on: Dec 11, 2017
 *      Author: phytech
 */

#ifndef SRC_UARTSWCONF_H_
#define SRC_UARTSWCONF_H_

// constants/macros/typdefs

#define UARTSW_RX_BUFFER_SIZE	0x40	///< UART receive buffer size in bytes

#define UARTSW_INVERT					///< define to invert polarity of RX/TX signals
// when non-inverted, the serial line is appropriate for passing though
// an RS232 driver like the MAX232.  When inverted, the serial line can
// directly drive/receive RS232 signals to/from a DB9 connector.  Be sure
// to use a current-limiting resistor and perhaps a diode-clamp circuit when
// connecting incoming RS232 signals to a microprocessor I/O pin.

// if non-inverted, the serial line idles high (logic 1) between bytes
// if inverted, the serial line idles low (logic 0) between bytes

// UART transmit pin defines
#define UARTSW_TX_PORT			GPIO_SENS_DAT_IN_PORT	///< UART Transmit Port
#define UARTSW_TX_PIN			12						///< UART Transmit Pin

// UART receive pin defines
#define UARTSW_RX_PORT			GPIO_SENS_DAT_IN_PORT	///< UART Receive Port
#define UARTSW_RX_PIN			12		///< UART Receive Pin


#endif /* SRC_UARTSWCONF_H_ */
