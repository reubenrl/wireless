/*
 * uartsw.h
 *
 *  Created on: Dec 11, 2017
 *      Author: phytech
 */

#ifndef MY_SRC_INC_UARTSW_H_
#define MY_SRC_INC_UARTSW_H_

// include configuration
#include "uartswconf.h"
#include "buffer.h"

// constants/macros/typdefs

// functions

//! enable and initialize the software uart
void uartswInit(void);
//! create and initialize the uart buffers
void uartswInitBuffers(void);
//! turns off software UART
void uartswOff(void);
//! returns the receive buffer structure
cBuffer* uartswGetRxBuffer(void);
//! sets the uart baud rate
void uartswSetBaudRate(uint32_t baudrate);
//! sends a single byte over the uart
void uartswSendByte(uint8_t data);

//! gets a single byte from the uart receive buffer
// Function returns TRUE if data was available, FALSE if not.
// Actual data is returned in variable pointed to by "data".
// example usage:
// char myReceivedByte;
// uartswReceiveByte( &myReceivedByte );
uint8_t uartswReceiveByte(uint8_t* rxData);

//! internal transmit bit handler
void uartswTxBitService(void);
//! internal receive bit handler
void uartswRxBitService(void);

void uartswFlushReceiveBuffer(void);

#endif /* MY_SRC_INC_UARTSW_H_ */
