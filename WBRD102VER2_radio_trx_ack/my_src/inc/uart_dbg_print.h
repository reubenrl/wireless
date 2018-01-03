/*
 * uart_dbg_print.h
 *
 *  Created on: Nov 1, 2016
 *      Author: phytech
 */

#ifndef UART_DBG_PRINT_H_
#define UART_DBG_PRINT_H_

#define PRINT_DBG_CRLF // print crlf

void uartDebugInit(void);
void printDbg(const char* format, ...);
void printDbgCRLF(void);
void printDbgStr(unsigned char nUart, char * pStr);

#endif /* UART_DBG_PRINT_H_ */
