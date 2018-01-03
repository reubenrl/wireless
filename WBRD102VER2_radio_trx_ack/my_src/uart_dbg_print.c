/*
 * uart_dbg_print.c
 *
 *  Created on: Nov 1, 2016
 *      Author: phytech
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "uart_dbg_print.h"
#include "uart2.h"


void printDbg(const char* format, ...)
{
    char       msg[100];

    if(!uartInitialized[DBG_VCOM]){// initialized uart0 (work with VCOM) for print debug
    	uart0Init();
    }

    va_list    args;
    va_start(args, format);
    vsnprintf(msg, sizeof(msg), format, args); // do check return value
    va_end(args);
#if defined(PRINT_DBG_CRLF)
    if(strcat(msg, "\r\n") != NULL){
#endif
    if(!uartSendBuffer(DBG_VCOM,(uint8_t *)msg, strlen(msg))){
    	while(1);
    }
#ifdef PRINT_DBG_CRLF
    }
#endif
}

void printDbgCRLF(void)
{
	printDbg("\r\n");
}

void printDbgStr(unsigned char nUart, char * pStr)
{
    if(!uartInitialized[nUart]){// initialized uart0 (work with VCOM) for print debug
    	if(nUart){
    		uart1Init();
    	}else{
    		uart0Init();
    	}

    }
	while(*pStr){
		uartSendByte(nUart, *pStr);
		pStr++;
	}
}
