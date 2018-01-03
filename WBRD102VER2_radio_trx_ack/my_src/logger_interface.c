/*
 * logger_interface.c
 *
 *  Created on: Mar 13, 2017
 *      Author: phytech
 */
#include "bsp.h"

#if defined(WBRD200_VER1)
#include <stdlib.h>
#include <string.h>
#include "em_cmu.h"
#include "em_timer.h"
#include "rtcdriver.h"
#include "uart2.h"
#include "logger_interface.h"


const char strPacketSend2Logger[] = "I'm sending data for you...\r\n";


//static RTCDRV_TimerID_t loggerTickTimerID;
static volatile uint32_t rtcLoggerTickCounter = 0;
static volatile uint8_t  uartRxTimeout_sec = 0;
static volatile uint32_t uartRxTimeout_msec = 0;

static volatile bool logger_interfaceInitialized = false;
/*

void RTC_Logger_IRQHandler()
{// every 1 miliseconds
	rtcLoggerTickCounter++;

	if(!(rtcLoggerTickCounter % 10u)){
		if(uartRxTimeout_sec) uartRxTimeout_sec--;
	}

	if(uartRxTimeout_msec) uartRxTimeout_msec--;
}
*/


/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER1_IRQHandler(void)
{
  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER1, TIMER_IF_OF);

	rtcLoggerTickCounter++;

	if(!(rtcLoggerTickCounter % 1000u)){
		if(uartRxTimeout_sec) uartRxTimeout_sec--;
	}

	if(uartRxTimeout_msec) uartRxTimeout_msec--;
}


void logger_interfaceInit(void)
{
	if(!logger_interfaceInitialized){
		logger_interfaceInitialized = true;

		if(!uartInitialized[UART_LOGGER]){
			uart1Init();
		}
/*
		RTCDRV_Init();
		if (ECODE_EMDRV_RTCDRV_OK !=
			  RTCDRV_AllocateTimer( &loggerTickTimerID) )
		  {
			while (1);
		  }
		  if (ECODE_EMDRV_RTCDRV_OK !=
			  RTCDRV_StartTimer(loggerTickTimerID, rtcdrvTimerTypePeriodic, 100u,
								(RTCDRV_Callback_t)RTC_Logger_IRQHandler, NULL ) )
		  {
			while (1);
		  }
		  */


		/* Enable clock for TIMER0 module */
		  CMU_ClockEnable(cmuClock_TIMER1, true);

		  /* Select TIMER0 parameters */
		  TIMER_Init_TypeDef timerInit =
		  {
		    .enable     = true,
		    .debugRun   = true,
		    .prescale   = timerPrescale1024,
		    .clkSel     = timerClkSelHFPerClk,
		    .fallAction = timerInputActionNone,
		    .riseAction = timerInputActionNone,
		    .mode       = timerModeUp,
		    .dmaClrAct  = false,
		    .quadModeX4 = false,
		    .oneShot    = false,
		    .sync       = false,
		  };

		  /* Enable overflow interrupt */
		  TIMER_IntEnable(TIMER1, TIMER_IF_OF);

		  /* Enable TIMER0 interrupt vector in NVIC */
		  NVIC_EnableIRQ(TIMER1_IRQn);

		  /* Set TIMER Top value */
		  TIMER_TopSet(TIMER1, 24000000u/1024u/1000u);

		  /* Configure TIMER */
		  TIMER_Init(TIMER1, &timerInit);
	}

}

void logger_interfaceSendBuffer(void)
{
	logger_interfaceInit();

	uint8_t *pBuffer = (uint8_t *)malloc(sizeof(strPacketSend2Logger) + 4);

	if(pBuffer != NULL){

		if(strcpy((char *)pBuffer, strPacketSend2Logger) != NULL){
			(void)uartSendBuffer(UART_LOGGER, pBuffer, strlen((char *)pBuffer));
		}

		free(pBuffer);
	}

}

uint16_t logger_interfaceGetBuffer(uint8_t strIn[], uint16_t nLenBuff, uint8_t nTimeout_sec)
{
	uint16_t idx;
	uint8_t c;

	logger_interfaceInit();
	uartFlushReceiveBuffer(UART_LOGGER);
	uartRxTimeout_sec = nTimeout_sec;
	idx = 0;
	do{
		if(!uartReceiveBufferIsEmpty(UART_LOGGER)){
			//uartRxTimeout_msec = 250u;
			//uartRxTimeout_msec = 200u;
			//while(uartRxTimeout_msec);
			do{
				if(uartReceiveByte(UART_LOGGER, &c)){
					if(idx <= nLenBuff ){
						strIn[idx++] = c;
					}else return idx;

					uartRxTimeout_msec = 100u;
				}//else return idx;
			}while(uartRxTimeout_msec && uartRxTimeout_sec);

			return idx;
		}
	}while(uartRxTimeout_sec);

	return 0;
}

#endif
