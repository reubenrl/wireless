/*
 * tmr_in_capture.h
 *
 *  Created on: Aug 28, 2017
 *      Author: phytech
 */

#ifndef MY_SRC_INC_TMR_IN_CAPTURE_H_
#define MY_SRC_INC_TMR_IN_CAPTURE_H_

#include "bsp.h"


/* TOP reset value is 0xFFFF so it doesn't need
   to be written for this example */
#define TOP 			0xFFFF

/* 13761 Hz -> 14Mhz (clock frequency) / 1024 (prescaler) */
//#define TIMER_FREQ 13671

#define TIMER_FREQ 		24000000u/32u	// 3000000Hz

#define GPIO_TMR_IN_CAPTURE_PORT				GPIO_SENS_DAT_IN_PORT
#define GPIO_TMR_IN_CAPTURE_PIN					GPIO_SENS_DAT_IN_PIN
#define GPIO_TMR_IN_CAPTURE_PRS_SIGSEL_PIN 		PRS_CH_CTRL_SIGSEL_GPIOPIN12



#define DMA_CHANNEL_TIMER2       	0
#define BUFFERSIZE               	8

void tmr2InCaptureInit(void);
uint16_t *tmr2InCaptureGet(void);

#endif /* MY_SRC_INC_TMR_IN_CAPTURE_H_ */
