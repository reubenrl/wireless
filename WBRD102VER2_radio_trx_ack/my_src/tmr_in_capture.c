/*
 * tmr_in_capture.c
 *
 *  Created on: Aug 28, 2017
 *      Author: phytech
 */


#include <stdio.h>

#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "em_dma.h"
#include "em_system.h"
#include "em_timer.h"
#include "dmactrl.h"


#include "uart_dbg_print.h"

#include "tmr_in_capture.h"

static bool tmr2InCaptureInitialized = false;
static bool tmr2InCaptureOverflow = false;
static bool tmr2InCaptureOccurred = false;

static uint16_t bufferA[BUFFERSIZE];
volatile uint8_t bufferAIdx;
//volatile uint16_t bufferB[BUFFERSIZE];


static unsigned int ovrfCount=0;
volatile uint8_t timerStatus;


uint16_t *tmr2InCaptureGet(void)
{
	bufferAIdx = 0;
	tmr2InCaptureOccurred = false;
	TIMER2->CNT = 0;

	while(!tmr2InCaptureOccurred && ((ovrfCount/TOP) < 10u));

	return &bufferA[0];
}

/**************************************************************************//**
 * @brief TIMER2_IRQHandler
 * Interrupt Service Routine TIMER2 Interrupt Line
 *****************************************************************************/
void TIMER2_IRQHandler(void)
{
	uint16_t intFlags = TIMER_IntGet(TIMER2);

	TIMER_IntClear(TIMER2, TIMER_IF_OF | TIMER_IF_CC1);

	timerStatus = (TIMER2->STATUS & TIMER_STATUS_ICV1) ? 1 : 0;


	/* Overflow interrupt occurred */
	if(intFlags & TIMER_IF_OF){
		/* Increment the counter with TOP = 0xFFFF */
		ovrfCount += TOP;

		//if(tmr2InCaptureOverflow == false) countSaved = count;

		/* Write overflow number */
	//	SPRINTF(overFlowString, "OVRFLW%d", count / TOP);
	//	SegmentLCD_Write(overFlowString);
		tmr2InCaptureOverflow = true;
	}

	/* Capture interrupt occurred */
	if(intFlags & TIMER_IF_CC1){
		/* Calculate total time of button pressing */
		//totalTime = count + TIMER_CaptureGet(TIMER2, 1);
	//	if(!tmr2InCaptureOccurred)
		if(timerStatus){
		//totalTimeSaved = (ovrfCountSaved + TIMER2->CC[1].CCVB) -  TIMER_CaptureGet(TIMER2, 1);
			if(bufferAIdx < BUFFERSIZE){
				bufferA[bufferAIdx++] = ovrfCount + TIMER_CaptureGet(TIMER2, 1);
			}else{
				tmr2InCaptureOccurred = true;
			}
		}
		/* Multiply by 1000 to avoid floats */
		//totalTime = (totalTime * 1000) / TIMER_FREQ;

		//if(tmr2InCaptureOccurred == false) totalTimeSaved = totalTime;


		/* Write time in seconds on the LCD */
		//SPRINTF(totalTimeString, "%d.%.3d", totalTime/1000, totalTime%1000);
		//SegmentLCD_Write(totalTimeString);

		/* Clear counter */
		//count = 0;
		ovrfCount = 0;

	}


}

/*

*************************************************************************
 * @brief  Call-back called when transfer is complete
 ****************************************************************************
void tmr2InCaptureDmaTransferComplete(unsigned int channel, bool primary, void *user)
{
  (void) user;
  BSP_LedToggle(LED1);

   Re-activate the DMA
  DMA_RefreshPingPong(channel,
                      primary,
                      false,
                      NULL,
                      NULL,
                      BUFFERSIZE - 1,
                      false);
}
*/

/*

 DMA callback structure
DMA_CB_TypeDef cb;

*************************************************************************
 * @brief Configure DMA for Ping-Pong transfers
 ****************************************************************************
void tmr2InCaptureDmaSetup(void)
{
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  chnlCfg;
  DMA_CfgDescr_TypeDef    descrCfg;

   Initializing the DMA
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

   Setup call-back function
  cb.cbFunc  = tmr2InCaptureDmaTransferComplete;
  cb.userPtr = NULL;

   Setting up channel
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_TIMER2_CC1;
  chnlCfg.cb        = &cb;
  DMA_CfgChannel(DMA_CHANNEL_TIMER2, &chnlCfg);

   Setting up channel descriptor
  descrCfg.dstInc  = dmaDataInc2;
  descrCfg.srcInc  = dmaDataIncNone;
  descrCfg.size    = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_TIMER2, true, &descrCfg);
  DMA_CfgDescr(DMA_CHANNEL_TIMER2, false, &descrCfg);

   Enabling PingPong Transfer
  DMA_ActivatePingPong(DMA_CHANNEL_TIMER2,
                       false,
                       (void *)&bufferA,
                       (void *)&(TIMER2->CC[1].CCV),
                       BUFFERSIZE - 1,
                       (void *)&bufferB,
                       (void *)&(TIMER2->CC[1].CCV),
                        BUFFERSIZE - 1);
}
*/


void tmr2InCaptureInit(void)
{
	if(tmr2InCaptureInitialized == true) return;
	tmr2InCaptureInitialized = true;

	/* Enable clock for GPIO module */
	CMU_ClockEnable(cmuClock_GPIO, true);

	/* Enable clock for TIMER2 module */
	CMU_ClockEnable(cmuClock_TIMER2, true);

	/* Enable clock for PRS module */
	//CMU_ClockEnable(cmuClock_PRS, true);

	/* Setup DMA */
	//tmr2InCaptureDmaSetup();

	/* Configure PB0_PIN as an input for PB0 button with filter and pull-up (dout = 1)*/
	GPIO_PinModeSet(GPIO_TMR_IN_CAPTURE_PORT, GPIO_TMR_IN_CAPTURE_PIN, gpioModeInputPullFilter, 1);

	/* Select PB0_PIN as external interrupt source*/
	//GPIO_IntConfig(GPIO_TMR_IN_CAPTURE_PORT, GPIO_TMR_IN_CAPTURE_PIN, false, false, false);

	/* Enable PRS sense on GPIO and disable interrupt sense */
	//GPIO_InputSenseSet(GPIO_INSENSE_PRS, _GPIO_INSENSE_RESETVALUE);

	/* Select GPIO as source and GPIO_TMR_IN_CAPTURE_PRS_SIGSEL_PIN as signal for PRS channel 0 */
	//PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_GPIOH, GPIO_TMR_IN_CAPTURE_PRS_SIGSEL_PIN, prsEdgeOff);

	/* Select CC channel parameters */
	TIMER_InitCC_TypeDef timerCCInit =
	{
		.eventCtrl  = timerEventEveryEdge,
		.edge       = timerEdgeRising,
		.prsSel     = timerPRSSELCh0,
		.cufoa      = timerOutputActionNone,
		.cofoa      = timerOutputActionNone,
		.cmoa       = timerOutputActionNone,
		.mode       = timerCCModeCapture,
		.filter     = true,
		.prsInput   = false,
		.coist      = false,
		.outInvert  = false,
	};

	/* Configure CC channel 1 */
	TIMER_InitCC(TIMER2, 1, &timerCCInit);


	/* Select timer parameters */
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = false,
		.debugRun   = true,
		.prescale   = timerPrescale32,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionReloadStart,
		.riseAction = timerInputActionStop,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = false,
	};

	/* Enable overflow and CC1 interrupt */
	TIMER_IntEnable(TIMER2, TIMER_IF_OF | TIMER_IF_CC1);

	/* Enable TIMER0 interrupt vector in NVIC */
	NVIC_EnableIRQ(TIMER2_IRQn);

	/* Configure timer */
	TIMER_Init(TIMER2, &timerInit);

	/* Module TIMER2 is configured to location 3 */
	TIMER2->ROUTE = (TIMER2->ROUTE & ~_TIMER_ROUTE_LOCATION_MASK) | TIMER_ROUTE_LOCATION_LOC3;

	/* Enable signals CC1 */
	TIMER2->ROUTE |= TIMER_ROUTE_CC1PEN;
	TIMER_Enable(TIMER2, true);

}

void tmr2InCaptureDeInit(void)
{



	/* Enable overflow and CC1 interrupt */
	TIMER_IntDisable(TIMER2, TIMER_IF_OF | TIMER_IF_CC1);

	/* Enable TIMER0 interrupt vector in NVIC */
	NVIC_EnableIRQ(TIMER2_IRQn);

}
