/*
 * cap_meter.c
 *
 *  Created on: Sep 13, 2017
 *      Author: phytech
 */

#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "em_adc.h"

#include "uart_dbg_print.h"

volatile bool 	tmr1CapMeterInitialized = false, tmr1Overflow = false;
volatile bool 	adc0CapMeterInitialized = false, adcFinished = false, \
				adcConversionOverflow = false, adcConversionComplete = false;
static int adcSampleCount;

static  uint16_t adcSampleBuff[128];
static uint16_t countTop = 0;


/**************************************************************************//**
 * @brief TIMER2_IRQHandler
 * Interrupt Service Routine TIMER2 Interrupt Line
 *****************************************************************************/
void TIMER1_IRQHandler(void)
{
	uint16_t intFlags = TIMER_IntGet(TIMER1);

	TIMER_IntClear(TIMER1, TIMER_IF_OF);


	countTop += 0xFFFF;

	if(intFlags & TIMER_IF_OF){
		tmr1Overflow = true;
	}

	/* Overflow interrupt occurred */
//	if(intFlags & TIMER_IF_OF){
//		GPIO_PinOutToggle(gpioPortF, 4);
		//ADC_Start(ADC0, adcStartSingle);
//		ADC0->CMD = (uint32_t)1u;
		/* Wait while conversion is active */
//		while (ADC0->STATUS & ADC_STATUS_SINGLEACT);
		/* Get ADC result */
		//adcSampleBuff[adcSampleCount++] = (uint16_t)ADC0->SINGLEDATA;// (uint16_t)ADC_DataSingleGet(ADC0);
/*
		if(adcSampleCount++ >= 128){
			adcFinished = true;
			TIMER_IntDisable(TIMER1, _TIMER_IF_MASK);
			TIMER_Enable(TIMER1, false);
		 }*/
//	}else{
//		GPIO_PinOutSet(gpioPortF, 3);
//	}

}


void tmr1CapMeterInit(void)
{
	if(tmr1CapMeterInitialized == true) return;
	tmr1CapMeterInitialized = true;

	/* Enable clock for TIMER1 module */
	CMU_ClockEnable(cmuClock_TIMER1, true);

	/* Select timer parameters */
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = false,
		.debugRun   = true,
		.prescale   = timerPrescale1,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = false,
	};

	//	TIMER_TopSet(TIMER1, 24000000u/200000u); // 5usec
//		TIMER_TopSet(TIMER1, 24000000u/125000u); // 8usec
//		TIMER_TopSet(TIMER1, 24000000u/100000u);	// 10usec
		TIMER_TopSet(TIMER1, 24000000u/62500u);	// 16usec
//		TIMER_TopSet(TIMER1, 24000000u/20000u); // 20usec


	/* Enable overflow  interrupt */
	TIMER_IntEnable(TIMER1, TIMER_IF_OF);

	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	/* Enable TIMER1 interrupt vector in NVIC */
	NVIC_EnableIRQ(TIMER1_IRQn);
	TIMER1->CNT = 0;
	/* Configure timer */
	TIMER_Init(TIMER1, &timerInit);

}

/**************************************************************************//**
 * @brief ADC Interrupt handler (ADC0)
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  uint32_t register flags;

  /* Clear interrupt flags */
  flags = ADC_IntGet( ADC0 );
  ADC_IntClear( ADC0, flags );

  if(flags & ADC_IF_SINGLEOF){
	  adcConversionOverflow = true;
  }
  if(flags & ADC_IF_SINGLE){
	  if(!(adcSampleCount & 128)){
		  adcSampleBuff[adcSampleCount] = (uint16_t)ADC_DataSingleGet(ADC0);
		 adcSampleCount++;
	  }else{
		  adcConversionComplete = true;
		  ADC_IntDisable(ADC0, ADC_IEN_SINGLE | ADC_IEN_SINGLEOF);
	  }
  }
}


void adc0CapMeterInit(void)
{
	if(adc0CapMeterInitialized == true) return;
	adc0CapMeterInitialized = true;

	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

	/* Enable clock for ADC0 */
	CMU_ClockEnable(cmuClock_ADC0, true);

	init.ovsRateSel = adcOvsRateSel2;
	init.lpfMode = adcLPFilterRC;
	init.warmUpMode = adcWarmupKeepADCWarm;
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(13000000, 0);
	init.tailgate = false;

	ADC_Init(ADC0, &init);

	// [ADC_InitSingle]
	ADC_InitSingle_TypeDef initsingle = ADC_INITSINGLE_DEFAULT;

	initsingle.prsSel = adcPRSSELCh0;
	initsingle.acqTime = adcAcqTime1;
	initsingle.reference = adcRef2V5;
	initsingle.resolution = adcRes12Bit;
	initsingle.input = adcSingleInputCh0;
	initsingle.diff = false;
	initsingle.prsEnable = false;
	initsingle.leftAdjust = false;
	initsingle.rep = false;

	/* Initialize a single sample conversion.
	 * To start a conversion, use ADC_Start().
	 * Conversion result can be read with ADC_DataSingleGet(). */
	ADC_InitSingle(ADC0, &initsingle);


		// [PRS initialization]
	/* Set up channel 0 */
//	PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_ADC0,PRS_CH_CTRL_SIGSEL_ADC0SINGLE, prsEdgeOff);

	adcConversionComplete = false;


}


void cap_meterStart(void)
{
	adc0CapMeterInit();
	tmr1CapMeterInit();
	adcSampleCount = 0;

	adcFinished = false;

	for(int i=0; i<128;i++) adcSampleBuff[i] = 0;

	TIMER_Enable(TIMER1, true);

	while(!tmr1Overflow);
	tmr1Overflow = false;

	while(!adcFinished){
		while(!tmr1Overflow);
		tmr1Overflow = false;
		GPIO_PinOutToggle(gpioPortF, 4);
		ADC0->CMD = (uint32_t)1u;
		/* Wait while conversion is active */
		while (ADC0->STATUS & ADC_STATUS_SINGLEACT);
		/* Get ADC result */
		//adcSampleBuff[adcSampleCount] = (uint16_t)ADC0->SINGLEDATA;// (uint16_t)ADC_DataSingleGet(ADC0);
		adcSampleBuff[0xFFFF - countTop] = (uint16_t)ADC0->SINGLEDATA;
		/*if(adcSampleCount > 128){
			adcFinished = true;
			TIMER_IntDisable(TIMER1, _TIMER_IF_MASK);
			TIMER_Enable(TIMER1, false);
		}*/

		if((0xFFFF - countTop) > 128){
					adcFinished = true;
					TIMER_IntDisable(TIMER1, _TIMER_IF_MASK);
					TIMER_Enable(TIMER1, false);
				}
	}

	for(int i=0; i<128;i++) printDbg("%d", adcSampleBuff[i]);
}
