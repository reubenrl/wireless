/*
 * adc0.c
 *
 *  Created on: Nov 23, 2016
 *      Author: phytech
 */
#include "em_emu.h"
#include "em_cmu.h"
#include "em_prs.h"
#include "em_adc.h"

#include "ustimer.h"
#include "adc0.h"

#include "uart_dbg_print.h"

volatile bool adcConversionComplete;

volatile bool adc0Initialized = false;


/**************************************************************************//**
 * @brief ADC Interrupt handler (ADC0)
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  uint32_t flags;

  adcConversionComplete = true;

  /* Clear interrupt flags */
  flags = ADC_IntGet( ADC0 );
  ADC_IntClear( ADC0, flags );

}

void adc0MeasureInit(void)
{
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

	/* Enable clock for ADC0 */
	CMU_ClockEnable(cmuClock_ADC0, true);

	init.ovsRateSel = adcOvsRateSel2;
	init.lpfMode = adcLPFilterRC;
	init.warmUpMode = adcWarmupNormal;
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(70000, 0);
	init.tailgate = 0;

	ADC_Init(ADC0, &init);

	// [ADC_InitSingle]
	ADC_InitSingle_TypeDef initsingle = ADC_INITSINGLE_DEFAULT;

	initsingle.prsSel = adcPRSSELCh0;
	initsingle.acqTime = adcAcqTime32;
	initsingle.reference = adcRef2V5;
	initsingle.resolution = adcRes12Bit;
	initsingle.input = adcSingleInpCh0;
	initsingle.diff = 0;
	initsingle.prsEnable = 0;
	initsingle.leftAdjust = 0;
	initsingle.rep = 0;

	adcConversionComplete = false;

	/* Initialize a single sample conversion.
	 * To start a conversion, use ADC_Start().
	 * Conversion result can be read with ADC_DataSingleGet(). */
	ADC_InitSingle(ADC0, &initsingle);

		// [PRS initialization]
	/* Set up channel 0 */
	PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_ADC0,PRS_CH_CTRL_SIGSEL_ADC0SINGLE, prsEdgeOff);

	adc0Initialized = true;

}



uint32_t adc0MeasureSingleSample(uint8_t  inputSelect)
{
	uint32_t adcValue, tmp;

	/* Enable interrupt on completed conversion */
	ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
	NVIC_ClearPendingIRQ( ADC0_IRQn );
	NVIC_EnableIRQ( ADC0_IRQn );

	tmp = ADC0->SINGLECTRL;
	tmp &= ~(0x0F << 8);
	tmp |= (uint32_t)(inputSelect << 8);
	ADC0->SINGLECTRL = tmp;

		adcConversionComplete = false;
		ADC_Start(ADC0, adcStartSingle);
		while (!adcConversionComplete){
			EMU_EnterEM1();
		}

	adcValue = ADC_DataSingleGet(ADC0);

	NVIC_DisableIRQ( ADC0_IRQn );
	ADC_IntDisable(ADC0, ADC_IEN_SINGLE);

	return adcValue;
}


/*
 * inputSelect	- input
 * return:
 * 		adcValue
 */
uint32_t adc0MeasureEightTimesSingleSample(uint8_t  inputSelect)
{
	uint32_t adcValue, tmp;
	uint32_t strTmp[10];
	uint8_t i;

	/* Enable interrupt on completed conversion */
	ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
	NVIC_ClearPendingIRQ( ADC0_IRQn );
	NVIC_EnableIRQ( ADC0_IRQn );

	tmp = ADC0->SINGLECTRL;
	tmp &= ~(0x0F << 8);
	tmp |= (uint32_t)(inputSelect << 8);
	ADC0->SINGLECTRL = tmp;

	for(i=0;i<10;i++){
		adcConversionComplete = false;
		ADC_Start(ADC0, adcStartSingle);
		while (!adcConversionComplete){
			EMU_EnterEM1();
		}

		strTmp[i] = ADC_DataSingleGet(ADC0);
		USTIMER_Init();
		USTIMER_Delay(2000u); // 2ms
		USTIMER_DeInit();
	}

	NVIC_DisableIRQ( ADC0_IRQn );
	ADC_IntDisable(ADC0, ADC_IEN_SINGLE);


	for(i=2;i<10;i++){
		adcValue += strTmp[i];
	}

	return (adcValue/8);
}


uint32_t adc0Get_mV(uint8_t  inputSelect)
{
	uint32_t value;

	if(!adc0Initialized){
		adc0Init();
	}

	value = adc0MeasureEightTimesSingleSample(inputSelect);
	value *= 2500u;	// voltage reference: 2500mV
	value /= 4095u;	// ADC-12Bit max scale: 0xFFF = 4095

	return (value); 	// mV = (voltage reference*ADC_raw)/ADC-12Bit max scale = 2500*ADC/4095
}
