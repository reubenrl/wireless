/*
 * measure.c
 *
 *  Created on: Nov 23, 2016
 *      Author: phytech
 */

#include "em_cmu.h"
#include "em_emu.h"
#include "ustimer.h"
#include "em_adc.h"
#include "measure.h"

#define MEASURE_ANALOG_SENSOR

static volatile bool measureInitialized = false;

volatile bool adc0MeasureConversionComplete;

volatile bool tmr0MeasureInitialized = false;

#if defined(MEASURE_ANALOG_SENSOR)

/**************************************************************************//**
 * @brief ADC Interrupt handler (ADC0)
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  uint32_t flags;

  adc0MeasureConversionComplete = true;

  /* Clear interrupt flags */
  flags = ADC_IntGet( ADC0 );
  ADC_IntClear( ADC0, flags );

}
#endif

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

	adc0MeasureConversionComplete = false;

	/* Initialize a single sample conversion.
	 * To start a conversion, use ADC_Start().
	 * Conversion result can be read with ADC_DataSingleGet(). */
	ADC_InitSingle(ADC0, &initsingle);

		// [PRS initialization]
	/* Set up channel 0 */
//	PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_ADC0,PRS_CH_CTRL_SIGSEL_ADC0SINGLE, prsEdgeOff);

}



uint32_t adc0MeasureSingleSample(uint8_t  inputSelect)
{
	uint32_t adcValue, tmp;

	/* Enable interrupt on completed conversion */
	ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
	NVIC_ClearPendingIRQ( ADC0_IRQn );
	NVIC_EnableIRQ( ADC0_IRQn );

	tmp = ADC0->SINGLECTRL;
	tmp &= ~(0x0000000FUL << 8);
	tmp |= (uint32_t)(inputSelect << 8);
	ADC0->SINGLECTRL = tmp;

	adc0MeasureConversionComplete = false;
		ADC_Start(ADC0, adcStartSingle);
		while (!adc0MeasureConversionComplete){
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
	uint8_t i,j;

	/* Enable interrupt on completed conversion */
	ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
	NVIC_ClearPendingIRQ( ADC0_IRQn );
	NVIC_EnableIRQ( ADC0_IRQn );

	tmp = ADC0->SINGLECTRL;
	tmp &= ~(0x0000000FUL << 8);
	tmp |= (uint32_t)(inputSelect << 8);
	ADC0->SINGLECTRL = tmp;

	for(i=0;i<10;i++){
		adc0MeasureConversionComplete = false;
		ADC_Start(ADC0, adcStartSingle);
		while (!adc0MeasureConversionComplete){
			EMU_EnterEM1();
		}

		strTmp[i] = ADC_DataSingleGet(ADC0);
		USTIMER_Init();
		USTIMER_Delay(2000u); // 2ms
		USTIMER_DeInit();
	}

	NVIC_DisableIRQ( ADC0_IRQn );
	ADC_IntDisable(ADC0, ADC_IEN_SINGLE);


	for(i=2,j=0;j<8;i++,j++){
		adcValue += strTmp[i];
	}

	return (adcValue/8U);
}




void measureInit(void)
{
	if(tmr0MeasureInitialized == true) return;
	tmr0MeasureInitialized = true;

#if defined(WBRD102_VER2_RF_TRANSMITTER) || defined(WBRD102_VER2_RF_RECEIVER)
	/* Initial I/O power sensor enable */
	GPIO_PinModeSet(SENS_PWR_ENA_PORT, SENS_PWR_ENA_PIN, gpioModePushPull, 0);
	/* Initial I/O battery level enable */
	GPIO_PinModeSet(BATT_VTG_LEVEL_ENA_PORT, BATT_VTG_LEVEL_ENA_PIN, gpioModePushPull, 1);
#else
	GPIO_PinModeSet(BATT_VTG_LEVEL_ENA_PORT, BATT_VTG_LEVEL_ENA_PIN, gpioModePushPull, 0);
#endif

	adc0MeasureInit();

}


uint32_t adc0MeasureGet_mV(uint8_t  inputSelect)
{
	uint32_t value;

	value = adc0MeasureEightTimesSingleSample(inputSelect);
	value *= 2500u;	// voltage reference: 2500mV
	value /= 4096u;	// ADC-12Bit max scale: 0xFFF = 4096

	return (value); 	// mV = (voltage reference*ADC_raw)/ADC-12Bit max scale = 2500*ADC/4096
}

uint32_t measureBatteryVoltage_mV(void)
{
	uint32_t value;

	if(!measureInitialized){
		measureInit();
	}

	BATT_VTG_LEVEL_ENA();
	USTIMER_Init();
	/* Delay for 100000us = 100ms time */
	USTIMER_Delay( 100000u );
	/* Deinit ustimer */
	USTIMER_DeInit();

	value = adc0MeasureGet_mV(MEASURE_SELECT_BATT_CHANNEL);
	BATT_VTG_LEVEL_DIS();

	value *= 2u;
	return (value);
}

uint32_t measureSensor_mV(void)
{
	uint32_t value;

	if(!measureInitialized){
		measureInit();
	}

	SENS_PWR_ENA_SET();
	USTIMER_Init();
	/* Delay for 200000us = 200ms time */
	USTIMER_Delay( 200000u );
	/* Deinit ustimer */
	USTIMER_DeInit();
	value = adc0MeasureGet_mV(MEASURE_SELECT_SEN_CHANNEL);
	SENS_PWR_ENA_CLR();

	return (value);
}

#define CALCULATE_ADC_MV
uint32_t measureDendrometerSensor_um(void)
{
	/*
	 * X = 5*mV - 1250 [um]
	 */
	uint32_t Value;

	Value = measureSensor_mV();
#if defined(CALCULATE_ADC_MV)
	Value *= 5u;
	Value -= 1250u;
#else
	Value *= 2u;	// network resistor 1/2
	Value *= 5u;
	Value -= 2500u;
	Value /=2u;
#endif

	return (Value);
}

/*

uint32_t measure(t_eMeasureType eMeasureType)
{
	uint32_t value = 0;

	if(eMeasureType == e_dendrometer){
		value = measureDendrometerSensor_um();
	}else if(eMeasureType == e_battery){
		value = measureBatteryVoltage_mV();
	}

	return (value);
}
*/
