/*
 * ezr_temperature.c
 *
 *  Created on: Apr 7, 2017
 *      Author: phytech
 */

#include "bsp.h"
#include "em_device.h"
#include "em_system.h"
#include "em_emu.h"
#include "em_adc.h"
#include "measure.h"


void ezr_temperatureInit(void)
{
//	adc0Init();

	// [ADC_InitSingle]
	ADC_InitSingle_TypeDef initsingle = ADC_INITSINGLE_DEFAULT;

	initsingle.prsSel = adcPRSSELCh0;
	initsingle.acqTime = adcAcqTime32;
	initsingle.reference = adcRef1V25;
	initsingle.resolution = adcRes12Bit;
	initsingle.input = adcSingleInpTemp;
	initsingle.diff = 0;
	initsingle.prsEnable = 0;
	initsingle.leftAdjust = 0;
	initsingle.rep = 0;

	/* Initialize a single sample conversion.
	 * To start a conversion, use ADC_Start().
	 * Conversion result can be read with ADC_DataSingleGet().
	 * */

	ADC_InitSingle(ADC0, &initsingle);

//	adc0Initialized = false;

}



/**************************************************************************//**
 * @brief Convert ADC sample values to celsius.
 * @detail See section 25.3.4.1 in the reference manual for detail on
 *   temperature measurement and conversion.
 * @param adcSample Raw value from ADC to be converted to celsius
 * @return The temperature in degrees celsius.
 *****************************************************************************/
float ConvertToCelsius(int32_t adcSample)
{
  uint32_t calTemp0;
  uint32_t calValue0;
  int32_t readDiff;
  float temp;

  /* Factory calibration temperature from device information page. */
  calTemp0 = ((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >> _DEVINFO_CAL_TEMP_SHIFT);

  calValue0 = ((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK) >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

  if ((calTemp0 == 0xFF) || (calValue0 == 0xFFF))
  {
    /* The temperature sensor is not calibrated */
    return -100.0;
  }

  /* Vref = 1250mV
     TGRAD_ADCTH = 1.92 mV/degC (from datasheet)
  */
  readDiff = calValue0 - adcSample;
  temp     = ((float)readDiff * 1500);
  temp    /= (4096 * -1.92);

  /* Calculate offset from calibration temperature */
  temp     = (float)calTemp0 - temp;
  return temp;
}


/***************************************************************************//**
 * @brief
 *    Get the current temperature.
 *
 * @return
 *    Current temperature in degrees Celsius.
 ******************************************************************************/
float ezr_temperatureGetTemp(void)
{
	uint32_t	adcValue;

  ezr_temperatureInit();

  /* Enable interrupt on completed conversion */
  	ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
  	NVIC_ClearPendingIRQ( ADC0_IRQn );
  	NVIC_EnableIRQ( ADC0_IRQn );

  	adc0MeasureConversionComplete = false;
	ADC_Start(ADC0, adcStartSingle);
	while (!adc0MeasureConversionComplete){
		EMU_EnterEM1();
	}
	adcValue = ADC_DataSingleGet(ADC0);

	NVIC_DisableIRQ( ADC0_IRQn );
	ADC_IntDisable(ADC0, ADC_IEN_SINGLE);

	return (ConvertToCelsius(adcValue));
}

