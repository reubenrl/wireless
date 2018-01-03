/*
 * measure.h
 *
 *  Created on: Nov 23, 2016
 *      Author: phytech
 */

#ifndef MY_SRC_INC_MEASURE_H_
#define MY_SRC_INC_MEASURE_H_

#include <stdbool.h>
#include "em_gpio.h"
#include "bspconfig.h"




extern volatile bool adc0MeasureConversionComplete;
extern volatile bool adc0Initialized;

#if defined(WBRD100_VER1)

/*
typedef enum{
	e_nothing = 0,
	e_dendrometer,
	e_battery
}t_eMeasureType;*/

#define SENS_PWR_ENA_PORT			GPIO_SENS_PWR_ENA_PORT
#define SENS_PWR_ENA_PIN			GPIO_SENS_PWR_ENA_PIN

#define BATT_VTG_LEVEL_ENA_PORT		GPIO_BATT_VTG_LEVEL_ENA_PORT
#define BATT_VTG_LEVEL_ENA_PIN		GPIO_BATT_VTG_LEVEL_ENA_PIN

/* Battery level enable */
#define BATT_VTG_LEVEL_ENA()		GPIO_PinOutClear(BATT_VTG_LEVEL_ENA_PORT, BATT_VTG_LEVEL_ENA_PIN)
/* Battery level disable */
#define BATT_VTG_LEVEL_DIS()		GPIO_PinOutSet(BATT_VTG_LEVEL_ENA_PORT, BATT_VTG_LEVEL_ENA_PIN)


#elif defined(WBRD102_VER2_RF_TRANSMITTER) || defined(WBRD102_VER2_RF_RECEIVER)
/*
 * Pin PD6 is configured to Push-pull
 */
#define SENS_PWR_ENA_PORT			GPIO_SENS_PWR_ENA_PORT
#define SENS_PWR_ENA_PIN			GPIO_SENS_PWR_ENA_PIN


/*
 * Pin PC9 is configured to Push-pull
 */
#define BATT_VTG_LEVEL_ENA_PORT		GPIO_BATT_VTG_LEVEL_ENA_PORT
#define BATT_VTG_LEVEL_ENA_PIN		GPIO_BATT_VTG_LEVEL_ENA_PIN

/* Battery level enable */
#define BATT_VTG_LEVEL_ENA()			GPIO_PinOutClear(BATT_VTG_LEVEL_ENA_PORT, BATT_VTG_LEVEL_ENA_PIN)
/* Battery level disable */
#define BATT_VTG_LEVEL_DIS()			GPIO_PinOutSet(BATT_VTG_LEVEL_ENA_PORT, BATT_VTG_LEVEL_ENA_PIN)


#endif


/* Power sensor  ON */
#define SENS_PWR_ENA_SET()					GPIO_PinOutSet(SENS_PWR_ENA_PORT, SENS_PWR_ENA_PIN)
/* Power sensor OFF */
#define SENS_PWR_ENA_CLR()					GPIO_PinOutClear(SENS_PWR_ENA_PORT, SENS_PWR_ENA_PIN)

/* ADC0 Channel Select */
#define MEASURE_SELECT_SEN_CHANNEL			0
#define MEASURE_SELECT_BATT_CHANNEL			1


uint32_t measureBatteryVoltage_mV(void);
uint32_t measureSensor_mV(void);
uint32_t measureDendrometerSensor_um(void);

#endif /* MY_SRC_INC_MEASURE_H_ */
