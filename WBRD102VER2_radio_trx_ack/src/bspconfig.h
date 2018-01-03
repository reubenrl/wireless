/***************************************************************************//**
 * @file
 * @brief Provide BSP (board support package) configuration parameters.
 * @version 4.4.0
 *******************************************************************************
 * @section License
 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#ifndef __SILICON_LABS_BSPCONFIG_H__
#define __SILICON_LABS_BSPCONFIG_H__


/* Standalone definitions */
#define WBRD102_VER2		// Sensor production card

#define WBRD102_VER2_RF_TRANSMITTER
//#define WBRD102_VER2_RF_RECEIVER

#if defined(WBRD102_VER2_RF_TRANSMITTER) & defined(WBRD102_VER2_RF_RECEIVER)
#error "not defined receiver and transmitter together..."
#endif

#include "wbrd102_io.h"

#define LED1			0
#define LED2			1

#if defined(WBRD102_VER2_RF_TRANSMITTER) || defined(WBRD102_VER2_RF_RECEIVER)

//#define BSP_GPS_MODULE

#define BSP_GPIO_LEDS
#define BSP_NO_OF_LEDS  2

#define BSP_GPIO_LEDARRAY_INIT {{GPIO_LED_1_PORT,GPIO_LED_1_PIN}, {GPIO_LED_2_PORT,GPIO_LED_2_PIN}}

#define BSP_GPIO_BUTTONARRAY
#define BSP_NO_OF_BUTTONS 1
#define BSP_GPIO_PB0_PORT 	GPIO_BUTTON_STATUS_PORT
#define BSP_GPIO_PB0_PIN  	GPIO_BUTTON_STATUS_PIN
#define BSP_GPIO_BUTTONARRAY_INIT {{GPIO_BUTTON_STATUS_PORT, GPIO_BUTTON_STATUS_PIN}}

#elif defined(WBRD100_VER1)

#define SENSOR_TRANSMITTER

#define BSP_GPIO_LEDS
#define BSP_NO_OF_LEDS  2

#define BSP_GPIO_LEDARRAY_INIT {{gpioPortF,4},{gpioPortF,3}}

#define BSP_GPIO_BUTTONARRAY
#define BSP_NO_OF_BUTTONS 1
#define BSP_GPIO_PB0_PORT gpioPortF
#define BSP_GPIO_PB0_PIN  2
#define BSP_GPIO_BUTTONARRAY_INIT {{BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN}}

/* TCXO Power Supply Enable */
#define GPIO_TCXO_PORT 					gpioPortC
#define GPIO_TCXO_PIN  					10



/* GPS Power Supply Enable */
#define GPIO_GPS_PWR_ENA_PORT			gpioPortC
#define GPIO_GPS_PWR_ENA_PIN			14

#define GPIO_GPS_IGNITION_PORT			gpioPortC
#define GPIO_GPS_IGNITION_PIN			15

#define GPIO_GPS_SYS_ON_RDY_PORT		gpioPortC
#define GPIO_GPS_SYS_ON_RDY_PIN			8


/* SENSOR Power Supply Enable */
#define GPIO_SENS_PWR_ENA_PORT			gpioPortD
#define GPIO_SENS_PWR_ENA_PIN			6

/* ADC Input For Sensor Analog Data */
#define GPIO_SENS_DAT_IN_PORT			gpioPortE
#define GPIO_SENS_DAT_IN_PIN			12

/* Battery Voltage Enable */
#define GPIO_BATT_VTG_LEVEL_ENA_PORT	gpioPortC
#define GPIO_BATT_VTG_LEVEL_ENA_PIN		9

/* ADC Input For Battery Voltage Analog Data */
#define GPIO_BATT_VTG_LEVEL_PORT		gpioPortE
#define GPIO_BATT_VTG_LEVEL_PIN			13

#define GPIO_MAIN_POWER_PORT			gpioPortF
#define GPIO_MAIN_POWER_PIN				2


#elif defined(WBRD200_VER1)

#define BSP_GPIO_LEDS
#define BSP_NO_OF_LEDS  1
#define BSP_GPIO_LEDARRAY_INIT {gpioPortF,3}

/* TCXO Power Supply Enable */
#define GPIO_TCXO_PORT 					gpioPortF
#define GPIO_TCXO_PIN  					2

/* Send Buffer Data To Logger */
#define GPIO_RADIO_CTS_PORT				gpioPortD
#define GPIO_RADIO_CTS_PIN				6

#else


#define BSP_STK
#define BSP_NET
#define BSP_WSTK
#define BSP_WSTK_BRD4001    // WSTK mainboard
#define BSP_WSTK_BRD4545A   // MCU/Radio plug-in board

#define BSP_BCC_LEUART      LEUART0
#define BSP_BCC_CLK         cmuClock_LEUART0
#define BSP_BCC_LOCATION    LEUART_ROUTE_LOCATION_LOC0
#define BSP_BCC_TXPORT      gpioPortD
#define BSP_BCC_TXPIN       4
#define BSP_BCC_RXPORT      gpioPortD
#define BSP_BCC_RXPIN       5
#define BSP_BCC_ENABLE_PORT gpioPortC
#define BSP_BCC_ENABLE_PIN  8

#define BSP_DISP_ENABLE_PORT  gpioPortA
#define BSP_DISP_ENABLE_PIN   1                   /* MemLCD display enable */

#define BSP_GPIO_LEDS
#define BSP_NO_OF_LEDS  2
#define BSP_GPIO_LEDARRAY_INIT {{gpioPortF,4},{gpioPortF,2}}

#define BSP_GPIO_BUTTONARRAY
#define BSP_NO_OF_BUTTONS 2
#define BSP_GPIO_PB0_PORT gpioPortC
#define BSP_GPIO_PB0_PIN  9
#define BSP_GPIO_PB1_PORT gpioPortC
#define BSP_GPIO_PB1_PIN  10

#define BSP_GPIO_BUTTONARRAY_INIT {{BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN}, \
                                   {BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN}}
#endif

#define BSP_INIT_DEFAULT  0

#define BSP_BCP_VERSION 2

#include "bsp_bcp.h"

#endif
