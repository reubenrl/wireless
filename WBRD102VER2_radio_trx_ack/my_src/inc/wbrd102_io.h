/*
 * wbrd102_io.h
 *
 *  Created on: Jul 26, 2017
 *      Author: phytech
 */

#ifndef SRC_WBRD102_IO_H_
#define SRC_WBRD102_IO_H_

// PORT_A
#define GPIO_IO_RSRV_PORT				gpioPortA		// io reserve
#define GPIO_IO_RSRV0_PIN				0
#define GPIO_BUTTON_STATUS_PORT			gpioPortA		// button
#define GPIO_BUTTON_STATUS_PIN			1
//?
#define GPIO_TCXO_26MHZ_OUT_PIN			0				// radio crystal
#define GPIO_TCXO_26MHZ_IN_PIN			1

//?
#define GPIO_RF_RXP_PIN					1
#define GPIO_RF_RXN_PIN					2
#define GPIO_RF_TX_PIN					3

// PORT_B
#define GPIO_XTAL_32768HZ_P_PORT		gpioPortB		// rtc 32Khz crystal
#define GPIO_XTAL_32768HZ_P_PIN			7
#define GPIO_XTAL_32768HZ_N_PORT		gpioPortB
#define GPIO_XTAL_32768HZ_N_PIN			8
#define GPIO_XTAL_24MHZ_P_PORT			gpioPortB		// mcu crystal
#define GPIO_XTAL_24MHZ_P_PIN			13
#define GPIO_XTAL_24MHZ_N_PORT			gpioPortB
#define GPIO_XTAL_24MHZ_N_PIN			14

// PORT_C
#define GPIO_GPS_SYS_ON_RDY_PORT		gpioPortC		// gps system ready
#define GPIO_GPS_SYS_ON_RDY_PIN			8
#define GPIO_BATT_VTG_LEVEL_ENA_PORT	gpioPortC		// network resistor measure battery voltage enable
#define GPIO_BATT_VTG_LEVEL_ENA_PIN		9
#define GPIO_TCXO_PWR_ENA_PORT			gpioPortC		// tcxo radio power enable
#define GPIO_TCXO_PWR_ENA_PIN			10
#define GPIO_GPS_PWR_ENA_PORT			gpioPortC		// gps power enable
#define GPIO_GPS_PWR_ENA_PIN			14
#define GPIO_GPS_IGNITION_PORT			gpioPortC		// gps ignition enable
#define GPIO_GPS_IGNITION_PIN			15

// PORT_D
#define GPIO_GPS_RXD0_PORT				gpioPortD		// gps uart interface
#define GPIO_GPS_RXD0_PIN				4
#define GPIO_GPS_TXD0_PORT				gpioPortD		// gps uart interface
#define GPIO_GPS_TXD0_PIN				5
#define GPIO_SENS_PWR_ENA_PORT			gpioPortD		// sensor power enable
#define GPIO_SENS_PWR_ENA_PIN			6

// PORT_E
#define GPIO_BOOTLDR_TXD_PORT			gpioPortE		// monitor uart interface
#define GPIO_BOOTLDR_TXD_PIN			10
#define GPIO_BOOTLDR_RXD_PORT			gpioPortE		// monitor uart interface
#define GPIO_BOOTLDR_RXD_PIN			11
#define GPIO_SENS_DAT_IN_PORT			gpioPortE		// sensor data input - ADC_CH0
#define GPIO_SENS_DAT_IN_PIN			12
#define GPIO_BATT_VTG_LEVEL_PORT		gpioPortE		// measure battery voltage level - ADC_CH1
#define GPIO_BATT_VTG_LEVEL_PIN			13

// PORT_F
#define GPIO_DBG_SWCLK_PORT				gpioPortF		// jlink interface
#define GPIO_DBG_SWCLK_PIN				0
#define GPIO_DBG_SWDIO_PORT				gpioPortF		// jlink interface
#define GPIO_DBG_SWDIO_PIN				1
#define GPIO_MAIN_PWR_ON_OFF_PORT		gpioPortF		// main power enable
#define GPIO_MAIN_PWR_ON_OFF_PIN		2
#define GPIO_LED_1_PORT					gpioPortF		// led 1 enable
#define GPIO_LED_1_PIN					3
#define GPIO_LED_2_PORT					gpioPortF		// led 2 enable
#define GPIO_LED_2_PIN					4

#endif /* SRC_WBRD102_IO_H_ */
