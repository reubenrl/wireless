/*
 * ow.h
 *
 *  Created on: Aug 22, 2017
 *      Author: phytech
 */

#ifndef MY_SRC_INC_OW_H_
#define MY_SRC_INC_OW_H_

#include "bsp.h"


typedef union{
	int  word;
	struct{
		unsigned char lsb;
		unsigned char msb;
	}byte;
}t_temperature;

typedef union{
	unsigned int word;
	union{
		unsigned char high;
		unsigned char low;
	}byte;
}t_alarm_trigger;


typedef struct
{
	t_temperature	temperature;
	t_alarm_trigger alarm_trigger;
	unsigned char 	reserva0;
	unsigned char 	reserva1;
	unsigned char 	count_remain;
	unsigned char 	count_per_c;
	unsigned char	crc8;
	unsigned char	scratchpad_error;
}t_scScratchpad, *t_pscScratchpad;



#define GPIO_OW_PORT	GPIO_SENS_DAT_IN_PORT
#define GPIO_OW_PIN		GPIO_SENS_DAT_IN_PIN

#define OW_PIN_SET_DIR_OUTPUT		0
#define OW_PIN_SET_DIR_INPUT		1


#define owOutp(value)	(value) ? (GPIO_PinOutSet(GPIO_OW_PORT, GPIO_OW_PIN)) : (GPIO_PinOutClear(GPIO_OW_PORT, GPIO_OW_PIN))
#define owInp()	GPIO_PinInGet(GPIO_OW_PORT, GPIO_OW_PIN)

#define owPinSetDir(dir)	(dir) ? GPIO_PinModeSet(GPIO_OW_PORT, GPIO_OW_PIN, gpioModeInput, 1) : GPIO_PinModeSet(GPIO_OW_PORT, GPIO_OW_PIN, gpioModePushPull, 1)

uint8_t ds1820GetTemperature(uint16_t *tempValuePtr);


uint8_t ds1820ReadScratchpad(uint8_t  *p_buff_out, uint8_t buff_out_len);

//---------------------------------------------------------------------------
// Write 1-Wire data byte
//
void owWriteByte(uint8_t data);
//---------------------------------------------------------------------------
// Read 1-Wire data byte and return it
//
uint8_t owReadByte(void);
//---------------------------------------------------------------------------
// Write a 1-Wire data byte and return the sampled result.
//
uint8_t owTouchByte(uint8_t data);
//---------------------------------------------------------------------------
// Write a block 1-Wire data bytes and return the sampled result in the same
// buffer.
//
void owBlock(uint8_t *data, uint8_t data_len);
//---------------------------------------------------------------------------
// Set all devices on 1-Wire to overdrive speed. Return '1' if at least one
// overdrive capable device is detected.
//
uint8_t owOverdriveSkip(uint8_t *data, uint8_t data_len);
//---------------------------------------------------------------------------
// Read and return the page data and SHA-1 message authentication code (MAC)
// from a DS2432.
//
uint8_t owReadPageMAC(uint8_t page, uint8_t *page_data, uint8_t *mac);
#endif /* MY_SRC_INC_OW_H_ */
