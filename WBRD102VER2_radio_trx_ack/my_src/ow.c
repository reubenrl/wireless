/*
 * ow.c
 *
 *  Created on: Aug 22, 2017
 *      Author: phytech
 */
#include <stdio.h>

#include "em_timer.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_int.h"

#include "uart_dbg_print.h"
#include "ustimer.h"
#include "ow.h"


static bool owInitialized = false;

// Pause for exactly 'tick' number of ticks = 166nsec
void owTickDelay_nsec(uint16_t  owTick); // Implementation is platform specific
void owSetSpeed(uint16_t standard);

// 'tick' values
volatile static uint16_t A,B,C,D,E,F,G,H,I,J;


void owTickDelay_nsec(uint16_t  delay)
{
	TIMER_CounterSet(TIMER1, 0x00000000UL);
	uint32_t timerCounter = TIMER_CounterGet(TIMER1);
	timerCounter += (uint32_t)delay;
	while(timerCounter  > TIMER_CounterGet(TIMER1));
}


void owInit(void)
{
	if(owInitialized == true) return;

	/* Enable clock for GPIO module */
	CMU_ClockEnable(cmuClock_GPIO, true);
	/* Enable clock for TIMER1 module */
	CMU_ClockEnable(cmuClock_TIMER1, true);

	/* Configure one wire i/o */
	GPIO_PinModeSet(GPIO_OW_PORT, GPIO_OW_PIN, gpioModePushPull, 1);

	/* Select TIMER1 parameters */
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = true,
		.debugRun   = true,
		.prescale   = timerPrescale4,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = false,
	};
	TIMER_TopSet(TIMER1, TIMER_TOP_TOP_DEFAULT);
	/* Configure TIMER */
	TIMER_Init(TIMER1, &timerInit);

	// set the speed to 'standard'
	owSetSpeed(1);

	owInitialized = true;

}

//---------------------------------------------------------------------------
// Set the 1-Wire timing to 'standard' (standard=1) or 'overdrive' (standard=0).
//
void owSetSpeed(uint16_t standard)
{
	 // Adjust tick values depending on speed
	 if (standard){// Standard Speed
		 A = 6   * 6;
		 B = 64  * 6;
		 C = 60  * 6;
		 D = 10  * 6;
		 E = 9   * 6;
		 F = 55  * 6;
		 G = 0;
		 H = 480 * 6;
		 I = 70  * 6;
		 J = 410 * 6;
	 }else{ // Overdrive Speed
		 A = 1.5  * 6;
		 B = 7.5  * 6;
		 C = 7.5  * 6;
		 D = 2.5  * 6;
		 E = 0.75 * 6;
		 F = 7    * 0;
		 G = 2.5  * 6;
		 H = 70   * 6;
		 I = 8.5  * 6;
		 J = 40   * 6;
	  }
}

/*
 * Generate a 1-Wire reset.
 * return:
 * 		1 if no presence detect was found,
 * 		0 otherwise.
 * (NOTE: Does not handle alarm presence from DS2404/DS1994)
 */
uint8_t owTouchReset(void)
{
	uint8_t result;

	INT_Disable();
	owTickDelay_nsec(G);
	owOutp(0x00); // Drives DQ low
	owTickDelay_nsec(H);
	owOutp(0x01); // Releases the bus
	owTickDelay_nsec(I);
	result = owInp() & 0x01; // Sample for presence pulse from slave
	owTickDelay_nsec(J); // Complete the reset sequence recovery
	INT_Enable();
	return result; // Return sample presence pulse result
}

//---------------------------------------------------------------------------
// Send a 1-Wire write bit. Provide 10us recovery time.
//
void owWriteBit(uint8_t bit)
{
	INT_Disable();
 	if (bit){// Write '1' bit
		owOutp(0x00); // Drives DQ low
		owTickDelay_nsec(A);
		owOutp(0x01); // Releases the bus
		owTickDelay_nsec(B); // Complete the time slot and 10us recovery
	}else{// Write '0' bit
		owOutp(0x00); // Drives DQ low
		owTickDelay_nsec(C);
		owOutp(0x01); // Releases the bus
		owTickDelay_nsec(D);
	}
	INT_Enable();
}

//---------------------------------------------------------------------------
// Read a bit from the 1-Wire bus and return it. Provide 10us recovery time.
//
uint8_t owReadBit(void)
{
	uint8_t result;
	INT_Disable();
	owOutp(0x00); // Drives DQ low
	owTickDelay_nsec(A);
	owOutp(0x01); // Releases the bus
	owTickDelay_nsec(E);
	result = owInp() & 0x01; // Sample the bit value from the slave
	owTickDelay_nsec(F); // Complete the time slot and 10us recovery
	INT_Enable();
	return result;
}

//---------------------------------------------------------------------------
// Write 1-Wire data byte
//
void owWriteByte(uint8_t data)
{
	uint8_t loop;
	// Loop to write each bit in the byte, LS-bit first
	for (loop = 0; loop < 8; loop++){
		owWriteBit(data & 0x01);
		// shift the data byte for the next bit
		data >>= 1;
	}
}

//---------------------------------------------------------------------------
// Read 1-Wire data byte and return it
//
uint8_t owReadByte(void)
{
	uint8_t loop, result=0;

	for (loop = 0; loop < 8; loop++){
		// shift the result to get it ready for the next bit
		result >>= 1;
		// if result is one, then set MS bit
		if (owReadBit()) result |= 0x80;
	}

	return result;
}

//---------------------------------------------------------------------------
// Write a 1-Wire data byte and return the sampled result.
//
uint8_t owTouchByte(uint8_t data)
{
	uint8_t loop, result=0;

	for (loop = 0; loop < 8; loop++){
		// shift the result to get it ready for the next bit
		result >>= 1;
		// If sending a '1' then read a bit else write a '0'
		if (data & 0x01){
			if (owReadBit()) result |= 0x80;
		}else
			owWriteBit(0);
		// shift the data byte for the next bit
		data >>= 1;
	}

	return result;
}

//---------------------------------------------------------------------------
// Write a block 1-Wire data bytes and return the sampled result in the same
// buffer.
//
void owBlock(uint8_t *data, uint8_t data_len)
{
	uint8_t loop;

	for (loop = 0; loop < data_len; loop++){
		data[loop] = owTouchByte(data[loop]);
	}
}

//---------------------------------------------------------------------------
// Set all devices on 1-Wire to overdrive speed. Return '1' if at least one
// overdrive capable device is detected.
//
uint8_t owOverdriveSkip(uint8_t *data, uint8_t data_len)
{
	owInit();

	// set the speed to 'standard'
	owSetSpeed(1);
	// reset all devices
	if (owTouchReset()){ // Reset the 1-Wire bus
		return 0; // Return if no devices found
	}
	// overdrive skip command
	owWriteByte(0x3C);
	// set the speed to 'overdrive'
	owSetSpeed(0);
	// do a 1-Wire reset in 'overdrive' and return presence result
	return owTouchReset();
}

//---------------------------------------------------------------------------
// Read and return the page data and SHA-1 message authentication code (MAC)
// from a DS2432.
//
uint8_t owReadPageMAC(uint8_t page, uint8_t *page_data, uint8_t *mac)
{
	uint8_t i;
	unsigned short data_crc16, mac_crc16;

	owInit();
	// set the speed to 'standard'
	//owSetSpeed(1);
	// select the device
	if (owTouchReset()) // Reset the 1-Wire bus
		return 0; // Return if no devices found

	owWriteByte(0xCC); // Send Skip ROM command to select single device
	// read the page
	owWriteByte(0xA5); // Read Authentication command
	owWriteByte((page << 5) & 0xFF); // TA1
	owWriteByte(0); // TA2 (always zero for DS2432)
	// read the page data
	for (i = 0; i < 32; i++) page_data[i] = owReadByte();

	owWriteByte(0xFF);
	// read the CRC16 of command, address, and data
	data_crc16 = owReadByte();
	data_crc16 |= (owReadByte() << 8);
	// delay 2ms for the device MAC computation
	// read the MAC
	for (i = 0; i < 20; i++) mac[i] = owReadByte();

	// read CRC16 of the MAC
	mac_crc16 = owReadByte();
	mac_crc16 |= (owReadByte() << 8);

	// check CRC16...
	return 1;
}


/*
 * ds1820StartConv
 * Initiates a single temperature conversion,
 * the resulting thermal data is stored in the 2-byte temperature
 * register in the scratchpad memory.
 * return:
 * 		0 - no devices found
 * 		1 - otherwise
 */
uint8_t ds1820StartConv(void)
{
	if (owTouchReset()) // Reset the 1-Wire bus
		return 0; // Return if no devices found

	owWriteByte(0xCC);	//  send Skip ROM command.
	owWriteByte(0x44);	//  send Convert T command.

	USTIMER_Init();
	USTIMER_Delay(1000000u); // 1 second
	//msec_delay(1000u); // maximum adc convert time.
	USTIMER_DeInit();

	return 1;
}


/*
 * ds1820ReadScratchpad
 * The master read the contents of the scratchpad.
 * The data transfer starts with the least significant bit of byte 0
 *
 * p_buff_out	- output
 * buff_out_len	- input
 *
 * return:
 * 	0	- not device found
 * 	1	- device found
 */
uint8_t ds1820ReadScratchpad(uint8_t  *p_buff_out, uint8_t buff_out_len)
{
	uint8_t i;

	owInit();

	if (!ds1820StartConv()) return 0;

	if (owTouchReset()){ // Reset the 1-Wire bus
#ifdef __DS1820_DBG
		printDbg("ds1820_read_scratchpad(OWTouchReset fail - 1)");
#endif
		return 0; 		// Return if no devices found
	}
	owWriteByte(0xCC);	//  send Skip ROM command.
	owWriteByte(0xBE);	//  send Read Scratchpad command.

	for(i = 0; i < buff_out_len; i++){
		*p_buff_out++ = owReadByte();
	}

	if (owTouchReset()){// Reset the 1-Wire bus
#ifdef __DS1820_DBG
		printDbg("ds1820_read_scratchpad(OWTouchReset fail - 2)");
#endif
		return 0; // Return if no devices found
	}

	return 1;
}

/*
 -- FAMILYCODE + ID CODE + CRC
*/
// Polynomial: x^8 + x^5 + x^4 + 1 (0x8C)
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"

//unsigned char dallas_crc8( unsigned char buff_in[], unsigned char buff_in_len)
/*
unsigned char ds1820CRC8( unsigned char buff_in[], unsigned char buff_in_len)
{
	unsigned char i,j,inbyte,mix,crc=0;

	for(i=0; i<buff_in_len;i++){
		inbyte = buff_in[i];
		for (j=0;j<8;j++){
			mix = (crc ^ inbyte) & 0x01;
        	crc >>= 1;
         	if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}

	return crc;
}
*/

/* calculates 8-bit CRC of given data */
/* based on the polynomial  X^8 + X^5 + X^4 + 1 */
unsigned int owCalcCRC(unsigned char *buff, int len)
{
    unsigned int crc = 0,c;
    int i,j;

    for(j=0; j<len; j++){
        c = buff[j];
        for(i=0;i<8;i++) {
            if((crc ^ c) & 1){
            	crc = (crc>>1)^0x8C;
            }else{
            	crc >>= 1;
            }
            c >>= 1;
        }
    }

    return(crc);
}



/*
TEMPERATURE = TEMP_READ - 0.25 + (COUNT_PER_C - COUNT_REMAIN)/COUNT_PER_C

BYTE 0	TEMPERATURE LSB (AAh)
BYTE 1	TEMPERATURE MSB (00) (85°C)
BYTE 2 	TH REGISTER OR USER BYTE 1*
BYTE 3	TL REGISTER OR USER BYTE 2*
BYTE 4	RESERVED (FFh)
BYTE 5	RESERVED (FFh)
BYTE 6	COUNT REMAIN (0Ch)
BYTE 7	COUNT PER °C (10h)
BYTE 8 CRC*
*/
uint8_t ds1820GetTemperature( uint16_t *tempValuePtr)
{
	uint8_t strTmp[9];
	char bufTemp[64];

	if(!ds1820ReadScratchpad(strTmp, 9)) return 1;
	if(owCalcCRC(strTmp, 9)) return 1;


	for(char i=0;i<9;i++){
		sprintf(&bufTemp[i*2],"%02x", strTmp[i]);

	}
	printDbg("%s", bufTemp);

	uint16_t tr = (uint16_t)strTmp[1] << 8;
	tr |= (uint16_t)strTmp[0] & 0x00FF;

	// 0.5C resolution
	// T = Tr - 0.25 + (C-Cr)/C
	// T = ( (100*COUNT PER °tr) + 75*COUNT PER °C - 100*COUNT REMAIN )/100*COUNT PER °C
	uint32_t temp = (uint32_t)tr;
/*

	temp *= (uint32_t)strTmp[7];				// COUNT PER °C (10h)
	temp *= (uint32_t)100u;

	temp += (uint32_t)strTmp[7]*(uint32_t)75u;		// COUNT PER
	temp -= (uint32_t)strTmp[6]*(uint32_t)100u;		// COUNT REMAIN
	temp *= (uint32_t)10u;			// 1 decimal fraction
	temp /= (uint32_t)strTmp[7]*(uint32_t)100u;
*/
	temp *= 10u;
	temp /= 2;
	*tempValuePtr = (uint16_t)temp;

	printDbg("temp value:%0.1fC",(signed)temp/10.0);

	return 0;
}
