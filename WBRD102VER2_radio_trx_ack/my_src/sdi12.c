/*
 * sdi12.c
 *
 *  Created on: Dec 11, 2017
 *      Author: phytech
 */
#include <stdint.h>
#include <stdbool.h>
#include "em_gpio.h"
#include "em_device.h"
#include "rtcdriver.h"

#include "bsp.h"
#include "uartsw.h"
#include "uart_dbg_print.h"
#include "sdi12.h"

volatile static bool sdiInitialized = false;
static uint16_t sdiRxTimeout_100ms;
static uint16_t sdiTickDelay_100ms;

static uint8_t sdi12RxBuffer[80];

static RTCDRV_TimerID_t	sdi12RtcTimerId;


__STATIC_INLINE  void sdi12RxDelay100ms(uint16_t  delay100ms)
{
	sdiTickDelay_100ms = delay100ms;
	while(sdiTickDelay_100ms);
}

__STATIC_INLINE  uint8_t uartswParityBitGenerator(uint8_t  data)
{
	data ^= data >> 4;
	data ^= data >> 2;
	data ^= data >> 1;

	return (~data & 1);
}

void sdi12SensorPwrOn(void)
{
	// SDI12 power on sensor
	GPIO_PinOutSet(GPIO_SENS_PWR_ENA_PORT, GPIO_SENS_PWR_ENA_PIN);
	sdi12RxDelay100ms(15*10); // wait 1.5 seconds.
}

void sdi12SensorPwrOff(void)
{
	// SDI12 power off sensor
	GPIO_PinOutClear(GPIO_SENS_PWR_ENA_PORT, GPIO_SENS_PWR_ENA_PIN);
}


void SDI12_100MS_TICKS_IRQHandler()
{
	if(sdiRxTimeout_100ms)  sdiRxTimeout_100ms--;
	if(sdiTickDelay_100ms) sdiTickDelay_100ms--;
}


void sdi12Init(void)
{
	if( sdiInitialized == true) return;
	sdiInitialized = true;

	RTCDRV_Init();
	if (ECODE_EMDRV_RTCDRV_OK != RTCDRV_AllocateTimer( &sdi12RtcTimerId) ){
		while (1);
	}
	if (ECODE_EMDRV_RTCDRV_OK != RTCDRV_StartTimer(sdi12RtcTimerId, rtcdrvTimerTypePeriodic, 100UL,
						 (RTCDRV_Callback_t)SDI12_100MS_TICKS_IRQHandler, NULL ) ){
		while (1);
	}

	uartswInit();

}

/*
 *  uint8_t sdi12GetData(uint8_t *pData)
 *  Description:
 *  	*pData - pointer to return data
 *  return:
 *  	false 	- not data
 *  	true 	- there is data
 *
 */
bool sdi12GetData(uint8_t *pData)
{

	uint8_t c, checkParityBit;

	if(uartswReceiveByte(&c)){
		checkParityBit = (c & 0x80) ? 1: 0;
		c &= 0x7F;
		if(!uartswParityBitGenerator(c) == checkParityBit){
			*pData = c;
			return true;
		}
	}

	return false;
}

/*
 * in:
 * 	timeout_100ms - multiple 100 miliseconds.
 * return:
 * 		NULL 	- empty
 * 		another	- successful
 */
uint8_t * sdi12GetDataBuffer(uint16_t timeout_100ms)
{
	uint8_t c, *pTmp;

	pTmp = &sdi12RxBuffer[0];
	sdiRxTimeout_100ms = timeout_100ms;
	do{
		if(sdi12GetData(&c)){
			sdi12RxDelay100ms(4);	// wait 400ms => 50 bytes @ 1200 baudrate
			*pTmp = c;
			pTmp++;
			while(sdi12GetData(&c)){
				*pTmp = c;
				pTmp++;
			}
			break;
		}

	}while(sdiRxTimeout_100ms);

	if(sdiRxTimeout_100ms){
		*pTmp = '\0';
		return &sdi12RxBuffer[0];
	}

	return NULL;
}


void sdi12TxData(uint8_t data)
{
	//sdi12Init();
	/// parity bit generator.
	data &= 0x7F;
	if(!uartswParityBitGenerator(data))	data |= (1 << 7);

	uartswSendByte(data);
}


/*
 * in:
 * 		strCmd[] 	- sdi12 command, string null character '\0' (Null-terminated string)
 * 		strResp[] 	- sdi12 response to command, string null character.
 * 		timeout_sec	- response timeout in seconds.
 *
 * return:
 * 		NULL 	- fail
 * 		Other 	- return pointer to receiver data buffer.
 */
char * sdi12Cmd(char strCmd[], char strResp[], uint16_t timeout_sec)
{
	int buffLen, i;
	char *ret;

	printDbg(strCmd);

	uartswFlushReceiveBuffer();

	buffLen = strlen(strCmd);
	if(buffLen){
		for(i=0;i<buffLen;i++) sdi12TxData(strCmd[i]);
	}else
		return NULL;

	if(timeout_sec){

		ret = (char *)sdi12GetDataBuffer(timeout_sec*10U);
		if(ret != NULL){
			if(strstr(ret, strResp) != NULL) return ret;
		}
	}

	return NULL;
}
