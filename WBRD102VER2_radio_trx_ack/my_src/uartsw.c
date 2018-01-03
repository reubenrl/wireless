/*
 * uartsw.c
 *
 *  Created on: Dec 11, 2017
 *      Author: phytech
 */

#include "em_timer.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_cmu.h"

#include "gpiointerrupt.h"
#include "bsp.h"

#include "uartsw.h"

#define TIMER_IF_CHNL_TXD 	TIMER_IF_CC0
#define TIMER_CHNL_TXD		0
#define TIMER_IF_CHNL_RXD 	TIMER_IF_CC1
#define TIMER_CHNL_RXD		1

typedef enum{
	E_UARTSW_STATE_IDLE,
	E_UARTSW_STATE_RXD,
	E_UARTSW_STATE_TXD
}t_enumUartswState;

// Global variables
static t_enumUartswState enumUartswState;
volatile bool uartswInitialized = false;
volatile bool uartswRxTxEnable = false;

// uartsw transmit status and data variables
static volatile uint8_t UartswTxBusy;
static volatile uint8_t UartswTxData;
static volatile uint8_t UartswTxBitNum;

// baud rate common to transmit and receive
static  uint32_t UartswBaudRateDiv;

// uartsw receive status and data variables
static volatile uint8_t UartswRxBusy;
static volatile uint8_t UartswRxData;
static volatile uint8_t UartswRxBitNum;
// uartsw receive buffer
static cBuffer uartswRxBuffer;
// automatically allocate space in ram for each buffer
static uint8_t uartswRxData[UARTSW_RX_BUFFER_SIZE];


// functions
 static void GPIO_UARTSW_RX_IRQHandler( uint8_t pin )
 {
   (void)pin;
   GPIO_PinOutToggle(GPIO_LED_1_PORT,GPIO_LED_1_PIN);
   uartswRxBitService();
 }

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER1_IRQHandler(void)
{
	/* Clear flag for TIMER0 overflow interrupt */
	uint32_t flags = TIMER_IntGet(TIMER1);
	TIMER_IntClear(TIMER1, flags);

	switch(enumUartswState){
	case E_UARTSW_STATE_RXD:
		GPIO_PinOutToggle(GPIO_LED_2_PORT,GPIO_LED_2_PIN);
		uartswRxBitService();
		break;
	case E_UARTSW_STATE_TXD:
		uartswTxBitService();
		break;
	default:
		break;
	}

}



//! enable and initialize the software uart
void uartswInit(void)
{
	if(uartswInitialized == true) return;
	uartswInitialized = true;

	enumUartswState = E_UARTSW_STATE_IDLE;

	// initialize the buffers
	uartswInitBuffers();

	/* Enable clock for GPIO module */
	CMU_ClockEnable(cmuClock_GPIO, true);
	// initialize the ports
	GPIO_PinModeSet(UARTSW_RX_PORT, UARTSW_RX_PIN, gpioModeInput, 0);

	/* Enable clock for TIMER1 module */
	CMU_ClockEnable(cmuClock_TIMER1, true);

	/* Select TIMER1 parameters */
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = true,
		.debugRun   = true,
		.prescale   = _TIMER_CTRL_PRESC_DIV1,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = false,
	};

	// initialize baud rate
	uartswSetBaudRate(1200U);

	NVIC_GetPendingIRQ(TIMER1_IRQn);
	/* Enable TIMER1 interrupt vector in NVIC */
	NVIC_EnableIRQ(TIMER1_IRQn);
	/* Configure TIMER */
	TIMER_Init(TIMER1, &timerInit);

	// setup the transmitter
	UartswTxBusy = false;

	// setup the receiver
	UartswRxBusy = false;

	#ifdef UARTSW_INVERT
	// trigger on rising edge
	GPIO_IntConfig(UARTSW_RX_PORT, UARTSW_RX_PIN, true, false, true);
	#else
	// trigger on falling edge
	GPIO_IntConfig(UARTSW_RX_PORT, UARTSW_RX_PIN, false, true, true);
	#endif
	// enable interrupt
	GPIOINT_CallbackRegister( UARTSW_RX_PIN, GPIO_UARTSW_RX_IRQHandler );

}

//! create and initialize the uart buffers
void uartswInitBuffers(void)
{
	// initialize the UART receive buffer
	bufferInit(&uartswRxBuffer, uartswRxData, UARTSW_RX_BUFFER_SIZE);
}

//! turns off software UART
void uartswOff(void)
{
	// disable interrupts
	TIMER_IntDisable(TIMER1, TIMER_IF_OF);
	TIMER_IntDisable(TIMER1, TIMER_IF_CHNL_TXD);
	TIMER_IntDisable(TIMER1, TIMER_IF_CHNL_RXD);

	NVIC_DisableIRQ(TIMER1_IRQn);

	GPIO_IntConfig(UARTSW_RX_PORT, UARTSW_RX_PIN, false, false, false);

	uartswInitialized = false;

}

void uartswSetBaudRate(uint32_t baudrate)
{
	// set timer prescaler
	// calculate division factor for requested baud rate, and set it
	UartswBaudRateDiv = (uint32_t)((24000000UL+(baudrate/2L))/(baudrate*1L));
}

//! returns the receive buffer structure
cBuffer* uartswGetRxBuffer(void)
{
	// return rx buffer pointer
	return &uartswRxBuffer;
}

void uartswSendByte(uint8_t data)
{

	// wait until uart is ready
	while(UartswTxBusy);
	// set busy flag
	UartswTxBusy = true;

	// save data
	UartswTxData = data;
	// set number of bits (+1 for stop bit)
	UartswTxBitNum = 9;

	enumUartswState = E_UARTSW_STATE_TXD;
	/// disable start bit interrupt - uart receiver
	GPIO_IntDisable(1 << UARTSW_RX_PIN);

	// set the start bit
	#ifdef UARTSW_INVERT
	// config as output
	GPIO_PinModeSet(UARTSW_TX_PORT, UARTSW_TX_PIN, gpioModePushPull, 1);
	#else
	// config as output
	GPIO_PinModeSet(UARTSW_TX_PORT, UARTSW_TX_PIN, gpioModePushPull, 0);
	#endif

	// schedule the next bit
	TIMER_CounterSet(TIMER1, 0UL);
	TIMER_TopSet(TIMER1, UartswBaudRateDiv);

	// enable timer overflow interrupt
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
	TIMER_IntEnable(TIMER1, TIMER_IF_OF);
}

//! gets a byte (if available) from the uart receive buffer
uint8_t uartswReceiveByte(uint8_t* rxData)
{
	// make sure we have a receive buffer
	if(uartswRxBuffer.size){
		// make sure we have data
		if(uartswRxBuffer.datalength){
			// get byte from beginning of buffer
			*rxData = bufferGetFromFront(&uartswRxBuffer);
			return true;
		}else{
			// no data
			return false;
		}
	}else{
		// no buffer
		return false;
	}
}

void uartswTxBitService(void)
{
	if(UartswTxBitNum){
		// there are bits still waiting to be transmitted
		if(UartswTxBitNum > 1){
			// transmit data bits (inverted, LSB first)
			#ifdef UARTSW_INVERT
			if( !(UartswTxData & 0x01) )
			#else
			if( (UartswTxData & 0x01) )
			#endif
			GPIO_PinOutSet(UARTSW_TX_PORT, UARTSW_TX_PIN);
			else
			GPIO_PinOutClear(UARTSW_TX_PORT, UARTSW_TX_PIN);
			// shift bits down
			UartswTxData = UartswTxData>>1;
		}else{
			// transmit stop bit
			#ifdef UARTSW_INVERT
			GPIO_PinOutClear(UARTSW_TX_PORT, UARTSW_TX_PIN);
			#else
			GPIO_PinOutSet(UARTSW_TX_PORT, UARTSW_TX_PIN);
			#endif
		}
		// count down
		UartswTxBitNum--;
	}else{
		enumUartswState = E_UARTSW_STATE_IDLE;
		GPIO_PinModeSet(UARTSW_RX_PORT, UARTSW_RX_PIN, gpioModeInput, 0);
		/// enable start bit interrupt - uart receiver
		GPIO_IntClear(1 << UARTSW_RX_PIN);
		GPIO_IntEnable(1 << UARTSW_RX_PIN);

		TIMER_IntDisable(TIMER1, TIMER_IF_OF);

		// transmission is done
		// clear busy flag
		UartswTxBusy = false;
	}
}

void uartswRxBitService(void)
{
	// this function runs on either:
	// - a rising edge interrupt
	if(!UartswRxBusy)
	{
		// this is a start bit
		// disable start bit interrupt
		GPIO_IntDisable(1 << UARTSW_TX_PIN);
		enumUartswState = E_UARTSW_STATE_RXD;

		// schedule data bit sampling 1.5 bit periods from now
		TIMER_CounterSet(TIMER1, 0UL);
		TIMER_TopSet(TIMER1,  UartswBaudRateDiv + (UartswBaudRateDiv/2));
		// clear interrupt flag
		TIMER_IntClear(TIMER1, TIMER_IF_OF); // rxd

		 /* Ensable  interrupt */
		TIMER_IntEnable(TIMER1, TIMER_IF_OF); // rxd

		// set start bit flag
		UartswRxBusy = true;
		// reset bit counter
		UartswRxBitNum = 0;
		// reset data
		UartswRxData = 0;
	}
	else
	{
		// start bit has already been received
		// we're in the data bits

		// shift data byte to make room for new bit
		UartswRxData = UartswRxData>>1;

		// sample the data line
		#ifdef UARTSW_INVERT
		if( !GPIO_PinInGet(UARTSW_RX_PORT, UARTSW_RX_PIN) )
		#else
			if( GPIO_PinInGet(UARTSW_RX_PORT, UARTSW_RX_PIN) )
		#endif
		{
			// serial line is marking
			// record '1' bit
			UartswRxData |= 0x80;
		}

		// increment bit counter
		UartswRxBitNum++;
		// schedule next bit sample
		TIMER_TopSet(TIMER1, UartswBaudRateDiv);

		// check if we have a full byte
		if(UartswRxBitNum >= 8)
		{
			// save data in receive buffer
			bufferAddToEnd(&uartswRxBuffer, UartswRxData);

			// disable interrupt
			TIMER_IntDisable(TIMER1, TIMER_IF_OF); // rxd

			enumUartswState = E_UARTSW_STATE_IDLE;
			// clear interrupt flag
			GPIO_IntClear(1 << UARTSW_RX_PIN);
			// enable interrupt
			GPIO_IntEnable(1 << UARTSW_RX_PIN);
			// clear start bit flag
			UartswRxBusy = false;
		}
	}
}


void uartswFlushReceiveBuffer(void)
{
	// flush all data from receive buffer
	bufferFlush(&uartswRxBuffer);
}
