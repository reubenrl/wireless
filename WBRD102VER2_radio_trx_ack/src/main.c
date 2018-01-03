/***************************************************************************//**
 * @file main.c
 * @brief EZRadio trx example with auto acknowledge option enabled.
 *
 * This example shows how to easily implement a trx code with auto acknowledge
 * option for your controller using EZRadio or EZRadioPRO devices.
 *
 * @version 4.4.0
 *******************************************************************************
 * @section License
 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "spidrv.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "rtcdriver.h"
#include "ustimer.h"

#include "ezradio_cmd.h"
#include "ezradio_api_lib.h"
#include "ezradio_plugin_manager.h"

#include "bsp.h"
#include "uart_dbg_print.h"
#include "measure.h"
#include "tmr_in_capture.h"
#include "cap_meter.h"
#include "sdi12.h"

//#define ONEWIRE_TEST
//#define FREQUENCY_COUNTER
#include "ow.h"

#if defined(WBRD102_VER2_RF_TRANSMITTER)
#define SENSOR_ID		0x00000002
#elif defined(WBRD102_VER2_RF_RECEIVER)
#define SENSOR_ID		0x00000001
#else
#error "not defined Sensor ID..."
#endif

#define MY_PKG_SIZE		9

#if defined(BSP_GPS_MODULE)
#include "gps.h"
//#define GPS_UART_BYPASS
#endif


/* Push button callback functionns. */
static void GPIO_PB0_IRQHandler( uint8_t pin );
volatile bool buttonPressed = false;

/* GPS System ON callback functionns. */
static void GPIO_GPS_SYS_ON_RDY_IRQHandler( uint8_t pin );
volatile bool gpsSysOnReady = false;

static volatile uint8_t idxCounter = 0;


void ustimerDelay_ms(uint32_t delay_ms);

unsigned int calc_crc_8(unsigned char *buff, int len) ;

#if (defined EZRADIO_VARIABLE_DATA_START)
#define APP_PKT_DATA_START EZRADIO_VARIABLE_DATA_START
#else
#define APP_PKT_DATA_START 1u
#endif

static void appPacketTransmittedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status );
static void appPacketReceivedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status );
static void appPacketCrcErrorCallback ( EZRADIODRV_Handle_t handle, Ecode_t status );
static void appAutoAckTransmittedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status );

#if !defined(__CROSSWORKS_ARM) && defined(__GNUC__)
/* sniprintf does not process floats, but occupy less flash memory ! */
#define snprintf    sniprintf
#endif

/* Defines the number of packets to send for one press of PB1.
 * Sends infinite number of packets if defined to 0xFFFF. */
#define APP_TX_PKT_SEND_NUM   0xFFFF

/* Length of the actual data in the Tx packet */
#define APP_TX_PKT_DATA_LENGTH   2u

/* Length of the actual data in the ACK packet */
#define APP_AUTO_ACK_PKT_DATA_LENGTH   3u

/* Rx packet data array */
static uint8_t radioRxPkt[EZRADIO_FIFO_SIZE];

/* Tx packet data array, initialized with the default payload in the generated header file */
static uint8_t radioTxPkt[EZRADIO_FIFO_SIZE] = RADIO_CONFIG_DATA_CUSTOM_PAYLOAD;

/* Auto ack packet data array */
static uint8_t radioAutoAckPkt[RADIO_CONFIG_DATA_MAX_PACKET_LENGTH];

#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == TRUE)
/* Default length configuration for normal transmission */
static EZRADIODRV_PacketLengthConfig_t radioTxLengthConf =
    { ezradiodrvTransmitLenghtDefault, RADIO_CONFIG_DATA_MAX_PACKET_LENGTH, RADIO_CONFIG_DATA_FIELD_LENGTH };
#endif //#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == TRUE)

/* Data counter in transmitted packet */
//static volatile uint16_t appDataCntr = 0;

/* Packet counter */
//static volatile uint16_t appTxPktCntr = 0;

/* Sign tx active state */
static volatile bool appTxActive = false;

static volatile bool appAckWaiting = false;

/* RTC frequency */
#define APP_RTC_FREQ_HZ 2u
/* RTC timeout */
#define APP_RTC_TIMEOUT_MS ( 1000u / APP_RTC_FREQ_HZ )

/* RTC set time is expired */
static volatile bool rtcTick = false;
/* RTC set sllep time is expired */
static volatile bool rtcSleepTick = false;
static volatile bool AckWaitingTimeout = false;

static volatile bool tmr2InCapEnable = false;

/** Timer used to issue time elapsed interrupt. */
static RTCDRV_TimerID_t rtcTickTimer;
static RTCDRV_TimerID_t rtcSleepTimer;
static RTCDRV_TimerID_t rtcAckWaitingTimer;

/* EZRadio response structure union */
ezradio_cmd_reply_t ezradioReply;

/**************************************************************************//**
 * @brief Setup GPIO interrupt for pushbuttons.
 *****************************************************************************/
static void GpioSetup(void)
{
  /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Initialize GPIO interrupt */
  GPIOINT_Init();

  /* Configure main power enable */
  GPIO_PinModeSet(GPIO_MAIN_PWR_ON_OFF_PORT, GPIO_MAIN_PWR_ON_OFF_PIN, gpioModePushPull, 1);

  /* Configure GPS on system i/o */
  GPIO_PinModeSet(GPIO_GPS_SYS_ON_RDY_PORT, GPIO_GPS_SYS_ON_RDY_PIN, gpioModeInput, 0);
  /* Configure GPS on system interrupt */
  GPIO_IntConfig(GPIO_GPS_SYS_ON_RDY_PORT, GPIO_GPS_SYS_ON_RDY_PIN, true, false, true);
  GPIOINT_CallbackRegister( GPIO_GPS_SYS_ON_RDY_PIN, GPIO_GPS_SYS_ON_RDY_IRQHandler );

  /* Configure PB0 as input and enable interrupt */
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInput, 0);
  GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, false, true, true);
  GPIOINT_CallbackRegister( BSP_GPIO_PB0_PIN, GPIO_PB0_IRQHandler );

  /* Configure TCXO power enable */
  GPIO_PinModeSet(GPIO_TCXO_PWR_ENA_PORT, GPIO_TCXO_PWR_ENA_PIN, gpioModePushPull, 0);

  /* Configure GPS power enable */
  GPIO_PinModeSet(GPIO_GPS_PWR_ENA_PORT, GPIO_GPS_PWR_ENA_PIN, gpioModePushPull, 0);

  /* Configure GPS ignition */
  GPIO_PinModeSet(GPIO_GPS_IGNITION_PORT, GPIO_GPS_IGNITION_PIN, gpioModePushPull, 0);

  /* Configure sensor power enable */
  GPIO_PinModeSet(GPIO_SENS_PWR_ENA_PORT, GPIO_SENS_PWR_ENA_PIN, gpioModePushPull, 0);

  /* Configure battery measure enable */
  GPIO_PinModeSet(GPIO_BATT_VTG_LEVEL_ENA_PORT, GPIO_BATT_VTG_LEVEL_ENA_PIN, gpioModePushPull, 1);



	GPIO_PinModeSet(gpioPortF, 3, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, 0);

}

/**************************************************************************//**
 * @brief GPIO Interrupt handler (PB0)
 *        SYSTEM_ON signal reflects the power state of the SE880,
 *        logic low for hibernate mode, and logic high for full power mode.
 *****************************************************************************/
static void GPIO_PB0_IRQHandler( uint8_t pin )
{
  (void)pin;

  buttonPressed = true;
}


/**************************************************************************//**
 * @brief GPIO_GPS_SYS_ON_RDY_IRQHandler
 *        Switches between analog and digital clock modes.
 *****************************************************************************/
static void GPIO_GPS_SYS_ON_RDY_IRQHandler( uint8_t pin )
{
 (void)pin;
 // GPIO_IntDisable(1 << GPIO_GPS_SYS_ON_RDY_PIN);
  gpsSysOnReady = true;

}

static uint32_t rtcTickCount250_ms;

void RTC_App_IRQHandler()
{
  rtcTick = true;
  rtcTickCount250_ms++;
  tmr2InCapEnable = true;
}

void RTC_SLEEP_IRQHandler()
{
	rtcSleepTick = true;
}

void RTC_ACKWAITING_IRQHandler()
{
	AckWaitingTimeout = true;
}

/**************************************************************************//**
 * @brief  Main function of the example.
 *****************************************************************************/
int main(void)
{
	uint16_t *pInCaptureValue;

  /* EZRadio driver init data and handler */
  EZRADIODRV_HandleData_t appRadioInitData = EZRADIODRV_INIT_DEFAULT;
  EZRADIODRV_Handle_t appRadioHandle = &appRadioInitData;


  /* Chip errata */
  CHIP_Init();

  /* HFXO 48MHz, divided by 1 */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Setup GPIO for pushbuttons. */
  GpioSetup();

 /*
  cap_meterStart();
  while(1);
*/

  sdi12Init();
  USTIMER_Init();
  USTIMER_Delay(1000UL*5000UL);
  USTIMER_DeInit();
  sdi12SensorPwrOn();
  char * ret = sdi12Cmd(PSTR_SDI12_CMD_START_MEASURE, "\r\n", 10);
  if(ret != NULL) printDbg("%s", ret);
  else printDbg("sdi12...error");
  USTIMER_Init();
   USTIMER_Delay(1000UL*2500UL);
   USTIMER_DeInit();
  ret = sdi12Cmd(PSTR_SDI12_CMD_READ_DATA, "\r\n", 10);
  if(ret != NULL) printDbg("%s", ret);
    else printDbg("sdi12...error");

  sdi12SensorPwrOff();
  /*
  sdi12TxData("0I!", 3);
  while(1){
	  sdi12RxProc();
  }*/

  while(1);

  /* Set RTC to generate interrupt 250ms. */
  RTCDRV_Init();
#if defined (WBRD102_VER2_RF_RECEIVER)
  if (ECODE_EMDRV_RTCDRV_OK != RTCDRV_AllocateTimer( &rtcTickTimer) ){
    while (1);
  }
  if (ECODE_EMDRV_RTCDRV_OK != RTCDRV_StartTimer(rtcTickTimer, rtcdrvTimerTypePeriodic, APP_RTC_TIMEOUT_MS,
                        (RTCDRV_Callback_t)RTC_App_IRQHandler, NULL ) ){
    while (1);
  }
#elif defined (WBRD102_VER2_RF_TRANSMITTER)
  /* Set RTC to generate sleep interrupt 1 minute */
  RTCDRV_Init();
  if (ECODE_EMDRV_RTCDRV_OK != RTCDRV_AllocateTimer( &rtcSleepTimer) ){
	  while (1);
  }
  if (ECODE_EMDRV_RTCDRV_OK != RTCDRV_StartTimer(rtcSleepTimer, rtcdrvTimerTypePeriodic, 1000u*60u,
                         (RTCDRV_Callback_t)RTC_SLEEP_IRQHandler, NULL ) ){
	  while (1);
  }

  if (ECODE_EMDRV_RTCDRV_OK != RTCDRV_AllocateTimer( &rtcAckWaitingTimer) ){
	  while (1);
  }
  /*
  if (ECODE_EMDRV_RTCDRV_OK != RTCDRV_StartTimer(rtcAckWaiting, rtcdrvTimerType, 1000u*60u,
                         (RTCDRV_Callback_t)RTC_ACKWAITING_IRQHandler, NULL ) ){
	  while (1);
  }
 */

#endif


  /* Print header */
  printDbgCRLF();
#if defined(WBRD102_VER2_RF_TRANSMITTER)
  printDbg("EZRadio Transmitter...");
#elif defined(WBRD102_VER2_RF_RECEIVER)
  printDbg("EZRadio Receiver...");
#endif


  /* Configure packet transmitted callback. */
  appRadioInitData.packetTx.userCallback = &appPacketTransmittedCallback;
#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == TRUE)
  /* Store length for variable length packet */
  radioTxPkt[EZRADIO_LENGTH_WORD_START + RADIO_CONFIGURATION_DATA_PKT_LENGTH_SIZE - 1] = APP_TX_PKT_DATA_LENGTH;

  /* Set the length of the variable length to the actual data */
#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 1)
  radioTxLengthConf.fieldLen.f1 = APP_TX_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 2)
  radioTxLengthConf.fieldLen.f2 = APP_TX_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 3)
  radioTxLengthConf.fieldLen.f3 = APP_TX_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 4)
  radioTxLengthConf.fieldLen.f4 = APP_TX_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 5)
  radioTxLengthConf.fieldLen.f5 = APP_TX_PKT_DATA_LENGTH;
#endif

#endif //#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == TRUE)

  /* Configure packet received buffer and callback. */
  appRadioInitData.packetRx.userCallback = &appPacketReceivedCallback;
  appRadioInitData.packetRx.pktBuf = radioRxPkt;

  /* Configure packet received with CRC error callback. */
  appRadioInitData.packetCrcError.userCallback = &appPacketCrcErrorCallback;

  /* Configure auto ack packet buffer and callback. */
  appRadioInitData.autoAck.pktBuf = radioAutoAckPkt;
  appRadioInitData.autoAck.userCallback = &appAutoAckTransmittedCallback;

  /*Store auto ack packet data */
  radioAutoAckPkt[APP_PKT_DATA_START]     = 'A';
  radioAutoAckPkt[APP_PKT_DATA_START + 1] = 'C';
  radioAutoAckPkt[APP_PKT_DATA_START + 2] = 'K';

#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == TRUE)
  /* Auto ack data length differs from the normal packet length, so handle that accordingly. */
  appRadioInitData.autoAck.pktMode = ezradiodrvAutoAckPktCustom;
  appRadioInitData.autoAck.lenConfig.lenMode = ezradiodrvTransmitLenghtCustomFieldLen;

  /* Store length for variable length packet */
  radioAutoAckPkt[EZRADIO_LENGTH_WORD_START + RADIO_CONFIGURATION_DATA_PKT_LENGTH_SIZE - 1] = APP_AUTO_ACK_PKT_DATA_LENGTH;

  /* Set the length of the variable length to the auto ack data */
#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 1)
  appRadioInitData.autoAck.lenConfig.fieldLen.f1 = APP_AUTO_ACK_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 2)
  appRadioInitData.autoAck.lenConfig.fieldLen.f2 = APP_AUTO_ACK_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 3)
  appRadioInitData.autoAck.lenConfig.fieldLen.f3 = APP_AUTO_ACK_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 4)
  appRadioInitData.autoAck.lenConfig.fieldLen.f4 = APP_AUTO_ACK_PKT_DATA_LENGTH;
#elif (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_LENGTH_FIELD == 5)
  appRadioInitData.autoAck.lenConfig.fieldLen.f5 = APP_AUTO_ACK_PKT_DATA_LENGTH;
#endif

#endif //#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == TRUE)

  /* tcxo power on */
  printDbg("tcxo power on");
  GPIO_PinOutSet(GPIO_TCXO_PWR_ENA_PORT, GPIO_TCXO_PWR_ENA_PIN);
  /* stabilize tcxo delay  */
  ustimerDelay_ms(200u);	// 200ms

  printDbg("Initialize EZRadio device");
  /* Initialize EZRadio device. */
  ezradioInit( appRadioHandle );

  /* Enable auto acknowledge feature. */
  ezradioEnableAutoAck( &(appRadioHandle->autoAck) );

  /* Print EZRadio device number. */
  ezradio_part_info(&ezradioReply);
  printDbg("Device: Si%04x", ezradioReply.PART_INFO.PART);

  /* Print instructions. */
  //printDbg(" Press PB0 to send\n  one packet.\n");
#if (APP_TX_PKT_SEND_NUM == 0xFFFF)
  //printf(" Press PB1 to send\n  unlimited packets.\n");
#else
  //printDbg(" Press PB1 to send\n  %d packets.\n", APP_TX_PKT_SEND_NUM);
#endif

  /* Reset radio fifos and start reception. */
  ezradioResetTRxFifo();
  ezradioStartRx( appRadioHandle );

  BSP_LedsInit();

#if defined(BSP_GPS_MODULE) & defined (WBRD102_VER2_RF_TRANSMITTER)

  printDbg("gps ldo enable");
  BSP_LedSet(LED2);
  GPIO_PinOutSet(GPIO_GPS_PWR_ENA_PORT, GPIO_GPS_PWR_ENA_PIN);
  /* stabilize gps ldo  */
  ustimerDelay_ms(3000u); 	// 3 seconds
  printDbg("gps ignition power on");
  GPIO_PinOutSet(GPIO_GPS_IGNITION_PORT, GPIO_GPS_IGNITION_PIN);
  ustimerDelay_ms(2000u); // 2 seconds
  GPIO_PinOutClear(GPIO_GPS_IGNITION_PORT, GPIO_GPS_IGNITION_PIN);

  /*
  while(!gpsSysOnReady);	// system on rising detected

  GPIO_IntConfig(GPIO_GPS_SYS_ON_RDY_PORT, GPIO_GPS_SYS_ON_RDY_PIN, false, true, true);
  gpsSysOnReady = false;
  while(!gpsSysOnReady);


  ustimerDelay_ms(250u); // 250ms
  printDbg("gps ignition power on");
  GPIO_PinOutSet(GPIO_GPS_IGNITION_PORT, GPIO_GPS_IGNITION_PIN);
  ustimerDelay_ms(500u); // 500msec
  GPIO_PinOutClear(GPIO_GPS_IGNITION_PORT, GPIO_GPS_IGNITION_PIN);

  ustimerDelay_ms(100u); // 100ms
  */
  if(GPIO_PinInGet(GPIO_GPS_SYS_ON_RDY_PORT, GPIO_GPS_SYS_ON_RDY_PIN)){
	  printDbg("gps start on... successful");

  }else{
	  printDbg("gps start on... failed");
	  BSP_LedSet(LED1);
	  BSP_LedClear(LED2);
  }

#if defined(GPS_UART_BYPASS)
  gps_bypassUart();
#else
  gpsInit();
#endif

#endif

  /* wait until push button is released. */
  while(!GPIO_PinInGet(GPIO_BUTTON_STATUS_PORT, GPIO_BUTTON_STATUS_PIN));
  buttonPressed = false;
#if defined(FREQUENCY_COUNTER)
  tmr2InCaptureInit();
  uint32_t frequencyValue;
#endif
  /* Enter infinite loop that will take care of ezradio plugin manager, packet transmission
   * and auto acknowledge. */
  while (1){

	  /* Run radio plug-in manager */
    ezradioPluginManager( appRadioHandle );

#if defined(FREQUENCY_COUNTER)
    if(!(rtcTickCount250_ms % 20u) && (tmr2InCapEnable == true)){
    	tmr2InCapEnable = false;
    	pInCaptureValue = tmr2InCaptureGet();
    	//for(uint8_t idx=0; idx < 8; idx++) printDbg("bufferA[%d]:%d", idx, pInCaptureValue[idx] );
    	frequencyValue = (pInCaptureValue[4] - pInCaptureValue[3]);
    	frequencyValue *= 32u;
    	frequencyValue = 240000000u/frequencyValue;

    	printDbg("\r\nFREQUENCY:%d.%.1dHZ", frequencyValue/10u, frequencyValue%10u);

    	//inCaptureValue = tmr2InCaptureGet();
    	//printDbg("frequency:%dhz", 24000000/(inCaptureValue * 32u));
      }
#endif

#if defined (WBRD102_VER2_RF_RECEIVER)
    if(rtcTick && buttonPressed){
    	rtcTick = false;

#elif defined(WBRD102_VER2_RF_TRANSMITTER)
    	/* Send a packet if push button pressed  OR timer wakeup*/
    if (rtcSleepTick || buttonPressed){
    	rtcSleepTick = false;
#endif
    	 /* wait until push button is released. */
    	while(!GPIO_PinInGet(GPIO_BUTTON_STATUS_PORT, GPIO_BUTTON_STATUS_PIN));
    	buttonPressed = false;

        /* Try to send the packet */
        if ( !appTxActive ){
          /* Sing tx active state */
        	appTxActive = true;
        	appAckWaiting = true;
        	// radio sleep - low noise capture sensor
        	ezradio_change_state( EZRADIO_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);
        	ustimerDelay_ms(200);
          	GPIO_PinOutClear(GPIO_TCXO_PWR_ENA_PORT, GPIO_TCXO_PWR_ENA_PIN);
            BSP_LedSet(LED2);
            printDbg("RF TXD starting" );

            uint32_t sensorID = SENSOR_ID;
          printDbg("Sensor ID:%lu", sensorID);
          radioTxPkt[APP_PKT_DATA_START + 0] = (uint8_t)( ((uint32_t)sensorID) >> 24 );			// high value
          radioTxPkt[APP_PKT_DATA_START + 1] = (uint8_t)( ((uint32_t)sensorID) >> 16 );
          radioTxPkt[APP_PKT_DATA_START + 2] = (uint8_t)( ((uint32_t)sensorID) >> 8 );
          radioTxPkt[APP_PKT_DATA_START + 3] = (uint8_t)( ((uint32_t)sensorID) & 0x000000FF );	// low value

          radioTxPkt[APP_PKT_DATA_START + 4] = ++idxCounter;
          printDbg("idxCounter:%d", idxCounter);

#if defined(ONEWIRE_TEST)
          uint16_t ds1820Temperature;
          if(ds1820GetTemperature(&ds1820Temperature)){
        	  ds1820Temperature = 0x31CE;
          }
          printDbg("Temperature:%0.1fC", (signed)ds1820Temperature/10.0);
          uint16_t adcValue_mV = ds1820Temperature;
#elif defined(FREQUENCY_COUNTER)
          uint16_t adcValue_mV = (uint16_t)frequencyValue/10u;

#else
          uint16_t adcValue_mV = (uint16_t)measureSensor_mV();
          printDbg("Sensor Value:%dmV", adcValue_mV);
#endif

          radioTxPkt[APP_PKT_DATA_START + 5] = (uint8_t)(adcValue_mV >> 8);	// high value
          radioTxPkt[APP_PKT_DATA_START + 6] = (uint8_t)adcValue_mV;		// low value

          adcValue_mV = (uint16_t)measureBatteryVoltage_mV();
          printDbg("Battery Value::%dmV", adcValue_mV);
          radioTxPkt[APP_PKT_DATA_START + 7]   	= (uint8_t)( ((uint16_t)adcValue_mV) >> 8 );			// high value
          radioTxPkt[APP_PKT_DATA_START + 8] 	= (uint8_t)( ((uint16_t)adcValue_mV) & 0x00FF );		// low value

    	  radioTxPkt[APP_PKT_DATA_START + 9] = (uint8_t)calc_crc_8(&radioTxPkt[APP_PKT_DATA_START], MY_PKG_SIZE);

    	  char  strTmp[32];
    	  for(int i=0; i< (MY_PKG_SIZE + 1);i++){
    		  sprintf(&strTmp[i*2],"%02X", radioTxPkt[APP_PKT_DATA_START + i]);
    	  }
    	  printDbg("payload:%s", strTmp);
    	  printDbg("tx-<chks:%x", radioTxPkt[APP_PKT_DATA_START + 9]);
    	  // radio wakeup
          GPIO_PinOutSet(GPIO_TCXO_PWR_ENA_PORT, GPIO_TCXO_PWR_ENA_PIN);
          ustimerDelay_ms(200);
          ezradio_change_state( EZRADIO_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_RX);

          /* Add data cntr as the data to be sent to the packet */
          //radioTxPkt[APP_PKT_DATA_START]   = (uint8_t)( ((uint16_t)appDataCntr) >> 8 );
          //radioTxPkt[APP_PKT_DATA_START+1] = (uint8_t)( ((uint16_t)appDataCntr) & 0x00FF );

          /* Note: The following line issues the auto acknowledge feature to skip
           *       one session.
           *        - Should be used for links where both nodes transmits ACK packets,
           *          in order to skip auto ACK for received ACK packets.
           *        - Should be commented out if the receiver node does not send back
           *          auto acknowledge packets. */
          ezradioSkipAutoAck( &(appRadioHandle->autoAck) );

          /* Transmit packet */
#if (RADIO_CONFIGURATION_DATA_PKT_VARIABLE_PACKET_LENGTH_ENABLE == FALSE)
          ezradioStartTransmitConfigured( appRadioHandle, radioTxPkt );
#else
          ezradioStartTransmitCustom( appRadioHandle, radioTxLengthConf, radioTxPkt );
#endif

#if defined (WBRD102_VER2_RF_TRANSMITTER)
          // 3 second
          AckWaitingTimeout = false;
           if (ECODE_EMDRV_RTCDRV_OK != RTCDRV_StartTimer(rtcAckWaitingTimer, rtcdrvTimerTypeOneshot, 1000u*3u,
                                  (RTCDRV_Callback_t)RTC_ACKWAITING_IRQHandler, NULL ) ){
         	  while (1);
           }
#endif
        }else{
          printDbg("Data TX: need to wait.");
        }

    }else{


#if defined (WBRD102_VER2_RF_TRANSMITTER)
    	if((!appTxActive && !appAckWaiting) || AckWaitingTimeout){
			ezradio_change_state( EZRADIO_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);
			ustimerDelay_ms(200);
			GPIO_PinOutClear(GPIO_TCXO_PWR_ENA_PORT, GPIO_TCXO_PWR_ENA_PIN);
			EMU_EnterEM2(true);
			GPIO_PinOutSet(GPIO_TCXO_PWR_ENA_PORT, GPIO_TCXO_PWR_ENA_PIN);
			ustimerDelay_ms(200);
			ezradio_change_state( EZRADIO_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_RX);
    	}
#endif
    }
  }// end while
}// end main


/**************************************************************************//**
 * @brief  Packet transmitted callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
static void appPacketTransmittedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status )
{
  if ( status == ECODE_EMDRV_EZRADIODRV_OK )
  {
    /* Sign tx passive state */
    appTxActive = false;

    /* Change to RX state */
    ezradioStartRx( handle );
  }

  BSP_LedClear(LED2);
}

/**************************************************************************//**
 * @brief  Packet received callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
static void appPacketReceivedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status )
{
  //Silent warning.
  (void)handle;
  char strTmp[32];

  if ( status == ECODE_EMDRV_EZRADIODRV_OK ){
	  BSP_LedSet(LED1);

    /* Read out and print received packet data:
     *  - print 'ACK' in case of ACK was received
     *  - print the data if some other data was received. */
    if ( (radioRxPkt[APP_PKT_DATA_START] == 'A') &&
         (radioRxPkt[APP_PKT_DATA_START + 1] == 'C') &&
         (radioRxPkt[APP_PKT_DATA_START + 2] == 'K') ){
      printDbg("Data RX: ACK");
      appAckWaiting = false;
    }else{
    	printDbg("Data RX: DATA");
    	for(int i=0;i< (MY_PKG_SIZE + 1); i++)  sprintf(&strTmp[i*2],"%02X",radioRxPkt[i + APP_PKT_DATA_START]);
    	printDbg("%s", strTmp);

    	// check package
    	uint16_t  chks = calc_crc_8(&radioRxPkt[APP_PKT_DATA_START], MY_PKG_SIZE + 1);
    	if(chks){
    		printDbg("chks error");
    	}
    	else{// package process
    	      uint32_t sensorID = 0;
    	      sensorID |= (uint32_t)(radioRxPkt[APP_PKT_DATA_START + 0] << 24) & (uint32_t)(0xFF << 24); // high value
    	      sensorID |= (uint32_t)(radioRxPkt[APP_PKT_DATA_START + 1] << 16) & (uint32_t)(0xFF << 16);
    	      sensorID |= (uint32_t)(radioRxPkt[APP_PKT_DATA_START + 2] << 8) & (uint32_t)(0xFF << 8);
    	      sensorID |= (uint32_t)(radioRxPkt[APP_PKT_DATA_START + 3] << 0) & (uint32_t)(0xFF << 0);	// low value
    		  printDbg("SENSOR ID:%lu", sensorID);

    		  printDbg("Counter:%d", radioRxPkt[APP_PKT_DATA_START + 4]);

    		  uint16_t adcValue_mV = 0;
    		  adcValue_mV = (uint16_t)radioRxPkt[APP_PKT_DATA_START + 5];			// high value
    		  adcValue_mV <<= 8;
    		  adcValue_mV |= (uint16_t)radioRxPkt[APP_PKT_DATA_START + 6] & 0x00FF;	// low value
#if defined(ONEWIRE_TEST)
    		  printDbg("Raw Temp:%d", adcValue_mV);
    		  printDbg("Temperature:%0.1fC", (signed)adcValue_mV/10.0);
#else
    		  printDbg("SENSOR MEASURE:%dmV", adcValue_mV);
    		  // cBar=1013-(Vo+387.5)*10/22.5
    		  // limit Vo <= 1891mV
    		  uint32_t tmpValue = (uint32_t) adcValue_mV;
    		  if(tmpValue <= 1891u){
    			  tmpValue += 387.5;
    			  tmpValue *= 10;
    			  tmpValue /= 22.5;
    			  tmpValue = 1013u - tmpValue;
    		  }else{
    			  tmpValue = 0;
    		  }
    		  // limit cBar < 84
    		  if(tmpValue > 840){
    			  tmpValue = 840;
    		  }
    		  printDbg("TEN:%fcBar", tmpValue/10.0);
#endif
    		  adcValue_mV = 0;
    		  adcValue_mV |= (uint16_t)(radioRxPkt[APP_PKT_DATA_START + 7] << 8) & (uint16_t)(0xFF << 8); 	// high value
    		  adcValue_mV |= (uint16_t)(radioRxPkt[APP_PKT_DATA_START + 8] << 0) & (uint16_t)(0xFF << 0);	// low value
    		  printDbg("BATTERY MEASURE:%dmV", adcValue_mV);
    	}

    }
    /* Read ezradio modem status RSSI value */
     ezradio_get_modem_status(EZRADIO_CMD_GET_MODEM_STATUS_ARG_MODEM_CLR_PEND_MASK, &ezradioReply);
     printDbg("RSSI:%02d\n", ezradioReply.GET_MODEM_STATUS.LATCH_RSSI);

     BSP_LedClear(LED1);
  }


}

/**************************************************************************//**
 * @brief  Packet received with CRC error callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
static void appPacketCrcErrorCallback ( EZRADIODRV_Handle_t handle, Ecode_t status )
{
  if ( status == ECODE_EMDRV_EZRADIODRV_OK )
  {
    //printDbg("-->Pkt  RX: CRC Error\n");

    /* Change to RX state */
    ezradioStartRx( handle );
  }
}

/**************************************************************************//**
 * @brief  Packet transmitted callback of the application.
 *
 * @param[in] handle EzRadio plugin manager handler.
 * @param[in] status Callback status.
 *****************************************************************************/
static void appAutoAckTransmittedCallback ( EZRADIODRV_Handle_t handle, Ecode_t status )
{
	if ( status == ECODE_EMDRV_EZRADIODRV_OK ){
		ezradioStartRx( handle );
	}
}


void ustimerDelay_ms(uint32_t delay_ms)
{
	USTIMER_Init();
	USTIMER_Delay(1000u*delay_ms);	// ms
	USTIMER_DeInit();
}


/* calculates 8-bit CRC of given data */
/* based on the polynomial  X^8 + X^5 + X^4 + 1 */
unsigned int calc_crc_8(unsigned char *buff, int len)
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
