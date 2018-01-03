/*
 * gps.c
 *
 *  Created on: Nov 17, 2016
 *      Author: phytech
 */

#include "bsp.h"
#if defined(BSP_GPS_MODULE)
#include <stdint.h>

#include "ustimer.h"
#include "uart2.h"
#include "uart_dbg_print.h"
#include "nmea.h"
#include "gps.h"

#ifndef PI
#define PI                 3.14159265358979f
#endif

extern 	GpsInfoType 	GpsInfo;

//extern bool uartInitialized[UART_NUMB_MAX];

void gps_bypassUart(void)
{
	uint8_t c;

	if(!uartInitialized[DBG_VCOM]){
		uart0Init();
	}else if(!uartInitialized[UART_GPS]){
		uart1Init();
	}

	do{
		if (uartReceiveByte(UART_GPS, &c)){// data in the leuart
			uartSendByte(DBG_VCOM, c);
		}else
			if(uartReceiveByte(DBG_VCOM, &c)){
				uartSendByte(UART_GPS, c);
			}
	}while(1);
}


#define NMEA_NODATA		0	// No data. Packet not available, bad, or not decoded
#define NMEA_GPGGA		1	// Global Positioning System Fix Data
#define NMEA_GPVTG		2	// Course over ground and ground speed
#define NMEA_GPGLL		3	// Geographic position - latitude/longitude
#define NMEA_GPGSV		4	// GPS satellites in view
#define NMEA_GPGSA		5	// GPS DOP and active satellites
#define NMEA_GPRMC		6	// Recommended minimum specific GPS data
#define NMEA_UNKNOWN	0xFF// Packet received but not known


// Functions
void gpsInit(void)
{
	uint8_t *Packet;

	if(!uartInitialized[UART_GPS]){
		uart1Init();
	}

	Packet = nmeaGetPacketBuffer();
	for(;;){
		if(!uartReceiveBufferIsEmpty(UART_GPS)){
			USTIMER_Init();
			USTIMER_Delay(250000UL); // 250ms
			USTIMER_DeInit();


			switch (nmeaProcess(uartGetRxBuffer(UART_GPS))) {
				case NMEA_GPGGA:
					//printDbg("found GPGGA packet."); printDbgCRLF();
					nmeaProcessGPGGA(Packet);
					BSP_LedToggle(LED2);
					break;
				case NMEA_GPVTG:
					//printDbg("found GPVTG packet."); printDbgCRLF();
					nmeaProcessGPVTG(Packet);
					break;
				case NMEA_GPGLL:
				case NMEA_NODATA:
				case NMEA_UNKNOWN:
				case NMEA_GPGSV:
				case NMEA_GPGSA:
				case NMEA_GPRMC:
					break;
				default:
					break;
			}

			gpsInfoPrint();
		}
	}// end for
}



GpsInfoType* gpsGetInfo(void)
{
	return &GpsInfo;
}

void gpsInfoPrint(void)
{

	printDbg("TOW:      %f", GpsInfo.TimeOfWeek.f);
	printDbg("WkNum:    %d", GpsInfo.WeekNum);
	printDbg("UTCoffset:%f", GpsInfo.UtcOffset.f);
	printDbg("Num SVs:  %d", GpsInfo.numSVs);
	printDbgCRLF();
	printDbg("X_ECEF:   %f", GpsInfo.PosECEF.x.f);
	printDbg("Y_ECEF:   %f", GpsInfo.PosECEF.y.f);
	printDbg("Z_ECEF:   %f", GpsInfo.PosECEF.z.f);
	printDbg("TOF:      %f", GpsInfo.PosECEF.TimeOfFix.f);
	printDbg("Updates:  %d", GpsInfo.PosECEF.updates);
	printDbgCRLF();
	//u08 str[20];
	//rprintfProgStrM(" PosLat: ");		rprintfStr(dtostrf(GpsInfo.PosLat.f, 10, 5, str));
	printDbg("PosLat:   %f", 180*(GpsInfo.PosLLA.lat.f/PI));
	printDbg("PosLon:   %f", 180*(GpsInfo.PosLLA.lon.f/PI));
	printDbg("PosAlt:   %f", GpsInfo.PosLLA.alt.f);
	printDbg("TOF:      %f", GpsInfo.PosLLA.TimeOfFix.f);
	printDbg("Updates:  %d", GpsInfo.PosLLA.updates);
	printDbgCRLF();
	printDbg("Vel East: %f", GpsInfo.VelENU.east.f);
	printDbg("Vel North:%f", GpsInfo.VelENU.north.f);
	printDbg("Vel Up:   %f", GpsInfo.VelENU.up.f);
//	rprintfProgStrM("TOF:      ");	rprintfFloat(8, GpsInfo.VelENU.TimeOfFix.f);	rprintfCRLF();
	printDbg("Updates:  %d", GpsInfo.VelENU.updates);
	printDbgCRLF();
	printDbg("Vel Head: %f", GpsInfo.VelHS.heading.f);
	printDbg("Vel Speed:%f", GpsInfo.VelHS.speed.f);
//	rprintfProgStrM("TOF:      ");	rprintfFloat(8, GpsInfo.VelHS.TimeOfFix.f);	rprintfCRLF();
	printDbg("Updates:  %d",GpsInfo.VelHS.updates);
	printDbgCRLF();
}

#endif

