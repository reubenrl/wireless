/*
 * sdi12.h
 *
 *  Created on: Dec 17, 2017
 *      Author: phytech
 */

#ifndef MY_SRC_INC_SDI12_H_
#define MY_SRC_INC_SDI12_H_

/*
 * Probe identification (aI! command)
 * Send the aI! command to fetch the probe identification. This responds with:
 * Allccccccccmmmmmmvvvxxxxxxxxxxxx<CR><LF>
 *	a = Probe address
 *	l ="13" SDI-12 protocol version 1.3
 *	c = "AquaChck" company name
 *	m = "ACCSDI" or “ACHSDI” – probe type model;
 */
#define PSTR_SDI12_CMD_IDENTIFICATION	"0I!"
#define PSTR_SDI12_CMD_FIND_ADDR		"?!" 	// find the address of a probe – only use this command with one probe connected on the bus
#define PSTR_SDI12_CMD_ACK_ACTIVE		"0!"	// Acknowledge probe is present.
#define PSTR_SDI12_CMD_START_MEASURE	"0C!"	// Start measurement.
												// The probe replies with atttn<cr><lf>
												// where ttt is the delay time in seconds and n is the number of sensors.
#define PSTR_SDI12_CMD_READ_DATA		"0D!"	// Read data.
												// command responds with data in the format:
												// asiii.dddsiii.dddsiii.ddd<cr><lf>
												// where s is a sign “+” or “-“, iii is the integer value and ddd is the decimal value
												// example : a+123.45-4.689+100.234<cr><lf> This response has 3 values of 123.45 -4.689 and +100.234
#define PSTR_SDI12_CMD_READ_PROB_CONFIG	"0X#!"	// Read Probe Configuration
												// responds: a#nLx<CR><LF>
												// #n = number of sensors (Only available from firmware V40)
												// Lx = probe length identifier
												// where x=
												//			0 Not Set
												//			1 10cm
												//			2 20cm
												//			3 30cm
												//			4 40cm
												//			5 60cm
												//			6 80cm
												//			7 100cm
												//			8 120cm
												//			9 140cm
												//			10 150cm
												//			11 160cm (example a#6L6 is a 6 sensor 80cm probe)

void sdi12SensorPwrOn(void);
void sdi12SensorPwrOff(void);
void sdi12Init(void);
void sdi12TxData(uint8_t data);
bool sdi12GetData(uint8_t *pData);
uint8_t * sdi12GetDataBuffer(uint16_t timeout_100ms);
char * sdi12Cmd(char strCmd[], char strResp[], uint16_t timeout_sec);

#endif /* MY_SRC_INC_SDI12_H_ */
