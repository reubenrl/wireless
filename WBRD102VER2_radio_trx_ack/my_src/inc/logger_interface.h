/*
 * logger_interface.h
 *
 *  Created on: Mar 13, 2017
 *      Author: phytech
 */

#ifndef MY_SRC_INC_LOGGER_INTERFACE_H_
#define MY_SRC_INC_LOGGER_INTERFACE_H_

#include "bsp.h"

void logger_interfaceSendBuffer(void);
uint16_t logger_interfaceGetBuffer(uint8_t strIn[], uint16_t nLenBuff, uint8_t nTimeout_sec);


#endif /* MY_SRC_INC_LOGGER_INTERFACE_H_ */
