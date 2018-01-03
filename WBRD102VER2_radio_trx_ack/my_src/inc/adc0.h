/*
 * adc0.h
 *
 *  Created on: Nov 23, 2016
 *      Author: phytech
 */

#ifndef MY_SRC_INC_ADC0_H_
#define MY_SRC_INC_ADC0_H_

#include <stdbool.h>



void adc0Init(void);
uint32_t adc0EightTimesSingleSample(uint8_t  inputSelect);
uint32_t adc0SingleSample(uint8_t  inputSelect);
uint32_t adc0Get_mV(uint8_t  inputSelect);


#endif /* MY_SRC_INC_ADC0_H_ */
