/*
 * eeprom.h
 *
 *  Created on: Mar 16, 2017
 *      Author: phytech
 */

#ifndef EE_M24C08_H_
#define EE_M24C08_H_

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

#define EE_24C08_DEVICE_ADDRESS			0xA0

#define EE_24C08_TOTAL_PAGE_NUMBER		64
#define EE_24C08_PAGE_SIZE				16
/** Number of bytes in EEPROM */
#define EE_24C08_TOTAL_MEMORY_BYTES		EE_24C08_TOTAL_PAGE_NUMBER*EE_24C08_PAGE_SIZE

uint8_t ee_m24c0Test(void);
int ee_m24c08Read(I2C_TypeDef *i2c, uint8_t addr, unsigned int offset, uint8_t *data, unsigned int len);
int ee_m24c08Write(I2C_TypeDef *i2c, uint8_t addr, unsigned int offset, uint8_t *data, unsigned int len);

#endif /* EE_M24C08_H_ */
