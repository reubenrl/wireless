/*
 * eeprom.c
 *
 *  Created on: Mar 16, 2017
 *      Author: phytech
 */

/***************************************************************************//**
 * @file
 * @brief EEPROM driver for M24C08 (8Kbit) EEPROM device on the WBRD200-VER2
 *******************************************************************************/


#include "bsp.h"

#if defined(WBRD200_VER1)

#include <stddef.h>
#include "i2cspm.h"

#include "ustimer.h"
#include "uart_dbg_print.h"
#include "ee_m24c08.h"


static volatile bool ee_m24c0Initialized = false;
I2CSPM_Init_TypeDef __i2cInit = I2CSPM_INIT_DEFAULT;

void ee_m24c0Init(void)
{
	if(!ee_m24c0Initialized){
		I2CSPM_Init(&__i2cInit);
		ee_m24c0Initialized = true;
	}
}

uint8_t ee_m24c0Test(void)
{
	uint8_t strPage[EE_24C08_PAGE_SIZE];
	int i;
	bool testSuccessful = 0;

	printDbg("\r\nstart eeprom test");
	for(i=0;i<EE_24C08_PAGE_SIZE;i++){
		if(i & 1) strPage[i] = '\x55';
		else{
			strPage[i] = '\xAA';
		}
	}

	if(ee_m24c08Write(__i2cInit.port, EE_24C08_DEVICE_ADDRESS, (EE_24C08_TOTAL_MEMORY_BYTES - EE_24C08_PAGE_SIZE) , strPage, EE_24C08_PAGE_SIZE) == -1){
	}else{
		USTIMER_Init();
		USTIMER_Delay(5000u);
		USTIMER_DeInit();

		for(i=0;i<EE_24C08_PAGE_SIZE;i++) strPage[i] = 0xFF;
		if(ee_m24c08Read(__i2cInit.port,EE_24C08_DEVICE_ADDRESS , (EE_24C08_TOTAL_MEMORY_BYTES - EE_24C08_PAGE_SIZE), strPage, EE_24C08_PAGE_SIZE) == -1){
		}else{
			for(i=0;i<EE_24C08_PAGE_SIZE;i++){
				if(i & 1){
					if(strPage[i] == '\x55') continue;
					else break;
				}else{
					if(strPage[i] == '\xAA') continue;
					else break;
				}
			}// end for

			if(i == EE_24C08_PAGE_SIZE){
				for(i=0;i<EE_24C08_PAGE_SIZE;i++) printDbg("%X", strPage[i]);
				testSuccessful = true;
			}
		}

	}

	return testSuccessful;
}


/***************************************************************************//**
 * @brief
 *   Do acknowledge polling on EEPROM device.
 *
 * @details
 *   When writing to an EEPROM, the EEPROM device will be busy for some time
 *   after issuing a (page) write. During this time, the EEPROM is not
 *   accessible, and will therefore not ACK any requests. This feature can
 *   be used to determine when the write is actually completed, and is denoted
 *   acknowledgement polling.
 *
 * @note
 *   This function will not return until the EEPROM device acknowledges (or some
 *   sort of I2C failure occurs). If trying to acknowledge poll a non-existing
 *   device, NACK will always result and this function will never return. Thus,
 *   it should not be used unless the EEPROM device is actually present.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] addr
 *   I2C address for EEPROM, in 8 bit format, where LSB is reserved
 *   for R/W bit.
 *
 * @return
 *   Returns 0 when EEPROM acknowledges. Negative value is returned
 *   is some sort of error occurred during acknowledgement polling.
 ******************************************************************************/
static int ee_m24c08AckPoll(I2C_TypeDef *i2c, uint8_t addr)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;

  /* Do acknowledge polling waiting for write process to finish in EEPROM */
  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE;
  /* Just access device with write operation */
  seq.buf[0].data = NULL;
  seq.buf[0].len  = 0;

  /* Wait for ACK from device */
  while (1)
  {
    ret = I2CSPM_Transfer(i2c, &seq);
    if (ret == i2cTransferDone)
    {
      break;
    }
    else if (ret == i2cTransferNack)
    {
      continue;
    }
    else
    {
      return((int) ret);
    }
  }

  return(0);
}


/***************************************************************************//**
 * @brief
 *   Read data from EEPROM.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] addr
 *   I2C address for EEPROM, in 8 bit format, where LSB is reserved
 *   for R/W bit.
 *
 * @param[in] offset
 *   Offset in EEPROM to start reading from.
 *
 * @param[out] data
 *   Location to place read data, must be at least @p len long.
 *
 * @param[in] len
 *   Number of bytes to read.
 *
 * @return
 *   Returns number of bytes read. Less than specified number of bytes is
 *   returned if reading beyond end of EEPROM. Negative value is returned
 *   is some sort of error occurred during read.
 ******************************************************************************/
int ee_m24c08Read(I2C_TypeDef *i2c,
                uint8_t addr,
                unsigned int offset,
                uint8_t *data,
                unsigned int len)
{
  I2C_TransferSeq_TypeDef    	seq;
  I2C_TransferReturn_TypeDef 	ret;

  uint8_t           			offsetLoc[1];
  uint8_t						offsetLoc_h;

  ee_m24c0Init();

  if (offset >= EE_24C08_TOTAL_MEMORY_BYTES)
  {
	  return(0);
  }

  if ((offset + len) > EE_24C08_TOTAL_MEMORY_BYTES)
  {
	  len = EE_24C08_TOTAL_MEMORY_BYTES - offset;
  }

  offsetLoc_h = (uint8_t)((offset >> 8) & 0x0003);
  addr &= 0xF1;

  seq.addr  = addr | offsetLoc_h;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select offset to start reading from */
  offsetLoc[0]    = (uint8_t) offset;
  seq.buf[0].data = offsetLoc;
  seq.buf[0].len  = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = data;
  seq.buf[1].len  = len;

  ret = I2CSPM_Transfer(i2c, &seq);
  if (ret != i2cTransferDone)
  {
    return((int) ret);
  }

  return((int) len);
}


/***************************************************************************//**
 * @brief
 *   Write data to EEPROM.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] addr
 *   I2C address for EEPROM, in 8 bit format, where LSB is reserved
 *   for R/W bit.
 *
 * @param[in] offset
 *   Offset in EEPROM to start writing to.
 *
 * @param[out] data
 *   Location holding data to write, must be at least @p len long.
 *
 * @param[in] len
 *   Number of bytes to write.
 *
 * @return
 *   Returns number of bytes written. Less than specified number of bytes is
 *   returned if writing beyond end of EEPROM. Negative value is returned
 *   is some sort of error occurred during write.
 ******************************************************************************/
int ee_m24c08Write(I2C_TypeDef *i2c,
                 uint8_t addr,
                 unsigned int offset,
                 uint8_t *data,
                 unsigned int len)
{
  I2C_TransferSeq_TypeDef    	seq;
  I2C_TransferReturn_TypeDef 	ret;
  int                        	tmp;
  unsigned int               	chunk;
  unsigned int               	max;
  uint8_t                    	offsetLoc[1];
  uint8_t						offsetLoc_h;

  ee_m24c0Init();

  if (offset >= EE_24C08_TOTAL_MEMORY_BYTES)
  {
    return(0);
  }

  if ((offset + len) > EE_24C08_TOTAL_MEMORY_BYTES)
  {
    len = EE_24C08_TOTAL_MEMORY_BYTES - offset;
  }

  /* Write max one page at a time */
  while (len)
  {
    max = EE_24C08_PAGE_SIZE - (offset % EE_24C08_PAGE_SIZE);

    if (len > max)
    {
      chunk = max;
    }
    else
    {
      chunk = len;
    }

    offsetLoc_h = (uint8_t)((offset >> 8) & 0x0003);
    addr &= 0xF1;

    seq.addr  = addr | offsetLoc_h;
    seq.flags = I2C_FLAG_WRITE_WRITE;
    /* Select offset to start writing to */
    offsetLoc[0]    = (uint8_t) offset;
    seq.buf[0].data = offsetLoc;
    seq.buf[0].len  = 1;
    /* Select location/length of data to be written */
    seq.buf[1].data = data;
    seq.buf[1].len  = chunk;

    ret = I2CSPM_Transfer(i2c, &seq);
    if (ret != i2cTransferDone)
    {
      return((int) ret);
    }

    /* Update counters etc */
    data   += chunk;
    offset += chunk;
    len    -= chunk;

    /* Do acknowledge polling waiting for write process to finish in EEPROM */
    tmp = ee_m24c08AckPoll(i2c, addr);
    if (tmp)
    {
      return(tmp);
    }
  }

  return((int) len);
}


#endif
