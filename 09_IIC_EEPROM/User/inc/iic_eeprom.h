#ifndef __IIC_EEPROM_H__
#define __IIC_EEPROM_H__

#include "stm32f1xx.h"

#define EEPROM_PAGE_SIZE        8
#define EEPROM_PAGE_NUM         256
#define EEPROM_ADDR             0xa0

void IIC_Write_Byte(uint16_t, uint8_t*);
HAL_StatusTypeDef  IIC_Read_Byte(uint16_t, uint8_t*);

#endif /* __IIC_EEPROM_H__ */
