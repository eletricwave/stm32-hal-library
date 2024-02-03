#include "iic_eeprom.h"
#include "stm32f1xx.h"
#include "bsp_uart.h"

extern I2C_HandleTypeDef IIC_Handle;
extern I2C_HandleTypeDef hi2c1;

void IIC_Write_Byte(uint16_t WriteAddr, uint8_t* dat){
    if (HAL_ERROR ==  HAL_I2C_Mem_Write(&IIC_Handle, EEPROM_ADDR, WriteAddr, I2C_MEMADD_SIZE_8BIT, dat, 1, 1000)){
		printf("fail to HAL_I2C_Mem_Write in %s and line: %d\r\n", __func__, __LINE__);
		return;
	}
	
	while (HAL_I2C_GetState(&IIC_Handle) != HAL_I2C_STATE_READY);   // wait for ready status
	
	while (HAL_I2C_IsDeviceReady(&IIC_Handle, EEPROM_ADDR, 10, 100) == HAL_TIMEOUT);
	
	while (HAL_I2C_GetState(&IIC_Handle) != HAL_I2C_STATE_READY);
}

HAL_StatusTypeDef  IIC_Read_Byte(uint16_t ReadAddr, uint8_t* buf){
	if (HAL_ERROR == HAL_I2C_Mem_Read(&IIC_Handle, EEPROM_ADDR, ReadAddr, I2C_MEMADD_SIZE_8BIT, buf, 1, 1000)){
		printf("fail to HAL_I2C_Mem_Read in %s and line: %d\r\n", __func__, __LINE__);
		return HAL_ERROR;
	}
	return HAL_OK;
}


