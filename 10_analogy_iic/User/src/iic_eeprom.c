#include "iic_eeprom.h"
#include "stm32f1xx.h"
#include "bsp_uart.h"
#include "bsp_aiic.h"
#include "bsp_systick.h"
#include <stdio.h>

extern I2C_HandleTypeDef IIC_Handle;
extern I2C_HandleTypeDef hi2c1;

void IIC_Write_Byte(uint8_t WriteAddr, uint8_t* dat){
	printf("write data:0x%02x\r\n", *dat);
    AIIC_Start();
	AIIC_Send_Byte(EEPROM_ADDR);
	
	// printf("11111\r\n");
	if (!AIIC_Wait_ACK()){
		printf("dev addr Nack\r\n");
	}
	
	// printf("22222\r\n");
	AIIC_Send_Byte(WriteAddr);
	
	// printf("333333\r\n");
	if (!AIIC_Wait_ACK()){
		printf("eeprom addr Nack\r\n");
	}
	
	// printf("44444\r\n");
	AIIC_Send_Byte(*dat);
	
	// printf("55555\r\n");
	if (!AIIC_Wait_ACK()){
		printf("send dat Nack\r\n");
	}
	
	// printf("66666\r\n");
	AIIC_Stop();
	
	delay_us(10);		//wait for write complete
	
}

HAL_StatusTypeDef  IIC_Read_Byte(uint8_t ReadAddr, uint8_t* buf){
	
	AIIC_Start();
	
	AIIC_Send_Byte(EEPROM_ADDR);
	
	if (!AIIC_Wait_ACK()){
		printf("dev addr Nack\r\n");
	}
	
	AIIC_Send_Byte(ReadAddr);
	
	if (!AIIC_Wait_ACK()){
		printf("eeprom addr Nack\r\n");
	}
	
	*buf = AIIC_Read_Byte(0);
	
	AIIC_Stop();
	
	return HAL_OK;
}


