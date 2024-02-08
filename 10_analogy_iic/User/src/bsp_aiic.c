#include "stm32f1xx.h"
#include "bsp_aiic.h"
#include "bsp_systick.h"

void AIIC_Init(void){
	AIIC_SDA_CLK_ENABLE();
	//AIIC_SCL_CLK_ENABLE();
	
	GPIO_InitTypeDef InitStruct;
	
	InitStruct.Pin		= AIIC_SCL_PIN;
	// InitStruct.Pull		= GPIO_PULLUP;
	InitStruct.Pull 	= GPIO_NOPULL;
	InitStruct.Mode		= GPIO_MODE_OUTPUT_PP;
	InitStruct.Speed	= GPIO_SPEED_FREQ_HIGH;
	
	HAL_GPIO_Init(AIIC_SCL_PORT, &InitStruct);
	
	InitStruct.Pin 		= AIIC_SDA_PIN;
	InitStruct.Mode		= GPIO_MODE_OUTPUT_OD;
	HAL_GPIO_Init(AIIC_SDA_PORT, &InitStruct);
}

void AIIC_Start(void){
	AIIC_SDA_H;
	AIIC_SCL_H;
	delay_us(2);
	AIIC_SDA_L;
	
	delay_us(2);
	AIIC_SCL_L;
	delay_us(2);
}

void AIIC_Stop(void){
	AIIC_SDA_L;
	AIIC_SCL_H;
	delay_us(2);
	AIIC_SDA_H;
	delay_us(2);
}

void AIIC_ACK(void){
	AIIC_SCL_L;
	delay_us(2);
	AIIC_SDA_L;
	delay_us(2);
	AIIC_SCL_H;
	delay_us(2);
}

void AIIC_NACK(void){
	AIIC_SCL_L;
	delay_us(2);
	AIIC_SDA_H;
	delay_us(2);
	AIIC_SCL_H;
	delay_us(2);
}

// success return 1 and fail to return 0
uint8_t AIIC_Wait_ACK(void){
	AIIC_SDA_H;  	// release sda
	delay_us(2);
	AIIC_SCL_H;
	delay_us(2);
	
	if (AIIC_READ_SDA){
		AIIC_Stop();
		return 0;
	}
	AIIC_SCL_L;
	delay_us(2);
	return 1;
}

void AIIC_Send_Byte(uint8_t dat){
	uint8_t idx = 8;
	while (idx--){
		if ((dat >> idx) & 0x01)	AIIC_SDA_H;
		else						AIIC_SDA_L;
		delay_us(2);
		AIIC_SCL_H;
		delay_us(2);
		AIIC_SCL_L;
	}
	AIIC_SDA_H;
}

uint8_t AIIC_Read_Byte(uint8_t ack){
	uint8_t dat = 0x00, idx = 8;
	while (idx--){
		AIIC_SCL_H;
		delay_us(2);
		if (AIIC_READ_SDA)	dat |= (0x01 << idx);
		AIIC_SCL_L;
		delay_us(2);
	}
	
	if (ack)	AIIC_ACK();
	else		AIIC_NACK();
	
	return dat;
}
