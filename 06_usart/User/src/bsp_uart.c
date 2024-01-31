#include "stm32f1xx.h"
#include "bsp_uart.h"
#include <string.h>
#include <stdio.h>
#include "bsp_led.h"

UART_HandleTypeDef UART_Handle;

void UART_Init(void){
	UART_Handle.Instance		= UART_INSTANCE;
	
	UART_Handle.Init.BaudRate 	= UART_BAUD;
	UART_Handle.Init.WordLength = UART_WORDLENGTH_8B;
	UART_Handle.Init.Parity		= UART_PARITY_NONE;
	UART_Handle.Init.StopBits	= UART_STOPBITS_1;
	UART_Handle.Init.Mode		= UART_MODE_TX_RX;
	UART_Handle.Init.HwFlowCtl	= UART_HWCONTROL_NONE;
	
	HAL_UART_Init(&UART_Handle);
	__HAL_UART_ENABLE_IT(&UART_Handle, UART_IT_RXNE);
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (uartHandle->Instance == UART_INSTANCE){
		// enable clock
		UART_CLK_ENABLE();
		UART_TX_CLK_ENABLE();
		UART_RX_CLK_ENABLE();
		
		// init gpio
		GPIO_InitStruct.Pin 	= UART_TX_PIN;
		GPIO_InitStruct.Mode 	= GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Pull	= GPIO_PULLUP;
		HAL_GPIO_Init(UART_TX_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin		= UART_RX_PIN;
		GPIO_InitStruct.Mode	= GPIO_MODE_AF_INPUT;
		HAL_GPIO_Init(UART_RX_PORT, &GPIO_InitStruct);
		
		HAL_NVIC_SetPriority(UART_IQR, 0, 0);
		HAL_NVIC_EnableIRQ(UART_IQR);
	}

}


void UART_SendStr(unsigned char *str){
	int i = 0;
	do{
		HAL_UART_Transmit(&UART_Handle, (unsigned char*)(str + i), 1, 1000);
	}while(*(str + i++) != '\0');
}

int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&UART_Handle, (unsigned char*)&ch, 1, 1000);
	return (ch);
}

int fgetc(FILE *f){
	int ch;
	HAL_UART_Receive(&UART_Handle, (unsigned char*)&ch ,1, 1000);
	return (ch);
}
