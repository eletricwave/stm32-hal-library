#include "stm32f1xx.h"
#include "bsp_uart.h"
#include <string.h>
#include <stdio.h>
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_dma.h"
#include "application.h"

UART_HandleTypeDef 	UART_Handle;
DMA_HandleTypeDef	UART_DMA_Handle;

static uartRecvData recv_info = {0};

uint8_t *get_uart_recv_data(void){
	return recv_info.data;
}

uint8_t get_uart_recv_len(void){
	return recv_info.data_len;
}

void handle_rece_info(void){
	HAL_UART_DMAStop(&UART_Handle);
	recv_info.data_len = UART_BUFFER_LEN - __HAL_DMA_GET_COUNTER(&UART_DMA_Handle);
	recv_info.data[recv_info.data_len] = '\0';
	printf("recv data length: %d, info: %s\r\n", get_uart_recv_len(), get_uart_recv_data());
	HAL_UART_Receive_DMA(&UART_Handle, get_uart_recv_data(), UART_BUFFER_LEN);
	change_mode_by_uart_info();
}

void UART_Init(void){
	UART_Handle.Instance		= UART_INSTANCE;
	
	// init uart
	UART_Handle.Init.BaudRate 	= UART_BAUD;
	UART_Handle.Init.WordLength = UART_WORDLENGTH_8B;
	UART_Handle.Init.Parity		= UART_PARITY_NONE;
	UART_Handle.Init.StopBits	= UART_STOPBITS_1;
	UART_Handle.Init.Mode		= UART_MODE_TX_RX;
	UART_Handle.Init.HwFlowCtl	= UART_HWCONTROL_NONE;
	
	HAL_UART_Init(&UART_Handle);
	// __HAL_UART_ENABLE_IT(&UART_Handle, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&UART_Handle, UART_IT_IDLE); //open idle interrupt
	HAL_UART_Receive_DMA(&UART_Handle, recv_info.data, UART_BUFFER_LEN);	// open dma receive
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (uartHandle->Instance == UART_INSTANCE){
		// enable clock
		UART_CLK_ENABLE();
		UART_TX_CLK_ENABLE();
		UART_RX_CLK_ENABLE();
		UART_DMA_CLK_ENABLE();
		
		// init gpio
		GPIO_InitStruct.Pin 	= UART_TX_PIN;
		GPIO_InitStruct.Mode 	= GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Pull	= GPIO_PULLUP;
		HAL_GPIO_Init(UART_TX_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin		= UART_RX_PIN;
		GPIO_InitStruct.Mode	= GPIO_MODE_AF_INPUT;
		HAL_GPIO_Init(UART_RX_PORT, &GPIO_InitStruct);
		
		// dma init
		UART_DMA_Handle.Instance 					= DMA1_Channel5;
		UART_DMA_Handle.Init.Direction 				= DMA_PERIPH_TO_MEMORY;
		UART_DMA_Handle.Init.PeriphInc 				= DMA_PINC_DISABLE;
		UART_DMA_Handle.Init.MemInc					= DMA_MINC_ENABLE;
		UART_DMA_Handle.Init.PeriphDataAlignment	= DMA_PDATAALIGN_BYTE;
		UART_DMA_Handle.Init.MemDataAlignment		= DMA_MDATAALIGN_BYTE;
		UART_DMA_Handle.Init.Mode					= DMA_CIRCULAR;
		UART_DMA_Handle.Init.Priority				= DMA_PRIORITY_LOW;
		HAL_DMA_Init(&UART_DMA_Handle);
		
		__HAL_LINKDMA(uartHandle, hdmarx, UART_DMA_Handle);
		
		HAL_NVIC_SetPriority(UART_IQR, 0, 0);
		HAL_NVIC_EnableIRQ(UART_IQR);
		
		HAL_NVIC_SetPriority(UART_DMA_IQR, 0, 0);
		HAL_NVIC_EnableIRQ(UART_DMA_IQR);
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
