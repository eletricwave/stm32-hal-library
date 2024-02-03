#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include <stdio.h>
#include "stm32f1xx.h"

#define UART_INSTANCE		USART1
#define UART_BAUD			115200

#define UART_CLK_ENABLE()		__HAL_RCC_USART1_CLK_ENABLE()
#define UART_TX_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
#define UART_RX_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
#define UART_DMA_CLK_ENABLE()   __HAL_RCC_DMA1_CLK_ENABLE()

#define UART_TX_PIN			GPIO_PIN_9
#define UART_RX_PIN			GPIO_PIN_10

#define UART_TX_PORT		GPIOA
#define UART_RX_PORT		GPIOA

#define UART_IQR			USART1_IRQn
#define UART_DMA_IQR        DMA1_Channel5_IRQn

#define UART_BUFFER_LEN          40

typedef struct __recv_data_info{
    uint8_t data[UART_BUFFER_LEN];
    uint8_t data_len;
}uartRecvData;

void UART_Init(void);
void UART_SendStr(unsigned char *);
uint8_t *get_uart_recv_data(void);
uint8_t get_uart_recv_len(void);
void handle_rece_info(void);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);

#endif /* __BSP_UART_H__ */

