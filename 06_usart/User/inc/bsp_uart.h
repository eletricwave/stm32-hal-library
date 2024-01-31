#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include <stdio.h>
#include "stm32f1xx.h"

#define UART_INSTANCE		USART1
#define UART_BAUD			115200

#define UART_CLK_ENABLE()		__HAL_RCC_USART1_CLK_ENABLE()
#define UART_TX_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
#define UART_RX_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()

#define UART_TX_PIN			GPIO_PIN_9
#define UART_RX_PIN			GPIO_PIN_10

#define UART_TX_PORT		GPIOA
#define UART_RX_PORT		GPIOA

#define UART_IQR			USART1_IRQn


void UART_Init(void);
void UART_SendStr(unsigned char *);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);

#endif /* __BSP_UART_H__ */

