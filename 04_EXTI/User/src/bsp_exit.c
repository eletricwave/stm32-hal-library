#include "stm32f1xx.h"
#include "bsp_exit.h"

void KEY1_EXIT_INIT(void){
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitTypeDef EXIT_Struct = {0};
	EXIT_Struct.Pin 	= KEY1_EXIT_PIN;
	EXIT_Struct.Mode 	= KEY1_EXIT_MODE;
	EXIT_Struct.Pull	= KEY1_EXIT_PULL;
	
	HAL_GPIO_Init(KEY1_EXIT_PORT, &EXIT_Struct);
	
	HAL_NVIC_SetPriority(KEY1_EXIT_IRQ, 1, 0);
	HAL_NVIC_EnableIRQ(KEY1_EXIT_IRQ);
}

void KEY2_EXIT_INIT(void){
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	GPIO_InitTypeDef EXIT_Struct = {0};
	EXIT_Struct.Pin 	= KEY2_EXIT_PIN;
	EXIT_Struct.Mode 	= KEY2_EXIT_MODE;
	EXIT_Struct.Pull	= KEY2_EXIT_PULL;
	
	HAL_GPIO_Init(KEY2_EXIT_PORT, &EXIT_Struct);
	
	HAL_NVIC_SetPriority(KEY2_EXIT_IRQ, 1, 0);
	HAL_NVIC_EnableIRQ(KEY2_EXIT_IRQ);
}
