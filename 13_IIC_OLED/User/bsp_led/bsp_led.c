#include "bsp_led.h"

#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"

void LED_Init(void){
	GPIO_InitTypeDef LED_INIT_STRUCT;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	LED_INIT_STRUCT.Mode 	= GPIO_MODE_OUTPUT_PP;
	LED_INIT_STRUCT.Pin 	= LED_PIN;
	LED_INIT_STRUCT.Pull	= GPIO_NOPULL;
	LED_INIT_STRUCT.Speed	= GPIO_SPEED_FREQ_LOW;
	
	HAL_GPIO_Init(LED_PORT, &LED_INIT_STRUCT);
}
