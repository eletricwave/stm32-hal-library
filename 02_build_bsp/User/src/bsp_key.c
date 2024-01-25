#include "bsp_key.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"

void KEY_Init(void){
	GPIO_InitTypeDef KEY_INIT_STRUCT;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	KEY_INIT_STRUCT.Mode 	= GPIO_MODE_INPUT;
	KEY_INIT_STRUCT.Pin		= KEY1_PIN;
	KEY_INIT_STRUCT.Pull	= GPIO_PULLDOWN;
	KEY_INIT_STRUCT.Speed	= GPIO_SPEED_FREQ_HIGH;
	
	HAL_GPIO_Init(KEY1_PORT, &KEY_INIT_STRUCT);
	KEY_INIT_STRUCT.Pin 	= KEY2_PIN;
	HAL_GPIO_Init(KEY2_PORT, &KEY_INIT_STRUCT);
}

KEY_Status Scan_Key(GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_x){
	if (KEY_PRESS == HAL_GPIO_ReadPin(GPIOx, GPIO_PIN_x)){
		HAL_Delay(20);
		if (KEY_PRESS == HAL_GPIO_ReadPin(GPIOx, GPIO_PIN_x)){
			while (KEY_PRESS == HAL_GPIO_ReadPin(GPIOx, GPIO_PIN_x));
			return KEY_PRESS;
		}
	}
	
	return KEY_RELEASE;
}
