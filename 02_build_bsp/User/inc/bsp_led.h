#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#define LED_PORT 	GPIOB
#define LED_PIN		(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5)

#define LED_R_ON		do { HAL_GPIO_WritePin(LED_PORT, GPIO_PIN_5, GPIO_PIN_RESET); } while(0)
#define LED_R_OFF		do { HAL_GPIO_WritePin(LED_PORT, GPIO_PIN_5, GPIO_PIN_SET); } while(0)
#define LED_R_TOGGLE	do { HAL_GPIO_TogglePin(LED_PORT, GPIO_PIN_5); } while(0)

#define LED_G_ON		do { HAL_GPIO_WritePin(LED_PORT, GPIO_PIN_0, GPIO_PIN_RESET); } while(0)
#define LED_G_OFF		do { HAL_GPIO_WritePin(LED_PORT, GPIO_PIN_0, GPIO_PIN_SET); } while(0)
#define LED_G_TOGGLE	do { HAL_GPIO_TogglePin(LED_PORT, GPIO_PIN_0); } while(0)

#define LED_B_ON		do { HAL_GPIO_WritePin(LED_PORT, GPIO_PIN_1, GPIO_PIN_RESET); } while(0)
#define LED_B_OFF		do { HAL_GPIO_WritePin(LED_PORT, GPIO_PIN_1, GPIO_PIN_SET); } while(0)
#define LED_B_TOGGLE	do { HAL_GPIO_TogglePin(LED_PORT, GPIO_PIN_1); } while(0)

void LED_Init(void); 

#endif /* __BSP_LED_H__ */
