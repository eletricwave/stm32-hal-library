#ifndef __BSP_KEY_H__
#define __BSP_KEY_H__

#include "stm32f1xx.h"

#define KEY1_PORT 	GPIOA
#define KEY1_PIN	GPIO_PIN_0

#define KEY2_PORT 	GPIOC
#define KEY2_PIN 	GPIO_PIN_13

#define SCAN_KEY1	Scan_Key(KEY1_PORT, KEY1_PIN)
#define SCAN_KEY2	Scan_Key(KEY2_PORT, KEY2_PIN)

typedef enum{
	KEY_RELEASE = 0U,
	KEY_PRESS,
}KEY_Status;


void KEY_Init(void);
KEY_Status Scan_Key(GPIO_TypeDef*, uint16_t);

#endif /* __BSP_KEY_H__ */

