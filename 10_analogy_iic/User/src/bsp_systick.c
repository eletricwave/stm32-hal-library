#include "bsp_systick.h"
#include "stm32f1xx.h"

volatile unsigned long CurDelayTime = 0;

void sysTick_Init(void){
	while(HAL_SYSTICK_Config(SystemCoreClock / 1000000));  // per 1us a interrupt
}

void decrease_current_time(void){
	if(CurDelayTime)
		--CurDelayTime;
}

void delay_ms(unsigned int t){
	CurDelayTime = t * 1000;
	while (CurDelayTime);
}

void delay_us(unsigned int t){
	CurDelayTime = t;
	while (CurDelayTime);
}
