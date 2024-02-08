#ifndef __BSP_SYSTICK_H__
#define __BSP_SYSTICK_H__

void sysTick_Init(void);
void decrease_current_time(void);
void delay_ms(unsigned int);
void delay_us(unsigned int);

#endif /* __BSP_SYSTICK_H__ */

