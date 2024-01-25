#include "stm32f1xx.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "application.h"

static int cur_mode = 1;

void set_cur_mode(int mode){
	cur_mode = mode;
}

int get_cur_mode(void){
	return cur_mode;
}

void close_all_led(void){
	LED_R_OFF;
	LED_G_OFF;
	LED_B_OFF;
}

void app_loop(void){
	close_all_led();
	while (1){
		switch (cur_mode){
			case 1:{
				LED_R_ON;
				HAL_Delay(500);
				LED_R_OFF;
				HAL_Delay(500);
			}
			break;
			
			case 2:{
				LED_G_ON;
				HAL_Delay(500);
				LED_G_OFF;
				HAL_Delay(500);
			}
			break;
			
			case 3:{
				LED_B_ON;
				HAL_Delay(500);
				LED_B_OFF;
				HAL_Delay(500);
			}
			break;
			
			case 4:{
				LED_R_ON;
				HAL_Delay(500);
				LED_R_OFF;
				LED_G_ON;
				HAL_Delay(500);
				LED_G_OFF;
				LED_B_ON;
				HAL_Delay(500);
				LED_B_OFF;
			}
			break;

			default: {
				LED_R_OFF;
				LED_G_OFF;
				LED_B_OFF;
			}
			break;
		}
		
		if (KEY_PRESS == SCAN_KEY1){
			if (++cur_mode == 5)
				cur_mode = 1;
			close_all_led();
			continue;
		}
		if (KEY_PRESS == SCAN_KEY2){
			if (--cur_mode == 0)
				cur_mode = 4;
			close_all_led();
			continue;
		}
	}
}

