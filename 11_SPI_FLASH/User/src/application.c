#include "stm32f1xx.h"
#include "bsp_led.h"
#include "application.h"
#include "bsp_uart.h"

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

void app_Init(void){
	uint8_t cmd;
	printf("cmd = 0x%02x\r\n", cmd);
	if (cmd < 0x01 || cmd > 0x04){
		printf("cmd:0x%02X to small or to large\r\n", cmd); 
	}
	else{
		cur_mode = cmd;
	}
}

void change_mode_by_uart_info(void){
	uint8_t cmd = *get_uart_recv_data();
	printf("cmd = 0x%02x\r\n", cmd);
	if (get_uart_recv_len() > 1){
		printf("size:%d too long \r\n", get_uart_recv_len());
	}
	else{
		if (cmd < 0x01 || cmd > 0x04) return;
		cur_mode = cmd;
	}
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
	}
}

