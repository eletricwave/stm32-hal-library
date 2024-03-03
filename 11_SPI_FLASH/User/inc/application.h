#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#define CUR_MODE_ADDR	0x00


void app_loop(void);
void app_Init(void);
void change_mode_by_uart_info(void);

#endif /* __APPLICATION_H__ */
