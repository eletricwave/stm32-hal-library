#ifndef __BSP_AIIC_H__
#define __BSP_AIIC_H__

#define AIIC_SDA_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define AIIC_SCL_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()

#define AIIC_SDA_PORT               GPIOB
#define AIIC_SCL_PORT               GPIOB

#define AIIC_SDA_PIN                GPIO_PIN_7
#define AIIC_SCL_PIN                GPIO_PIN_6

#define AIIC_SDA_H                  HAL_GPIO_WritePin(AIIC_SDA_PORT, AIIC_SDA_PIN, GPIO_PIN_SET)
#define AIIC_SDA_L                  HAL_GPIO_WritePin(AIIC_SDA_PORT, AIIC_SDA_PIN, GPIO_PIN_RESET)

#define AIIC_SCL_H                  HAL_GPIO_WritePin(AIIC_SCL_PORT, AIIC_SCL_PIN, GPIO_PIN_SET)
#define AIIC_SCL_L                  HAL_GPIO_WritePin(AIIC_SCL_PORT, AIIC_SCL_PIN, GPIO_PIN_RESET)

#define AIIC_READ_SDA               HAL_GPIO_ReadPin(AIIC_SDA_PORT, AIIC_SDA_PIN)

void AIIC_Init(void);
void AIIC_Start(void);
void AIIC_Stop(void);
void AIIC_ACK(void);
void AIIC_NACK(void);
uint8_t AIIC_Wait_ACK(void);
void AIIC_Send_Byte(uint8_t);
uint8_t AIIC_Read_Byte(uint8_t);

#endif /* __BSP_AIIC_H__ */
