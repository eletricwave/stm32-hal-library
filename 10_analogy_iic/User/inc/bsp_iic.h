#ifndef __BSP_IIC_H__
#define __BSP_IIC_H__

#define IICx                        I2C1
#define IICx_CLK_ENABLE()           __HAL_RCC_I2C1_CLK_ENABLE()
#define IICx_SDA_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define IICx_SCL_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()

#define IICx_SDA_PIN                GPIO_PIN_7
#define IICx_SCL_PIN                GPIO_PIN_6

#define IICx_SDA_PORT               GPIOB
#define IICx_SCL_PORT               GPIOB

#define I2Cx_FORCE_RESET()           __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()         __HAL_RCC_I2C1_RELEASE_RESET()


void IIC_Init(void);



#endif /* __BSP_IIC_H__ */
