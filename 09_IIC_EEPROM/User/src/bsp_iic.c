#include "bsp_iic.h"
#include "stm32f1xx.h"

I2C_HandleTypeDef IIC_Handle;

void IIC_Init(void){
	IIC_Handle.Instance = IICx;
	
	IIC_Handle.Init.ClockSpeed			= 400000;
	IIC_Handle.Init.DutyCycle			= I2C_DUTYCYCLE_2;
	IIC_Handle.Init.OwnAddress1     	= 0xcc;
	IIC_Handle.Init.AddressingMode 		= I2C_ADDRESSINGMODE_7BIT;
	IIC_Handle.Init.DualAddressMode		= I2C_DUALADDRESS_DISABLE;
	IIC_Handle.Init.OwnAddress2     	= 0x00; 
	IIC_Handle.Init.GeneralCallMode		= I2C_GENERALCALL_DISABLE;
	IIC_Handle.Init.NoStretchMode		= I2C_NOSTRETCH_DISABLE;
	
	
	HAL_I2C_Init(&IIC_Handle);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	IICx_CLK_ENABLE();
	IICx_SCL_CLK_ENABLE();
	IICx_SDA_CLK_ENABLE();
	
	GPIO_InitStruct.Pin 	= IICx_SCL_PIN;
	GPIO_InitStruct.Mode	= GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull	= GPIO_NOPULL;
	GPIO_InitStruct.Speed	= GPIO_SPEED_FREQ_HIGH;
	
	HAL_GPIO_Init(IICx_SCL_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin		= IICx_SDA_PIN;
	HAL_GPIO_Init(IICx_SDA_PORT, &GPIO_InitStruct);
	
    I2Cx_FORCE_RESET();
    I2Cx_RELEASE_RESET();
}



