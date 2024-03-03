#include "bsp_flash.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include <stdio.h>

SPI_HandleTypeDef spiHandle;

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  SPIx_SCK_GPIO_CLK_ENABLE();
  SPIx_MISO_GPIO_CLK_ENABLE();
  SPIx_MOSI_GPIO_CLK_ENABLE();
  SPIx_CS_GPIO_CLK_ENABLE();
  /* Enable SPI clock */
  SPIx_CLK_ENABLE(); 
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* SPI SCK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

  
  HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);
    
  /* SPI MISO GPIO pin configuration  */
  GPIO_InitStruct.Pin = SPIx_MISO_PIN;  
  HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);
  
  /* SPI MOSI GPIO pin configuration  */
  GPIO_InitStruct.Pin = SPIx_MOSI_PIN; 
  HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);   

  GPIO_InitStruct.Pin = FLASH_CS_PIN ;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(FLASH_CS_GPIO_PORT, &GPIO_InitStruct); 
}

void SPI_FLASH_Init(void)
{
   /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  spiHandle.Instance               = SPIx;
  spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  spiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  spiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
  spiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  spiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  spiHandle.Init.CRCPolynomial     = 7;
  spiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  spiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  spiHandle.Init.NSS               = SPI_NSS_SOFT;
  spiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  
  spiHandle.Init.Mode = SPI_MODE_MASTER;

  HAL_SPI_Init(&spiHandle); 
  
  __HAL_SPI_ENABLE(&spiHandle);     
}

uint8_t SPI_FLASH_SendReadByte(uint8_t byte)
{
	uint8_t rDat;
	HAL_SPI_TransmitReceive(&spiHandle, &byte, &rDat, 1,1000);
	
	return rDat;
}

void FLASH_Write_Byte(uint8_t byte){
    HAL_SPI_Transmit(&spiHandle, &byte, 1, 1000);
}

void FLASH_Read_Byte(uint8_t *byte){
    HAL_SPI_Receive(&spiHandle, byte, 1, 1000);
}

uint8_t SPI_FLASH_ReadByte(void){
  return (SPI_FLASH_SendReadByte(Dummy_Byte));
}

uint8_t SPI_FLASH_ReadDeviceID(void){
    uint8_t Temp = 0;

  /* Select the FLASH: Chip Select low */
    SPI_FLASH_CS_LOW();

  /* Send "RDID " instruction */
    FLASH_Write_Byte(W25X_DeviceID);
    // dummy data
    FLASH_Read_Byte(&Temp);
    FLASH_Read_Byte(&Temp);
    FLASH_Read_Byte(&Temp);

    FLASH_Read_Byte(&Temp);

    SPI_FLASH_CS_HIGH();

  return Temp;
}

uint32_t SPI_FLASH_ReadID(void){
    uint32_t dat = 0;
    uint8_t c1, c2, c3;
    SPI_FLASH_CS_LOW();

    FLASH_Write_Byte(W25X_JEDECID);

    FLASH_Read_Byte(&c1);
    FLASH_Read_Byte(&c2);
    FLASH_Read_Byte(&c3);

    SPI_FLASH_CS_HIGH();

    dat = ((uint32_t)c1 << 16) | ((uint32_t)c2 << 8) | c3;
    return dat;
}

void SPI_FLASH_WriteEnable(void){
    SPI_FLASH_CS_LOW();
    FLASH_Write_Byte(W25X_ENABLE);
    SPI_FLASH_CS_HIGH();
}

void SPI_FLASH_WriteDisable(void){
    SPI_FLASH_CS_LOW();
    FLASH_Write_Byte(W25X_DISABLE);
    SPI_FLASH_CS_HIGH();
}

void SPI_FLASH_WaitForReady(void){
    uint8_t reg;
	SPI_FLASH_CS_LOW();
    do {
        FLASH_Write_Byte(W25X_STATE);
        FLASH_Read_Byte(&reg);
    }while (reg & 0x01);   // 比较最低位的 BUSY 标志
	SPI_FLASH_CS_HIGH();
}

void SPI_FLASH_SectorErase(uint32_t addr){

	SPI_FLASH_WriteEnable();
	
	SPI_FLASH_CS_LOW();
	FLASH_Write_Byte(W25X_SECTORERASE);
	FLASH_Write_Byte((addr >> 16) & 0xff);
	FLASH_Write_Byte((addr >> 8) & 0xff);
	FLASH_Write_Byte(addr & 0xff);

	SPI_FLASH_CS_HIGH();
	SPI_FLASH_WaitForReady();	
}

void SPI_FLASH_WritePage(uint8_t *dat, uint32_t addr, uint32_t num){
	
	if (num > SPI_FLASH_PAGE_NUM) return;
	
	SPI_FLASH_WriteEnable();
	SPI_FLASH_WaitForReady();
	
	SPI_FLASH_CS_LOW();
	
	FLASH_Write_Byte(W25X_PAGEWRITE);
	FLASH_Write_Byte((addr >> 16) & 0xff);
	FLASH_Write_Byte((addr >> 8) & 0xff);
	FLASH_Write_Byte(addr & 0xff);
	
	while (num--){
		FLASH_Write_Byte(*dat);
		dat++;
	}
	
	SPI_FLASH_CS_HIGH();
}


void SPI_FLASH_ReadPage(uint8_t *dat, uint32_t addr, uint32_t num){
	
	SPI_FLASH_WriteEnable();
	SPI_FLASH_WaitForReady();
	
	SPI_FLASH_CS_LOW();
	FLASH_Write_Byte(W25X_RAED);
	FLASH_Write_Byte((addr >> 16) & 0xff);
	FLASH_Write_Byte((addr >> 8) & 0xff);
	FLASH_Write_Byte(addr & 0xff);
	while (num--){
		FLASH_Read_Byte(dat);
		dat++;
	}
	SPI_FLASH_CS_HIGH();
}
