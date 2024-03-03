#ifndef __BSP_FLASH_H__
#define __BSP_FLASH_H__

#include "stm32f1xx.h"

#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 
#define SPIx_CS_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE() 

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_5
#define SPIx_SCK_GPIO_PORT               GPIOA

#define SPIx_MISO_PIN                    GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT              GPIOA

#define SPIx_MOSI_PIN                    GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT              GPIOA

#define FLASH_CS_PIN                     GPIO_PIN_0              
#define FLASH_CS_GPIO_PORT               GPIOC  

#define	digitalHi(p,i)			    {p->BSRR=i;}			  //ÉèÖÃÎª¸ßµçÆ½		
#define digitalLo(p,i)			    {p->BSRR=(uint32_t)i << 16;}				//Êä³öµÍµçÆ½
#define SPI_FLASH_CS_LOW()      digitalLo(FLASH_CS_GPIO_PORT,FLASH_CS_PIN )
#define SPI_FLASH_CS_HIGH()     digitalHi(FLASH_CS_GPIO_PORT,FLASH_CS_PIN )


#define SPIT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT         ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))

#define W25X_DeviceID			        0xab
#define W25X_JEDECID                    0x9f
#define W25X_ENABLE                     0x06
#define W25X_DISABLE                    0x04
#define W25X_STATE                      0x05
#define W25X_SECTORERASE				0x20
#define W25X_BLOCKERASE					0x52
#define W25X_PAGEWRITE					0x02
#define W25X_RAED						0x03
#define Dummy_Byte                      0xff

#define SPI_FLASH_PAGE_NUM				256

void SPI_FLASH_Init(void);
uint8_t SPI_FLASH_ReadByte(void);
uint8_t SPI_FLASH_SendReadByte(uint8_t byte);
void FLASH_Write_Byte(uint8_t byte);
void FLASH_Read_Byte(uint8_t *byte);

uint8_t SPI_FLASH_ReadDeviceID(void);
uint32_t SPI_FLASH_ReadID(void);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WriteDisable(void);
void SPI_FLASH_WaitForReady(void);
void SPI_FLASH_SectorErase(uint32_t addr);
void SPI_FLASH_WritePage(uint8_t *dat, uint32_t addr, uint32_t num);
void SPI_FLASH_ReadPage(uint8_t *dat, uint32_t addr, uint32_t num);

#endif /* __BSP_FLASH_H__ */

