#include "bsp_dma_mtom.h"
#include "stm32f1xx.h"


DMA_HandleTypeDef DMA_Handle = {0};

void DMA_MTOT_Init(void){
	__HAL_RCC_DMA1_CLK_ENABLE();
	
	DMA_Handle.Instance 				= DMA1_Channel1;
	DMA_Handle.Init.Direction 			= DMA_MEMORY_TO_MEMORY;
	DMA_Handle.Init.MemDataAlignment	= DMA_MDATAALIGN_BYTE;
	DMA_Handle.Init.MemInc				= DMA_MINC_ENABLE;
	DMA_Handle.Init.Mode				= DMA_NORMAL;
	DMA_Handle.Init.PeriphDataAlignment	= DMA_PDATAALIGN_BYTE;
	DMA_Handle.Init.PeriphInc			= DMA_PINC_ENABLE;
	DMA_Handle.Init.Priority			= DMA_PRIORITY_MEDIUM;
	
	HAL_DMA_Init(&DMA_Handle);
}


