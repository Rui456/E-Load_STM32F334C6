#include "dac.h"


#define DAC_DHR12R2_ADDRESS      0x40007414
#define DAC_DHR8R1_ADDRESS       0x40007410

uint32_t dac_data[3];

void dac_init()
{
	DAC_InitTypeDef DAC_InitStructure;
	DMA_InitTypeDef	DMA_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	/* Enable GPIOA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	/* DMA2 clock enable (to be used with DAC) */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	
/******/	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
/******/	

/******/	
	/* DAC1 channel1 Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits2_0;
	DAC_InitStructure.DAC_Buffer_Switch = DAC_BufferSwitch_Disable;
	DAC_Init(DAC1, DAC_Channel_1, &DAC_InitStructure);
	
	/* Enable DAC1 Channel1 */
	DAC_Cmd(DAC1, DAC_Channel_1, ENABLE);
/******/	

/******/	
	/* DMA2 channel3 configuration */
	DMA_DeInit(DMA2_Channel3); 
	DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12R2_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&dac_data[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 32;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Channel3, &DMA_InitStructure);

	/* Enable DMA2 Channel3 */
	DMA_Cmd(DMA2_Channel3, ENABLE);

	/* Enable DMA for DAC Channel2 */
	DAC_DMACmd(DAC1, DAC_Channel_2, ENABLE);
/******/	

/******/	
	/* DAC channel1 Configuration */
	DAC_InitStructure.DAC_Buffer_Switch = DAC_BufferSwitch_Disable;
	DAC_Init(DAC1, DAC_Channel_1, &DAC_InitStructure);

	/* DMA2 channel4 configuration */
	DMA_DeInit(DMA2_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR8R1_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&dac_data[1];
	DMA_InitStructure.DMA_BufferSize = 6;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_Init(DMA2_Channel4, &DMA_InitStructure);

	/* Enable DMA2 Channel4 */
	DMA_Cmd(DMA2_Channel4, ENABLE);

	/* Enable DAC1 Channel1: Once the DAC1 channel1 is enabled, PA.05 is 
	 automatically connected to the DAC converter. */
	DAC_Cmd(DAC1, DAC_Channel_1, ENABLE);

	/* Enable DMA for DAC Channel1 */
	DAC_DMACmd(DAC1, DAC_Channel_1, ENABLE);
/******/	

/******/	
	/* DAC channel1 Configuration */
	DAC_InitStructure.DAC_Buffer_Switch = DAC_BufferSwitch_Disable;
	DAC_Init(DAC1, DAC_Channel_1, &DAC_InitStructure);

	/* DMA2 channel4 configuration */
	DMA_DeInit(DMA2_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR8R1_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&dac_data[1];
	DMA_InitStructure.DMA_BufferSize = 6;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_Init(DMA2_Channel4, &DMA_InitStructure);

	/* Enable DMA2 Channel4 */
	DMA_Cmd(DMA2_Channel4, ENABLE);

	/* Enable DAC1 Channel1: Once the DAC1 channel1 is enabled, PA.05 is 
	 automatically connected to the DAC converter. */
	DAC_Cmd(DAC1, DAC_Channel_1, ENABLE);

	/* Enable DMA for DAC Channel1 */
	DAC_DMACmd(DAC1, DAC_Channel_1, ENABLE);
/******/	

}
