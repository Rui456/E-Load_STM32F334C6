#include "adc_key.h"
#include "delay.h"
#include "stdio.h"

/*******************
*
*	GPIO		PA0
*	UP			0.5
*	DOWN		0.825
*	LEFT		0.75
*	RIGHT		0.872
*	CENTER		0.667
*
*******************/

#define AD_KEY_CH	0
#define AD_KEY_OFFSET	0x10
#define AD_OFFSET	4096

#define AD_UP			(0x0F39)
#define AD_DOWN			(0x0EFC)
#define AD_LEFT			(0x0D55)
#define AD_RIGHT		(0x0F5E)
#define AD_CENTER		(0x0E8A)

#define KEY_CHK(x,y)	(((x) - (y) < AD_KEY_OFFSET) && ((y) - (x) < AD_KEY_OFFSET))

#define ADC2_DR_Address    ((u32)ADC2_BASE+0x40)

u16 adc_RawValue = 0;
uint32_t calibration_value = 0;
uint8_t key_update;
//adc_isr_t adc_awd_isr;				//


/*** check the key value for once ***/
u8 key_read(void)
{
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == RESET)
		return KEY_EMG;
	else if(KEY_CHK(adc_RawValue,AD_UP))
		return KEY_UP;
	else if(KEY_CHK(adc_RawValue,AD_DOWN))
		return KEY_DOWN;
	else if(KEY_CHK(adc_RawValue,AD_LEFT))
		return KEY_LEFT;
	else if(KEY_CHK(adc_RawValue,AD_RIGHT))
		return KEY_RIGHT;
	else if(KEY_CHK(adc_RawValue,AD_CENTER))
		return KEY_CENTER;
	else
		return 0xFF;
}

/*** ADC1 ISR ***/
void ADC1_2_IRQHandler()
{
	if(ADC_GetITStatus(ADCx, ADC_IT_AWD1)){				//anolog watch dog interrupt
		key_update = 1;
//		printf("adc value update:0x%x.\r\n",adc_RawValue);
//		if(adc_awd_isr){
//			adc_awd_isr(ADC_ISR);
//		}
		ADC_ClearITPendingBit(ADCx, ADC_IT_AWD1);
	}
}


/*** initial adc for all ***/
void adc_init(void)
{
	adc_gpio_init();
	adc_dma_init();
	adc_watchdog_init();
}


/*** initial adc gpio ***/
void adc_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

    /** enable gpio clock  **/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB,ENABLE);

    /* init gpio */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*** adc analog watchdog function initial ***/
void adc_watchdog_init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
	
	ADC_AnalogWatchdog1SingleChannelConfig(ADCx,ADCx_Channelx);				//set single channel number
	ADC_AnalogWatchdog1ThresholdsConfig(ADCx, AD_OFFSET-1, 0.98*AD_OFFSET);		//set thresholds
	
    /*** 设置中断优先级 ***/
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	ADC_ITConfig(ADCx,ADC_IT_AWD1, ENABLE);

	ADC_AnalogWatchdogCmd(ADCx,ADC_AnalogWatchdog_SingleRegEnable);			//enable single regular mode
}

/*** initial adc dma,auto transfer ***/
void adc_dma_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

    /** 使能ADC & GPIO 时钟  **/
    ADCx_RCC_FUNC(ADCx_RCC_Periph, ENABLE);
    /** 使能DMA时钟 **/
    ADCx_DMAx_RCC_FUNC(ADCx_DMAx_RCC_Periph, ENABLE);

	/* DMA channel1 configuration */
	DMA_DeInit(ADCx_DMAx_Channelx);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC2_DR_Address;	                //ADC地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&adc_RawValue;                  //数组内存首地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;                                       //6个数据
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //外设地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //内存地址固定
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		                        //循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(ADCx_DMAx_Channelx, &DMA_InitStructure);

	/* Enable DMA channel1 */
	DMA_Cmd(ADCx_DMAx_Channelx, ENABLE);

	/* Configures the ADC DMA */
	ADC_DMAConfig(ADCx, ADC_DMAMode_Circular);
	
	/* 使能 ADC1 DMA */
	ADC_DMACmd(ADCx, ENABLE);

	/* ADC1 configuration */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;					//独立ADC模式                                                                   
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;                  
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
	ADC_CommonInit(ADCx, &ADC_CommonInitStructure);
	
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;	//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	                    //采集数据右对齐
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;	//不使用外部触发转换
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_Init(ADCx, &ADC_InitStructure);

	/*配置ADC时钟，为PCLK2的6分频，即12MHz,ADC频率最高不能超过14MHz*/
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div6);

	/*配置ADC1的通道1为7.5个采样周期，将待转换通道按顺序分配序号 */
	ADC_RegularChannelConfig(ADCx, ADCx_Channelx, 1, ADC_SampleTime_7Cycles5);

	/* Calibration procedure */ 
	ADC_VoltageRegulatorCmd(ADCx, ENABLE);
	delay_ms(10);
	ADC_SelectCalibrationMode(ADCx, ADC_CalibrationMode_Single);
	/* ADC校准 */
	ADC_StartCalibration(ADCx);
	
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADCx) != RESET )
		;
	calibration_value = ADC_GetCalibrationValue(ADCx);

	/* 使能 ADC1 */
	ADC_Cmd(ADCx, ENABLE);						//需要在校准完成之后再使能ADC，否则会校准失败阻塞

	/* wait for ADRDY */
	while(!ADC_GetFlagStatus(ADCx, ADC_FLAG_RDY));
	
	/* 启动ADC转换 */
	ADC_StartConversion(ADCx);
	

}

