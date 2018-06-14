#ifndef __ADC_KEY_H_
#define __ADC_KEY_H_
#include "stm32f30x.h"

#define ADCx					ADC2
#define ADCx_RCC_FUNC			RCC_AHBPeriphClockCmd
#define ADCx_RCC_Periph			RCC_AHBPeriph_ADC12
//#define ADCx_GPIO
//#define ADCx_PIN
#define ADCx_Channelx			ADC_Channel_4

#define ADCx_DMAx_Channelx		DMA1_Channel2
#define ADCx_DMAx_RCC_FUNC		RCC_AHBPeriphClockCmd
#define ADCx_DMAx_RCC_Periph	RCC_AHBPeriph_DMA1


enum {
	KEY_UP = 0xF1,
	KEY_DOWN,
	KEY_LEFT,
	KEY_RIGHT,
	KEY_CENTER,
	KEY_EMG,
	KEY_NONE = 0xFF,
	ADC_ISR = 0xAD
};

typedef void (*adc_isr_t)(uint8_t cmd);

extern adc_isr_t adc_awd_isr;
extern u16 adc_RawValue;
extern uint32_t calibration_value;
extern uint8_t key_update;


void adc_init(void);
void adc_gpio_init(void);
void adc_dma_init(void);
void adc_watchdog_init(void);
u8 key_read(void);




#endif
