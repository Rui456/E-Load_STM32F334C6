#ifndef _DMA_UART_H_
#define _DMA_UART_H_
#include "stm32f30x.h"

#define DMA_UART_TEST

#define UARTx					USART1
#define UARTx_GPIO				GPIOA
#define UARTx_RCC_CLK_FUNC		RCC_APB2PeriphClockCmd
#define UARTx_RCC_APB			RCC_APB2Periph_USART1
#define UARTx_IRQ				USART1_IRQn
#define UARTx_IRQHandler		USART1_IRQHandler

#define UARTx_RCC_GPIO_FUNC		RCC_AHBPeriphClockCmd
#define UARTx_GPIO_APB			RCC_AHBPeriph_GPIOA

#define UARTx_TX_PIN			GPIO_Pin_9
#define UARTx_TX_PIN_SRC		GPIO_PinSource9
#define UARTx_RX_PIN			GPIO_Pin_10
#define UARTx_RX_PIN_SRC		GPIO_PinSource10
#define UARTx_PIN_AF			GPIO_AF_7

#define DMAx					DMA1
#define DMAx_RCC_AHB			RCC_AHBPeriph_DMA1
// DMAx[1-2] -> Streamx[0-7] -> channelx[0-7]
#define DMAx_TX_Channel			DMA1_Channel4
#define DMAx_TX_Flag_ALLERR		(DMA1_FLAG_HT4 | DMA1_FLAG_TE4)
#define DMAx_TX_Flag_TC			DMA1_FLAG_TC4
#define DMAx_TX_IT_TC			DMA1_IT_TC4
#define DMAx_TX_IRQ				DMA1_Channel4_IRQn
#define DMAx_TX_IRQHandler		DMA1_Channel4_IRQHandler

#define DMAx_RX_Channel			DMA1_Channel5
#define DMAx_RX_Flag_ALLERR		(DMA1_FLAG_HT5 | DMA1_FLAG_TE5)
#define DMAx_RX_Flag_TC			DMA1_FLAG_TC5
#define DMAx_RX_IT_TC			DMA1_IT_TC5
#define DMAx_RX_IRQ				DMA1_Channel5_IRQn
#define DMAx_RX_IRQHandler 		DMA1_Channel5_IRQHandler

#define UART_BUF_MAX	1024
#define UART_BUF2_MAX	128


extern uint8_t uart_tx_buf[UART_BUF_MAX];
extern uint8_t uart_rx_buf[UART_BUF_MAX];
/*** 正常接收到一帧数据标志 ***/
#define UART_OK	0x01

typedef void(*pFun_uartCallBack)(uint16_t cnt);
extern pFun_uartCallBack dma_uartCallback_rx;

/* 初始化 */
void dma_uart_init(uint32_t baud);
/* 发送相关 */
void dma_uart_load_data(uint8_t *ptr, uint16_t len);
void dma_uart_start_tx(uint16_t cnt);
void uart_send_byte(uint8_t chr);
void uart_send_string(char* s);														//未实现
/* 接收相关 */
uint8_t uart_getRxStatus(void);
uint8_t uart_getRxLen(void);
uint8_t uart_getData(uint8_t * rxBuf, uint8_t len);

#endif
