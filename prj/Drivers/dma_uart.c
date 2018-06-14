#include "dma_uart.h"
#include "string.h"
#include "stdio.h"
//#include "led.h"
//#include "delay.h"


/**
  * @brief  重定向 printf & scanf 接口
  * @param  
  * @retval 接收状态
  */
struct __FILE
{
	int handle;
};
FILE __stdout;

int fputc(int ch, FILE *f)
{
		/* 发送一个字节 */
		USART_SendData(UARTx, (uint8_t) ch);
		
		/* 阻塞等待发送完成 */
		while (USART_GetFlagStatus(UARTx, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}
int fgetc(FILE *f)
{
		/* 阻塞等待一个字节输入 */
		while (USART_GetFlagStatus(UARTx, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(UARTx);
}

//小端存储，低字节数据在低地址
#define BYTE0(dwTemp)       (*( char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


uint8_t uart_tx_buf[UART_BUF_MAX];							//用于DMA发送
uint8_t uart_rx_buf[UART_BUF_MAX];									
uint8_t uart_tx_buf2[UART_BUF2_MAX];						//用于常规中断方式发送
//static uint16_t uart_tx_cnt = 0;
static uint8_t uart_tx2_cnt = 0;
//static uint16_t uart_buf_cnt = 0;
static uint8_t uart_buf2_cnt = 0;
uint8_t uartStatus = 0x00;									//接收错误标志
uint8_t uartRxLen = 0;										//数据接收长度

pFun_uartCallBack dma_uartCallback_rx = NULL;				//接收回调函数指针


/**
  * @brief  测试用，将收到的数据发出去
  * @param  rcvn 接收到的数据包长度
  * @retval none
  */
void dma_uart_sendBack_Looptest(uint16_t rcvn)
{ 
	memcpy(uart_tx_buf,uart_rx_buf, rcvn);
	dma_uart_start_tx(rcvn);
}


/**
  * @brief  DMA初始化
  * @param  
  * @retval 
  */
void dma_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHBPeriphClockCmd(DMAx_RCC_AHB, ENABLE);
	
	/*** Config the uart tx dma channel  ***/
	DMA_Cmd(DMAx_TX_Channel, DISABLE);
	DMA_DeInit(DMAx_TX_Channel);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UARTx->TDR);							//发送与接收数据寄存器(TDR/RDR)共用DR地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart_tx_buf;								//发送数据存储器基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;											//传输方向
	DMA_InitStructure.DMA_BufferSize = UART_BUF_MAX;											//存储器长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;							//外设地址自增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;										//存储器地址自增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;						//外设数据位宽
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;								//存储器数据位宽
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;												//单次或循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;										//优先级
	DMA_Init(DMAx_TX_Channel,&DMA_InitStructure);												//初始化DMA Stream
	DMA_ClearFlag(DMAx_TX_Flag_ALLERR | DMAx_TX_Flag_TC);										//清除现有标志位
	DMA_ITConfig(DMAx_TX_Channel, DMA_IT_TC, ENABLE);											//使能串口发送通道DMA发送完成中断，以便在发送完成之后关闭串口发送Stream
	
	/*** Config the uart rx dma channel  ***/
	DMA_Cmd(DMAx_RX_Channel, DISABLE);
	DMA_DeInit(DMAx_RX_Channel);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UARTx->RDR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart_rx_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(DMAx_RX_Channel,&DMA_InitStructure);
	DMA_ClearFlag(DMAx_RX_Flag_ALLERR | DMAx_RX_Flag_TC);
	DMA_Cmd(DMAx_RX_Channel, ENABLE);
}

/**
  * @brief  串口及DMA中断初始化
  * @param  
  * @retval 
  */
void nvic_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = DMAx_TX_IRQ;											//DMA的发送Stream中断，用于发送完成中断的处理
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = UARTx_IRQ;												//串口中断，用于处理串口空闲中断。
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  串口总初始化
  * @param  
  * @retval 接收状态
  */
void dma_uart_init(uint32_t baud)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/***  开启GPIO和USART时钟 ***/
	UARTx_RCC_CLK_FUNC(UARTx_RCC_APB, ENABLE);
	UARTx_RCC_GPIO_FUNC(UARTx_GPIO_APB,ENABLE);

	GPIO_PinAFConfig(UARTx_GPIO, UARTx_TX_PIN_SRC, UARTx_PIN_AF);			//链接UART引脚到GPIO
	GPIO_PinAFConfig(UARTx_GPIO, UARTx_RX_PIN_SRC, UARTx_PIN_AF);

	GPIO_InitStructure.GPIO_Pin =  UARTx_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UARTx_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  UARTx_RX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UARTx_GPIO, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(UARTx, &USART_InitStructure);
	
	/*** Enable uart idle interrput, as a indication of data frame finish ***/
	USART_ITConfig(UARTx, USART_IT_IDLE, ENABLE);											//使能串口空闲中断，当串口空闲超过一个字符时间，即产生空闲中断
	USART_Cmd(UARTx, ENABLE);
	/*** Enable the uart dma request line ***/
	USART_DMACmd(UARTx, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(UARTx, USART_DMAReq_Rx, ENABLE);

	dma_init();
	nvic_init();
#ifdef DMA_UART_TEST
	dma_uartCallback_rx = dma_uart_sendBack_Looptest;									//用于测试串口DMA收发
#endif
}



/**
  * @brief  常规中断方式发送一个字节
  * @param  
  * @retval 
  */
void uart_send_byte(uint8_t chr)
{
	uart_tx_buf2[uart_buf2_cnt++] = chr;
	
    if(!(UARTx->CR1 & USART_CR1_TXEIE))
    	USART_ITConfig(UARTx, USART_IT_TXE, ENABLE);
}

void uart_send_string(char* s)
{
	while(*s)
	{
		uart_send_byte(*s++);
	}
}

/**
  * @brief  提前加载数据到DMA发送缓存
  * @param  
  * @retval none
  */
void dma_uart_load_data(uint8_t *ptr, uint16_t len)
{
	if(ptr)
	{
		memcpy(uart_tx_buf,ptr,len);
	}
}

/**
  * @brief  启动一次DMA发送，单次发送
  * @param  
  * @retval none
  */
void dma_uart_start_tx(uint16_t cnt)
{
	/*** set the tx data length, data buf shoule be already avalible. ***/
	DMA_SetCurrDataCounter(DMAx_TX_Channel, (uint16_t)cnt);
	/*** start tx dma channel ***/
	DMA_Cmd(DMAx_TX_Channel, ENABLE);
}

/**
  * @brief  DMA发送完成中断，单次发送，完毕之后关闭DMA，下次发送时再启动
  * @param  
  * @retval 
  */
void DMAx_TX_IRQHandler(void)
{
	if(DMA_GetITStatus(DMAx_TX_Flag_TC) != RESET)
	{
		/*** tx finish ***/
		DMA_ClearITPendingBit(DMAx_TX_IT_TC);						//清除TC中断，因为使能了TC中断,清除中断标志位会连带事件标志位一起清除
		DMA_ClearFlag(DMAx_TX_Flag_ALLERR);							//清除其他错误标志位，这里没有使能这些中断，所以只需要清除标志位
		DMA_Cmd(DMAx_TX_Channel, DISABLE);							//关闭发送Stream，等待下次发送时使能
	}	
}

/**
  * @brief  串口中断，完成数据接收和中断方式发送数据
  * @param  
  * @retval none
  */
void UARTx_IRQHandler(void)
{
	 __IO uint16_t temp = 0;
	
    /** 发送缓存空中断 **/
    if(USART_GetITStatus(UARTx, USART_IT_TXE) && (UARTx->CR1 & USART_CR1_TXEIE))				//中断方式发送数据的实现，发送缓存为空时进入中断发送数据
    {
        UARTx->TDR = uart_tx_buf2[uart_tx2_cnt++];												//continue to send data byte
        if(uart_tx2_cnt == uart_buf2_cnt)														//data send finish, close TXE interrput.
        {
            USART_ITConfig(UARTx, USART_IT_TXE, DISABLE);
        }
    }

	/*** 串口空闲中断 ***/
	if(USART_GetITStatus(UARTx, USART_IT_IDLE ) != RESET)
	{
		/*** Read SR & DR register to clear flag ***/
		USART_ClearITPendingBit(UARTx, USART_IT_IDLE);
//		temp = USART1->ISR;  																	//不同于普通中断标志，串口空闲中断需要特定的软件清除序列，详见芯片参考手册
//		temp = USART1->RDR;
		
		DMA_Cmd(DMAx_RX_Channel, DISABLE);														//对DMA的修改配置，必须在关闭状态下进行，否则相应寄存器修改无效
		DMA_ClearITPendingBit(DMAx_RX_IT_TC);													//一次DMA传输完成之后，需要清除其TC标志位，否则不会响应第二次串口接收通道的DMA请求（F1上不用）
		
		/*** Get the Recieved lentgh ***/
		temp = UART_BUF_MAX - DMA_GetCurrDataCounter(DMAx_RX_Channel);							//获取接收数据包长度
		if(temp > 0)
		{
			uartRxLen = temp;																	//使用temp变量来暂存下数据，避免其被编译器优化掉
			uartStatus = UART_OK;																//置位接收标志位
			if(dma_uartCallback_rx != NULL)															//回调函数处理一帧接收数据
			{
				dma_uartCallback_rx(temp);
			}
		}
		
		
		/*** Reset the rx buf length, ***/
		DMA_SetCurrDataCounter(DMAx_RX_Channel, UART_BUF_MAX);
		/*** Restart the rx dma,waite for next data frame ***/
		DMA_Cmd(DMAx_RX_Channel, ENABLE);
	}
}

/**
  * @brief  获取UART当前接收状态
  * @param  
  * @retval 接收状态
  */
uint8_t uart_getRxStatus(void)
{
	return uartStatus;
}

/**
* @brief	获取UART当前接收长度
* @param	
* @retval 接收到的数据包长度
*/
uint8_t uart_getRxLen(void)
{
	if((uartStatus & UART_OK) == UART_OK)
		return uartRxLen;																		//如果当前接收到的一帧数据是正常的才返回有效长度
	else
		return 0;
}

/**
  * @brief  获取UART当前接收数据
  * @param  
  * @retval 拷贝数据包长度
  */
uint8_t uart_getData(uint8_t * rxBuf, uint8_t len)
{
	uint8_t i;
	for(i = 0;i < len;i++)
	{
		if(i >= uartRxLen)
		{
			uartStatus = 0;																		//即使没有完整接收当前数据包，也清除接收标志位，避免认为又有新数据到来
			return uartRxLen;																	//避免重复使用旧数据，这样逻辑实现起来要简单点
		}
		rxBuf[i] = uart_rx_buf[i];
	}
	uartStatus = 0;
	return len;
}



