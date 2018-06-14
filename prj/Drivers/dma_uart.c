#include "dma_uart.h"
#include "string.h"
#include "stdio.h"
//#include "led.h"
//#include "delay.h"


/**
  * @brief  �ض��� printf & scanf �ӿ�
  * @param  
  * @retval ����״̬
  */
struct __FILE
{
	int handle;
};
FILE __stdout;

int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ� */
		USART_SendData(UARTx, (uint8_t) ch);
		
		/* �����ȴ�������� */
		while (USART_GetFlagStatus(UARTx, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}
int fgetc(FILE *f)
{
		/* �����ȴ�һ���ֽ����� */
		while (USART_GetFlagStatus(UARTx, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(UARTx);
}

//С�˴洢�����ֽ������ڵ͵�ַ
#define BYTE0(dwTemp)       (*( char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


uint8_t uart_tx_buf[UART_BUF_MAX];							//����DMA����
uint8_t uart_rx_buf[UART_BUF_MAX];									
uint8_t uart_tx_buf2[UART_BUF2_MAX];						//���ڳ����жϷ�ʽ����
//static uint16_t uart_tx_cnt = 0;
static uint8_t uart_tx2_cnt = 0;
//static uint16_t uart_buf_cnt = 0;
static uint8_t uart_buf2_cnt = 0;
uint8_t uartStatus = 0x00;									//���մ����־
uint8_t uartRxLen = 0;										//���ݽ��ճ���

pFun_uartCallBack dma_uartCallback_rx = NULL;				//���ջص�����ָ��


/**
  * @brief  �����ã����յ������ݷ���ȥ
  * @param  rcvn ���յ������ݰ�����
  * @retval none
  */
void dma_uart_sendBack_Looptest(uint16_t rcvn)
{ 
	memcpy(uart_tx_buf,uart_rx_buf, rcvn);
	dma_uart_start_tx(rcvn);
}


/**
  * @brief  DMA��ʼ��
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
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UARTx->TDR);							//������������ݼĴ���(TDR/RDR)����DR��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart_tx_buf;								//�������ݴ洢������ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;											//���䷽��
	DMA_InitStructure.DMA_BufferSize = UART_BUF_MAX;											//�洢������
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;							//�����ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;										//�洢����ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;						//��������λ��
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;								//�洢������λ��
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;												//���λ�ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;										//���ȼ�
	DMA_Init(DMAx_TX_Channel,&DMA_InitStructure);												//��ʼ��DMA Stream
	DMA_ClearFlag(DMAx_TX_Flag_ALLERR | DMAx_TX_Flag_TC);										//������б�־λ
	DMA_ITConfig(DMAx_TX_Channel, DMA_IT_TC, ENABLE);											//ʹ�ܴ��ڷ���ͨ��DMA��������жϣ��Ա��ڷ������֮��رմ��ڷ���Stream
	
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
  * @brief  ���ڼ�DMA�жϳ�ʼ��
  * @param  
  * @retval 
  */
void nvic_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = DMAx_TX_IRQ;											//DMA�ķ���Stream�жϣ����ڷ�������жϵĴ���
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = UARTx_IRQ;												//�����жϣ����ڴ����ڿ����жϡ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  �����ܳ�ʼ��
  * @param  
  * @retval ����״̬
  */
void dma_uart_init(uint32_t baud)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/***  ����GPIO��USARTʱ�� ***/
	UARTx_RCC_CLK_FUNC(UARTx_RCC_APB, ENABLE);
	UARTx_RCC_GPIO_FUNC(UARTx_GPIO_APB,ENABLE);

	GPIO_PinAFConfig(UARTx_GPIO, UARTx_TX_PIN_SRC, UARTx_PIN_AF);			//����UART���ŵ�GPIO
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
	USART_ITConfig(UARTx, USART_IT_IDLE, ENABLE);											//ʹ�ܴ��ڿ����жϣ������ڿ��г���һ���ַ�ʱ�䣬�����������ж�
	USART_Cmd(UARTx, ENABLE);
	/*** Enable the uart dma request line ***/
	USART_DMACmd(UARTx, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(UARTx, USART_DMAReq_Rx, ENABLE);

	dma_init();
	nvic_init();
#ifdef DMA_UART_TEST
	dma_uartCallback_rx = dma_uart_sendBack_Looptest;									//���ڲ��Դ���DMA�շ�
#endif
}



/**
  * @brief  �����жϷ�ʽ����һ���ֽ�
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
  * @brief  ��ǰ�������ݵ�DMA���ͻ���
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
  * @brief  ����һ��DMA���ͣ����η���
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
  * @brief  DMA��������жϣ����η��ͣ����֮��ر�DMA���´η���ʱ������
  * @param  
  * @retval 
  */
void DMAx_TX_IRQHandler(void)
{
	if(DMA_GetITStatus(DMAx_TX_Flag_TC) != RESET)
	{
		/*** tx finish ***/
		DMA_ClearITPendingBit(DMAx_TX_IT_TC);						//���TC�жϣ���Ϊʹ����TC�ж�,����жϱ�־λ�������¼���־λһ�����
		DMA_ClearFlag(DMAx_TX_Flag_ALLERR);							//������������־λ������û��ʹ����Щ�жϣ�����ֻ��Ҫ�����־λ
		DMA_Cmd(DMAx_TX_Channel, DISABLE);							//�رշ���Stream���ȴ��´η���ʱʹ��
	}	
}

/**
  * @brief  �����жϣ�������ݽ��պ��жϷ�ʽ��������
  * @param  
  * @retval none
  */
void UARTx_IRQHandler(void)
{
	 __IO uint16_t temp = 0;
	
    /** ���ͻ�����ж� **/
    if(USART_GetITStatus(UARTx, USART_IT_TXE) && (UARTx->CR1 & USART_CR1_TXEIE))				//�жϷ�ʽ�������ݵ�ʵ�֣����ͻ���Ϊ��ʱ�����жϷ�������
    {
        UARTx->TDR = uart_tx_buf2[uart_tx2_cnt++];												//continue to send data byte
        if(uart_tx2_cnt == uart_buf2_cnt)														//data send finish, close TXE interrput.
        {
            USART_ITConfig(UARTx, USART_IT_TXE, DISABLE);
        }
    }

	/*** ���ڿ����ж� ***/
	if(USART_GetITStatus(UARTx, USART_IT_IDLE ) != RESET)
	{
		/*** Read SR & DR register to clear flag ***/
		USART_ClearITPendingBit(UARTx, USART_IT_IDLE);
//		temp = USART1->ISR;  																	//��ͬ����ͨ�жϱ�־�����ڿ����ж���Ҫ�ض������������У����оƬ�ο��ֲ�
//		temp = USART1->RDR;
		
		DMA_Cmd(DMAx_RX_Channel, DISABLE);														//��DMA���޸����ã������ڹر�״̬�½��У�������Ӧ�Ĵ����޸���Ч
		DMA_ClearITPendingBit(DMAx_RX_IT_TC);													//һ��DMA�������֮����Ҫ�����TC��־λ�����򲻻���Ӧ�ڶ��δ��ڽ���ͨ����DMA����F1�ϲ��ã�
		
		/*** Get the Recieved lentgh ***/
		temp = UART_BUF_MAX - DMA_GetCurrDataCounter(DMAx_RX_Channel);							//��ȡ�������ݰ�����
		if(temp > 0)
		{
			uartRxLen = temp;																	//ʹ��temp�������ݴ������ݣ������䱻�������Ż���
			uartStatus = UART_OK;																//��λ���ձ�־λ
			if(dma_uartCallback_rx != NULL)															//�ص���������һ֡��������
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
  * @brief  ��ȡUART��ǰ����״̬
  * @param  
  * @retval ����״̬
  */
uint8_t uart_getRxStatus(void)
{
	return uartStatus;
}

/**
* @brief	��ȡUART��ǰ���ճ���
* @param	
* @retval ���յ������ݰ�����
*/
uint8_t uart_getRxLen(void)
{
	if((uartStatus & UART_OK) == UART_OK)
		return uartRxLen;																		//�����ǰ���յ���һ֡�����������Ĳŷ�����Ч����
	else
		return 0;
}

/**
  * @brief  ��ȡUART��ǰ��������
  * @param  
  * @retval �������ݰ�����
  */
uint8_t uart_getData(uint8_t * rxBuf, uint8_t len)
{
	uint8_t i;
	for(i = 0;i < len;i++)
	{
		if(i >= uartRxLen)
		{
			uartStatus = 0;																		//��ʹû���������յ�ǰ���ݰ���Ҳ������ձ�־λ��������Ϊ���������ݵ���
			return uartRxLen;																	//�����ظ�ʹ�þ����ݣ������߼�ʵ������Ҫ�򵥵�
		}
		rxBuf[i] = uart_rx_buf[i];
	}
	uartStatus = 0;
	return len;
}



