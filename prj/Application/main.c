/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-August-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma_uart.h"
#include "oled.h"
#include "delay.h"
#include "adc_key.h"
#include "stdio.h"

/** @addtogroup STM32F30x_StdPeriph_Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured, 
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32f30x.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32f30x.c file
	 */ 

	/* Add your application code here
	 */
	delay_init(72);
	dma_uart_init(115200);
	OLED_Init();
	adc_init();
	/* Infinite loop */
	OLED_CP_Demo();
	printf("Hello World.\r\n");
	delay_ms(300);
	OLED_Clear();
	OLED_Fill(0,0,127,0,1);
	OLED_Fill(0,0,0,63,1);
	OLED_Fill(16,0,16,63,1);
	OLED_ShowNum(5,1,01,16,2,1);
	OLED_ShowString(24,1,"Uart OK.",16,1);
	OLED_ShowHex(24,18,calibration_value,16);
//	OLED_ShowString(0,1,"123456789",16,1);
	OLED_Fill(0,17,127,17,1);
//	OLED_ShowString(0,18,"123456789",16,1);
	OLED_Fill(0,34,127,34,1);
//	OLED_ShowString(0,35,"123456789",16,1);
	OLED_Fill(0,51,127,51,1);
//	OLED_ShowString(0,52,"123456789",16,1);
//	OLED_ShowString(72,0,"123456789",7,1);
	while (1)
	{
		if(key_update){
			OLED_ShowHex(24,35,adc_RawValue,16);
			printf("adc value update:0x%x.\r\n",adc_RawValue);
			switch(key_read())
			{
				case KEY_EMG:OLED_ShowString(100,1,"EMG",16,1);break;
				case KEY_UP:OLED_ShowChar(90,1,'U',16,1);break;
				case KEY_DOWN:OLED_ShowChar(90,35,'D',16,1);break;
				case KEY_LEFT:OLED_ShowChar(75,18,'L',16,1);break;
				case KEY_RIGHT:OLED_ShowChar(105,18,'R',16,1);break;
				case KEY_CENTER:OLED_ShowChar(90,18,'C',16,1);break;
				default:
						OLED_ShowString(100,1,"   ",16,1);
						OLED_ShowChar(90,1,' ',16,1);
						OLED_ShowChar(90,35,' ',16,1);
						OLED_ShowChar(75,18,' ',16,1);
						OLED_ShowChar(105,18,' ',16,1);
						OLED_ShowChar(90,18,' ',16,1);
			}
			key_update = 0;
			delay_ms(50);
		}
//		delay_ms(100);
	}
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
