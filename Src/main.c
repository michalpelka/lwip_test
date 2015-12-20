/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include "udp.h"
#include "3dunit/mavlink.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


extern uint8_t bSendUDP;
extern uint8_t bLEDBreathing;
extern uint64_t iMilisecs;
uint8_t mavlink_systemID = 1;
uint16_t mavlink_port = 14550;
uint8_t mavlink_getParams = 0x00;
struct ip_addr destAddr;
struct udp_pcb* my_udp_pcb;
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port);



void send_udp_simple(uint8_t *payload, uint16_t length)
{
	struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
    memcpy(p->payload, payload, length);
    udp_sendto(my_udp_pcb, p, &destAddr, mavlink_port);
	pbuf_free(p);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LWIP_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int i =0;
  int j =0;
  IP4_ADDR(&destAddr, 192, 168, 0, 100);
  my_udp_pcb =  udp_new();

  udp_bind(my_udp_pcb, IP_ADDR_ANY, mavlink_port);
  udp_recv(my_udp_pcb, udp_receive_callback, NULL);
  while (1)
  {


  /* USER CODE END WHILE */
	  MX_LWIP_Process();
	  if (bSendUDP == 0xff)
	  {
		  bSendUDP = 0x00;

		  // produce mavlink heartbeat
		  mavlink_heartbeat_t heart_beat;
		  heart_beat.type = MAV_TYPE_GENERIC;
		  heart_beat.autopilot = MAV_AUTOPILOT_GENERIC;
		  heart_beat.base_mode = MAV_MODE_FLAG_TEST_ENABLED;
		  heart_beat.system_status = MAV_STATE_ACTIVE;
		  heart_beat.mavlink_version = 3;
		  mavlink_message_t message;
		  mavlink_msg_heartbeat_encode(mavlink_systemID,0,&message, &heart_beat);
		  uint8_t buffer[200];
		  uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
		  send_udp_simple(buffer, len);
		  //struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
		  //memcpy(p->payload, buffer, len);
		  //udp_sendto(my_udp_pcb, p, &destAddr, mavlink_port);
		  //pbuf_free(p);

		  mavlink_named_value_int_t v1;
		  char name[10]="encoder";
		  memcpy((char*)v1.name, name, 10);
		  v1.time_boot_ms = iMilisecs;
		  v1.value =100;
		  mavlink_msg_named_value_int_encode(mavlink_systemID, 0 ,&message, &v1  );
		  len = mavlink_msg_to_send_buffer(buffer, &message);
		  send_udp_simple(buffer, len);

		  if (mavlink_getParams == 0xff)
		  {
			  mavlink_getParams = 0x00;
			  // give client parameters list
			  {

				  uint16_t IP_1=192;
				  mavlink_param_value_t param;
				  param.param_count = 4;
				  strncpy(param.param_id, "DST_IP_1", 16);
				  param.param_index =0;
				  param.param_type = MAV_PARAM_TYPE_INT16;
				  mavlink_param_union_t t;
				  t.param_int16 = IP_1;
				  param.param_value = t.param_float;
				  mavlink_message_t message;
				  mavlink_msg_param_value_encode(mavlink_systemID,0, &message, &param);
				  len = mavlink_msg_to_send_buffer(buffer, &message);
				  send_udp_simple(buffer, len);
			  }
			  {
					  uint16_t IP_2 =168;
					  mavlink_param_value_t param;
					  param.param_count = 1;
					  strncpy(param.param_id, "DST_IP_2", 16);
					  param.param_index =1;
					  param.param_type = MAV_PARAM_TYPE_INT16;
					  mavlink_param_union_t t;
					  t.param_int16 = IP_2;
					  param.param_value = t.param_float;
					  mavlink_message_t message;
					  mavlink_msg_param_value_encode(mavlink_systemID,0, &message, &param);
					  len = mavlink_msg_to_send_buffer(buffer, &message);
					  send_udp_simple(buffer, len);
			  }
			  {
					  uint16_t IP_3 =0;
					  mavlink_param_value_t param;
					  param.param_count = 4;
					  strncpy(param.param_id, "DST_IP_3", 16);
					  param.param_index =2;
					  param.param_type = MAV_PARAM_TYPE_INT16;
					  mavlink_param_union_t t;
					  t.param_int16 = IP_3;
					  param.param_value = t.param_float;
					  mavlink_message_t message;
					  mavlink_msg_param_value_encode(mavlink_systemID,0, &message, &param);
					  len = mavlink_msg_to_send_buffer(buffer, &message);
					  send_udp_simple(buffer, len);

				}

			  	{
					  uint16_t IP_4 =0;
					  mavlink_param_value_t param;
					  param.param_count = 4;
					  strncpy(param.param_id, "DST_IP_4", 16);
					  param.param_index =3;
					  param.param_type = MAV_PARAM_TYPE_INT16;
					  mavlink_param_union_t t;
					  t.param_int16 = IP_4;
					  param.param_value = t.param_float;
					  mavlink_message_t message;
					  mavlink_msg_param_value_encode(mavlink_systemID,0, &message, &param);
					  len = mavlink_msg_to_send_buffer(buffer, &message);
					  send_udp_simple(buffer, len);

			  	}

		  }

	  }
	  if (bLEDBreathing == 0xff)
	  {
		  bLEDBreathing = 0x00;

	  }

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA6   ------> ADCx_IN6
     PC4   ------> ADCx_IN14
     PC5   ------> ADCx_IN15
     PB15   ------> I2S2_SD
     PC6   ------> I2S2_MCK
     PA8   ------> RCC_MCO
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PC10   ------> SPI3_SCK
     PC11   ------> SPI3_MISO
     PC12   ------> SPI3_MOSI
     PD0   ------> CAN1_RX
     PD1   ------> CAN1_TX
     PD5   ------> USART2_TX
     PD6   ------> USART2_RX
     PB6   ------> I2C1_SCL
     PB7   ------> I2C1_SDA
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : MicroSDCard_CS_Pin */
  GPIO_InitStruct.Pin = MicroSDCard_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(MicroSDCard_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IDD_Measurement_Pin */
  GPIO_InitStruct.Pin = IDD_Measurement_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(IDD_Measurement_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Potentiometer_Pin VBAT_Voltage_OUT_Pin */
  GPIO_InitStruct.Pin = Potentiometer_Pin|VBAT_Voltage_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IO_Expander_INT_Pin */
  GPIO_InitStruct.Pin = IO_Expander_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IO_Expander_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S_DIN_Pin */
  GPIO_InitStruct.Pin = I2S_DIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(I2S_DIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED3_Pin LED4_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED3_Pin|LED4_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S_MCK_Pin PC10 PC12 */
  GPIO_InitStruct.Pin = I2S_MCK_Pin|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCO_Pin */
  GPIO_InitStruct.Pin = MCO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(MCO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_VBUS_Pin USB_ID_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin|USB_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCK_Pin PB7 */
  GPIO_InitStruct.Pin = I2C1_SCK_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MicroSDCard_Detection_Pin */
  GPIO_InitStruct.Pin = MicroSDCard_Detection_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MicroSDCard_Detection_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_SPI3_ENABLE();

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_CAN1_3();

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_USART2_ENABLE();

}

/* USER CODE BEGIN 4 */
mavlink_message_t message;
mavlink_status_t status;
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{

	struct pbuf *my_p = p;
	do
	{
		char* data = (char*)my_p->payload;
		int i;
		for ( i =0; i < my_p->len; i++)
		{
			if (mavlink_parse_char(1, data[i], &message, &status))
			{
				uint8_t sysid = message.sysid;
				uint8_t compid = message.compid;
				uint8_t system_id = message.sysid;
				uint8_t autopilot_id = message.compid;
				if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
				{
					HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
				}
				else
				if (message.msgid == 21)
				{
					mavlink_getParams = 0xff;
				}
			}
		}
		my_p = my_p->next;
	}
	while (my_p != NULL);

	/* Free the p buffer */
	pbuf_free(p);

}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
