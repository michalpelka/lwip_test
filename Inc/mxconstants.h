/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
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

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define TRACE_CK_Pin GPIO_PIN_2
#define TRACE_CK_GPIO_Port GPIOE
#define TRACE_D0_Pin GPIO_PIN_3
#define TRACE_D0_GPIO_Port GPIOE
#define TRACE_D1_Pin GPIO_PIN_4
#define TRACE_D1_GPIO_Port GPIOE
#define TRACE_D2_Pin GPIO_PIN_5
#define TRACE_D2_GPIO_Port GPIOE
#define TRACE_D3_Pin GPIO_PIN_6
#define TRACE_D3_GPIO_Port GPIOE
#define Anti_Tamper_Pin GPIO_PIN_13
#define Anti_Tamper_GPIO_Port GPIOC
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define MII_MDC_Pin GPIO_PIN_1
#define MII_MDC_GPIO_Port GPIOC
#define MII_TXD2_Pin GPIO_PIN_2
#define MII_TXD2_GPIO_Port GPIOC
#define MII_TX_CLK_Pin GPIO_PIN_3
#define MII_TX_CLK_GPIO_Port GPIOC
#define MII_CRS_Pin GPIO_PIN_0
#define MII_CRS_GPIO_Port GPIOA
#define MII_MDIO_Pin GPIO_PIN_2
#define MII_MDIO_GPIO_Port GPIOA
#define MII_COL_Pin GPIO_PIN_3
#define MII_COL_GPIO_Port GPIOA
#define MicroSDCard_CS_Pin GPIO_PIN_4
#define MicroSDCard_CS_GPIO_Port GPIOA
#define IDD_Measurement_Pin GPIO_PIN_6
#define IDD_Measurement_GPIO_Port GPIOA
#define Potentiometer_Pin GPIO_PIN_4
#define Potentiometer_GPIO_Port GPIOC
#define VBAT_Voltage_OUT_Pin GPIO_PIN_5
#define VBAT_Voltage_OUT_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_2
#define LCD_CS_GPIO_Port GPIOB
#define MII_RX_ER_Pin GPIO_PIN_10
#define MII_RX_ER_GPIO_Port GPIOB
#define MII_TX_EN_Pin GPIO_PIN_11
#define MII_TX_EN_GPIO_Port GPIOB
#define MII_TXD0_I2S_CMD_Pin GPIO_PIN_12
#define MII_TXD0_I2S_CMD_GPIO_Port GPIOB
#define MII_TXD1_I2S_CK_Pin GPIO_PIN_13
#define MII_TXD1_I2S_CK_GPIO_Port GPIOB
#define IO_Expander_INT_Pin GPIO_PIN_14
#define IO_Expander_INT_GPIO_Port GPIOB
#define I2S_DIN_Pin GPIO_PIN_15
#define I2S_DIN_GPIO_Port GPIOB
#define MII_RXD0_Pin GPIO_PIN_9
#define MII_RXD0_GPIO_Port GPIOD
#define MII_RXD1_Pin GPIO_PIN_10
#define MII_RXD1_GPIO_Port GPIOD
#define MII_RXD2_Pin GPIO_PIN_11
#define MII_RXD2_GPIO_Port GPIOD
#define MII_RXD3_Pin GPIO_PIN_12
#define MII_RXD3_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOD
#define I2S_MCK_Pin GPIO_PIN_6
#define I2S_MCK_GPIO_Port GPIOC
#define USB_PowerSwitchOn_Pin GPIO_PIN_9
#define USB_PowerSwitchOn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_8
#define MCO_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_SWDIO_Pin GPIO_PIN_13
#define TMS_SWDIO_GPIO_Port GPIOA
#define TCK_SWCLK_Pin GPIO_PIN_14
#define TCK_SWCLK_GPIO_Port GPIOA
#define TDI_Pin GPIO_PIN_15
#define TDI_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOD
#define TDO_Pin GPIO_PIN_3
#define TDO_GPIO_Port GPIOB
#define I2C1_SCK_Pin GPIO_PIN_6
#define I2C1_SCK_GPIO_Port GPIOB
#define MII_TXD3_Pin GPIO_PIN_8
#define MII_TXD3_GPIO_Port GPIOB
#define User_Button_Pin GPIO_PIN_9
#define User_Button_GPIO_Port GPIOB
#define MicroSDCard_Detection_Pin GPIO_PIN_0
#define MicroSDCard_Detection_GPIO_Port GPIOE
#define USB_OverCurrent_Pin GPIO_PIN_1
#define USB_OverCurrent_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
