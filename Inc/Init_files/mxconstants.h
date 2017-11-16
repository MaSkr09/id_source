/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define GPIO_CAP_BTN_PWR_O_Pin GPIO_PIN_13
#define GPIO_CAP_BTN_PWR_O_GPIO_Port GPIOC
#define ADC1_BATT_VOLT_ADC_AI_Pin GPIO_PIN_0
#define ADC1_BATT_VOLT_ADC_AI_GPIO_Port GPIOC
#define GPIO_CAP_BUTTON_I_Pin GPIO_PIN_0
#define GPIO_CAP_BUTTON_I_GPIO_Port GPIOA
#define TIM2_CH2_PWM_RED_LED_O_Pin GPIO_PIN_1
#define TIM2_CH2_PWM_RED_LED_O_GPIO_Port GPIOA
#define TIM2_CH3_PWM_GREEN_LED_O_Pin GPIO_PIN_2
#define TIM2_CH3_PWM_GREEN_LED_O_GPIO_Port GPIOA
#define TIM2_CH4_PWM_BLUE_LED_O_Pin GPIO_PIN_3
#define TIM2_CH4_PWM_BLUE_LED_O_GPIO_Port GPIOA
#define GPIO_MPU_9250_INT_I_Pin GPIO_PIN_4
#define GPIO_MPU_9250_INT_I_GPIO_Port GPIOA
#define GPIO_MPU_9250_FSYNC_O_Pin GPIO_PIN_5
#define GPIO_MPU_9250_FSYNC_O_GPIO_Port GPIOA
#define UART3_SARA_G340_3V0_TDX_O_Pin GPIO_PIN_10
#define UART3_SARA_G340_3V0_TDX_O_GPIO_Port GPIOB
#define UART3_SARA_G340_3V0_RDX_I_Pin GPIO_PIN_11
#define UART3_SARA_G340_3V0_RDX_I_GPIO_Port GPIOB
#define GPIO_VUSB_DETECT_I_Pin GPIO_PIN_12
#define GPIO_VUSB_DETECT_I_GPIO_Port GPIOB
#define GPIO_SARA_G340_3V0_GPIO1_I_Pin GPIO_PIN_15
#define GPIO_SARA_G340_3V0_GPIO1_I_GPIO_Port GPIOB
#define DEBUG_UART_TX_O_Pin GPIO_PIN_6
#define DEBUG_UART_TX_O_GPIO_Port GPIOC
#define DEBUG_UART_RX_I_Pin GPIO_PIN_7
#define DEBUG_UART_RX_I_GPIO_Port GPIOC
#define GPIO_LTC4065_CHRG_I_Pin GPIO_PIN_15
#define GPIO_LTC4065_CHRG_I_GPIO_Port GPIOA
#define GPIO_BATT_VOLT_EN_O_Pin GPIO_PIN_10
#define GPIO_BATT_VOLT_EN_O_GPIO_Port GPIOC
#define GPIO_SARA_G340_PWR_ON_O_Pin GPIO_PIN_3
#define GPIO_SARA_G340_PWR_ON_O_GPIO_Port GPIOB
#define GPIO_SARA_G340_RESET_O_Pin GPIO_PIN_4
#define GPIO_SARA_G340_RESET_O_GPIO_Port GPIOB
#define I2C1_MEMS_SCL_Pin GPIO_PIN_6
#define I2C1_MEMS_SCL_GPIO_Port GPIOB
#define I2C1_MEMS_SDA_Pin GPIO_PIN_7
#define I2C1_MEMS_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
