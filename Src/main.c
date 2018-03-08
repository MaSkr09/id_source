/****************************************************************************
* Copyright (c) 2017, Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in the
*      documentation and/or other materials provided with the distribution.
*    * Neither the name of the copyright holder nor the names of its
*      contributors may be used to endorse or promote products derived from
*      this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************
* File: main.c
* Purpose: DroneID main task
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-01-01 Martin Skriver, Source written
****************************************************************************/
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
//another change

#include <string.h>
#include <stdbool.h>
//#include <__cross_studio_io.h>

#if defined(DEBUG) || defined(DEBUG_BARO)
#include "debug_task.h"
#endif

#include "id_config.h"
#include "global_defines.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "pwr_management_task.h"
#include "cap_btn_reset_task.h"
#include "ublox_data_reader_task.h"
#include "droneID_ctrl_task.h"
#include "ublox_reg.h" 
#include "nmea.h"
#include "battery_voltage_task.h"
#include "indicator_task.h"
#include "barometric_sensor_task.h"
#include "imu_task.h"
#include "pos_est_task.h"
#include "uart_transmit.h"

#include "main.h"


void SystemClock_Config(void);
void Error_Handler(void);

/***************************************************************************/
/* Defines */
/***************************************************************************/
#if defined(DEBUG) || defined(DEBUG_BARO)
#define DEBUG_QUEUE_SIZE 512
#endif

/***************************************************************************/
/* Task handles */
/***************************************************************************/
TaskHandle_t xHandlePwrManagementTask;
TaskHandle_t xHandleCapBtnResetTask;
TaskHandle_t xHandleUbloxDataReaderTask;
TaskHandle_t xHandleUartTransmitTask;
TaskHandle_t xHandleDroneidCtrlTask;
TaskHandle_t xHandleBatteryVoltageTask;
TaskHandle_t xHandleIndicatorTask;
TaskHandle_t xHandleBaroSensTask;
TaskHandle_t xHandleImuTask;
TaskHandle_t xHandlePosEstTask;
TaskHandle_t xHandleDebugTask;

/***************************************************************************/
/* Mutex, queues, variables ... */
/***************************************************************************/

/* Debug queue */
#if defined(DEBUG) || defined(DEBUG_BARO)
QueueHandle_t xQueueDebugQueue;
SemaphoreHandle_t xSemaphoreDebugMutex;
#endif

/* variables to define indication with LEDs */
//cnt_task_indicator_t cnt_task_indicator = NO_INDICATION_CNTR;
pwr_task_indicator_t pwr_task_indicator = NO_INDICATION_PWR;
cap_rst_task_indicator_t cap_rst_task_indicator = NO_INDICATION_CAP_RST;
data_transmit_indicator_t data_transmit_indicator = NO_INDICATION_DATA_TRANS;
bool ctrl_power_on = true;
SemaphoreHandle_t xSemaphoreTaskIndicators;

/* Binary semaphore to shut down if low voltage */
SemaphoreHandle_t xSemaphoreBattLowV;


/* Check flag for startup from sleep */
bool enter_from_sleep = false;

/* Pwr management DroneID on/off */
droneid_pwr_state_t droneid_pwr_state = DRONEID_PWR_OFF;
SemaphoreHandle_t xSemaphorePwrMan;

/* I2C periph mutex and callback variable */
bool i2c_data_received = false;
SemaphoreHandle_t xSemaphoreI2CPeriph;

// int var magnetometer
bool newMagData = false;

/* Mutex to stop from pwr off when reset btn */
SemaphoreHandle_t xSemaphoreBtnCtrl;

/* Mutex for Ublox Tx */
SemaphoreHandle_t xSemaphoreUbloxTransmitState;

/*Protected barometric data */
SemaphoreHandle_t xSemaphoreBarometricData;
float pressure_baro_var = 0.0;
float temperature_baro_var = 0.0;

/*Protected msg ID */
SemaphoreHandle_t xSemaphoreServerFlightMsgs;

uint8_t echo_msg_count_cell_id = 5;
uint8_t echo_start_msg_count = 5;
uint8_t echo_stop_msg_count = 5;
uint8_t echo_cell_info_msg_count = 5;
uint8_t send_msg_count_cell_id = 0;
uint8_t server_msg_count_id = 0;
//uint8_t server_echo_msg_count_id = 0;
uint8_t server_msg_time_interval = 1;
uint8_t new_server_msg_time_interval = 1;
uint8_t max_horizontal_server_msg = 50;
uint8_t max_vertical_server_msg = 20;
uint8_t flight_permission = SERVER_NOT_CONNECTED;
uint8_t serv_cmd = 0;

/* Protected cell id and area code */
SemaphoreHandle_t xSemaphoreCellId;
bool received_resp_cell_id = false;
uint16_t mob_country_code = 0;
uint16_t mob_network_code = 0;
uint16_t mob_location_area_code = 0;
uint32_t mob_cell_id_code = 0;
uint16_t pre_mob_location_area_code = 0;
uint32_t pre_mob_cell_id_code = 0;

/* Queue to add data msg */
QueueHandle_t xQueueSlipData;

/* Var for uart transmit */
uint8_t ublox_trasnmit_complete = APP_FALSE;
uint8_t debug_trasnmit_complete = APP_FALSE;

/* Ublox mutex protected data register */
SemaphoreHandle_t xSemaphoreUbloxStatusReg;
ublox_reg_t ublox_status_reg;

/* Protected gga msg */
SemaphoreHandle_t xSemaphoreGgaMsg;
gpgga_t gga_data;

/* Protected gsv msg */
SemaphoreHandle_t xSemaphoreGsvMsg;
gpgsv_t gpgsv_data[4];
gpgsv_t glgsv_data[4];

/* Binary semaphore to shut down gsm and gnss modem before entering standby */
SemaphoreHandle_t xSemaphoreGsmPwrIsDown;

/* Binary semaphore to shut down barometer */
SemaphoreHandle_t xSemaphoreBaroPwrIsDown;

/* Binary semaphore to shut down imu */
SemaphoreHandle_t xSemaphoreImuPwrIsDown;

/* Queue to new ADC sample */
QueueHandle_t xQueueNewAdcSample;
uint16_t adc_data;

/* Protected battery voltage var */
SemaphoreHandle_t xSemaphoreBattVoltage;
float batteryVoltage = 0.0;
float initialBatteryVoltage = 0.0;

/* Receive queues */
QueueHandle_t xQueueUbloxReceive;
QueueHandle_t xQueueAuxReceive;
QueueHandle_t xQueueUartTransmit;

/* Receive buffers */
uint8_t uart3_rec_buffer[10];
uint8_t uart6_rec_buffer[10];

// Timer to update gsm info
TimerHandle_t xHandleGSMInfoTimer;
/* Mutex to indicate timer expired */
SemaphoreHandle_t xSemaphoreGSMInfoTimerExpired;

// Timer to flash indicator leds
TimerHandle_t xHandleIndicatorGPTimer;
/* Mutex to indicate timer expired */
SemaphoreHandle_t xSemaphoreIndicatorTimerExpired;

// Timer to wait for transmit pos
TimerHandle_t xHandleTransmitTimer;
/* Mutex to indicate timer expired */
SemaphoreHandle_t xSemaphoreTransmitPos;

// Timer to wait for ggs fetch
TimerHandle_t xHandleGetGgaTimer;
/* Mutex to indicate timer expired */
SemaphoreHandle_t xSemaphoreGetGgga;

#ifdef TRANSMIT_GSV_DATA
// Timer to wait for transmit gsv
TimerHandle_t xHandleGsvGPTimer;
/* Mutex to indicate timer expired */
SemaphoreHandle_t xSemaphoreGsvTimerExpired;
#endif

// Timer to wait for gsm response
TimerHandle_t xHandleVoltAdcGPTimer;
/* Mutex to indicate timer expired */
SemaphoreHandle_t xSemaphoreVoltAdcTimerExpired;

/***************************************************************************/
/* Timer callback read gsm info */
/***************************************************************************/
void gsm_info_timer_callback(TimerHandle_t xHandleGSMInfoTimer)
{
  xSemaphoreGive( xSemaphoreGSMInfoTimerExpired );
}

/***************************************************************************/
/* Timer callback to LED indicator */
/***************************************************************************/
void indicator_task_timer_callback(TimerHandle_t xHandleIndicatorGPTimer)
{
  xSemaphoreGive( xSemaphoreIndicatorTimerExpired );
}

/***************************************************************************/
/* Timer callback to transmit timer */
/***************************************************************************/
void transmit_task_timer_callback(TimerHandle_t xHandleTransmitTimer)
{
  xSemaphoreGive( xSemaphoreTransmitPos );
}

/***************************************************************************/
/* Timer callback to get gga msg */
/***************************************************************************/
void get_gga_timer_callback(TimerHandle_t xHandleGetGgaTimer)
{
  xSemaphoreGive( xSemaphoreGetGgga );
}

#ifdef TRANSMIT_GSV_DATA
/***************************************************************************/
/* Timer callback to transmit timer */
/***************************************************************************/
void gsv_task_timer_callback(TimerHandle_t xHandleGsvGPTimer)
{
  xSemaphoreGive( xSemaphoreGsvTimerExpired );
}
#endif
/***************************************************************************/
/* Timer callback to transmit timer */
/***************************************************************************/
void volt_task_timer_callback(TimerHandle_t xHandleVoltAdcGPTimer)
{
  xSemaphoreGive( xSemaphoreVoltAdcTimerExpired );
}

/***************************************************************************/
/* Transmit aux uart */
/***************************************************************************/
void transmit_uart_debug(uint8_t *str)
{
  uint16_t size;
  for(size = 0; str[size]!='\000'; size++){}

  if(HAL_UART_Transmit_DMA(&huart6, str, size)!= HAL_OK)
  {
    Error_Handler();
  }
  
  while(debug_trasnmit_complete != APP_TRUE)
  {
    vTaskDelay(2 / portTICK_RATE_MS);
  }
  debug_trasnmit_complete = APP_FALSE;
}

/***************************************************************************/
/* Check if wakeup from sleep */
/***************************************************************************/
uint8_t startup_check(void)
{
  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
  {
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

    enter_from_sleep = true;
  }
}

/***************************************************************************/
/* Init peripheals */
/***************************************************************************/
uint8_t init_peripherals(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
}

/***************************************************************************/
/* Main task */
/***************************************************************************/
void main(void)
{
  init_peripherals();
  startup_check();

  HAL_StatusTypeDef error_msg;
  error_msg = HAL_UART_Receive_DMA(&huart3, uart3_rec_buffer, sizeof( uint8_t ));
  error_msg = HAL_UART_Receive_IT(&huart6, uart6_rec_buffer, sizeof( uint8_t ));

  /* Mutex and semaphores */

  /* Binary semaphore to shut down imu */
  xSemaphoreImuPwrIsDown = xSemaphoreCreateMutex();

  /* Mutex for Ublox Tx */
  xSemaphoreUbloxTransmitState = xSemaphoreCreateMutex();

  /* Pwr management DroneID on/off */
  xSemaphorePwrMan = xSemaphoreCreateMutex();

/* I2C periph mutex */
  xSemaphoreI2CPeriph = xSemaphoreCreateMutex();

  /* Mutex to stop from pwr off when reset btn */
  xSemaphoreBtnCtrl = xSemaphoreCreateMutex();

  /*Protected barometric data */
  xSemaphoreBarometricData = xSemaphoreCreateMutex();

  /*Protected msg ID */
  xSemaphoreServerFlightMsgs = xSemaphoreCreateMutex();

  /* Protected cell id and area code */
  xSemaphoreCellId = xSemaphoreCreateMutex();
  
  /* variables to define indication with LEDs */
  xSemaphoreTaskIndicators = xSemaphoreCreateMutex();

  /* Mutex handle only allow one task to print at a time */
//  xSemaphoreTrace = xSemaphoreCreateMutex();

/* Queue to add data msg */
    xQueueSlipData = xQueueCreate( 512, sizeof( uint8_t ) );

  /* Ublox mutex protected data register */
  xSemaphoreUbloxStatusReg = xSemaphoreCreateMutex();

/* Protected gga and gsv msg */
  xSemaphoreGgaMsg = xSemaphoreCreateMutex();
  xSemaphoreGsvMsg = xSemaphoreCreateMutex();

  /* Binary semaphore to shut down if low voltage */
  xSemaphoreBattLowV = xSemaphoreCreateBinary();

  /* Binary semaphore to shut down gsm and gnss modem before entering standby */
  xSemaphoreGsmPwrIsDown = xSemaphoreCreateBinary();

  /* Binary semaphore to shut down IMU */
  xSemaphoreBaroPwrIsDown = xSemaphoreCreateBinary();

  /* Binary semaphore to indicate new ADC sample ready */
  xQueueNewAdcSample = xQueueCreate( 1, sizeof( uint16_t ) );

  /* Protected battery voltage var */
  xSemaphoreBattVoltage = xSemaphoreCreateMutex();

  // Timer to update gsm info
  xSemaphoreGSMInfoTimerExpired = xSemaphoreCreateBinary();
  xHandleGSMInfoTimer = xTimerCreate("gsmInfoTimer", 20000, pdTRUE, ( void * ) 0, gsm_info_timer_callback);

  // Timer to flash indicator leds
  xSemaphoreIndicatorTimerExpired = xSemaphoreCreateBinary();
  xHandleIndicatorGPTimer = xTimerCreate("IndicatorTimer", 1000, pdTRUE, ( void * ) 0, indicator_task_timer_callback);

  /* Timer and binary semaphore to wait for transmit pos */
  xSemaphoreTransmitPos = xSemaphoreCreateBinary();
  xHandleTransmitTimer = xTimerCreate("TransmitTimer", 1000, pdTRUE, ( void * ) 0, transmit_task_timer_callback);

  /* Timer and binary semaphore to wait for get gga */
  xSemaphoreGetGgga = xSemaphoreCreateBinary();
  xHandleGetGgaTimer = xTimerCreate("GetGgaTimer", 1000, pdTRUE, ( void * ) 0, get_gga_timer_callback);

#ifdef TRANSMIT_GSV_DATA
  /* Timer and binary semaphore to wait for transmit gsv */
  xSemaphoreGsvTimerExpired = xSemaphoreCreateBinary();
//  xHandleGsvGPTimer = xTimerCreate("CtrlResponseTimer", 5000, pdTRUE, ( void * ) 0, gsv_task_timer_callback);
  xHandleGsvGPTimer = xTimerCreate("CtrlResponseTimer", 20000, pdTRUE, ( void * ) 0, gsv_task_timer_callback);
#endif

  /* Timer for measuring voltage */ 
  xSemaphoreVoltAdcTimerExpired = xSemaphoreCreateBinary();
  xHandleVoltAdcGPTimer = xTimerCreate("VoltAdcTimer", 100, pdTRUE, ( void * ) 0, volt_task_timer_callback);

  /* Queues */
  xQueueUbloxReceive = xQueueCreate( 512, sizeof( uint8_t ) );
  xQueueAuxReceive = xQueueCreate( 20, sizeof( uint8_t ) );
  xQueueUartTransmit = xQueueCreate( 256, sizeof( uint8_t ) );

  /* Debug queue */
#if defined(DEBUG) || defined(DEBUG_BARO)
  xQueueDebugQueue = xQueueCreate( DEBUG_QUEUE_SIZE, sizeof( uint8_t ) );
  xSemaphoreDebugMutex = xSemaphoreCreateMutex();
#endif

  /* Tasks */
  xTaskCreate( pwr_management_main, "PwrManagement", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY+1, &xHandlePwrManagementTask );
  xTaskCreate( cap_btn_reset_main, "CapBtnReset", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY+1, &xHandleCapBtnResetTask );
  xTaskCreate( ublox_data_reader_main, "UbloxDataReader", 1024, ( void * ) NULL, tskIDLE_PRIORITY+10, &xHandleUbloxDataReaderTask );
  xTaskCreate( uart_data_task, "UbloxDataWriter", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY+1, &xHandleUartTransmitTask );
  xTaskCreate( droneid_ctrl_main, "UbloxCtrl", 4096, ( void * ) NULL, tskIDLE_PRIORITY+3, &xHandleDroneidCtrlTask );

  xTaskCreate( pos_est_task, "posEst", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY+1, &xHandlePosEstTask );
//  xTaskCreate( msg_to_server_task, "MsgToServer", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY+1, &xHandleMsgToServerTask );

  xTaskCreate( battery_voltage_main, "BattVolt", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY+1, &xHandleBatteryVoltageTask );
  xTaskCreate( indicator_main, "indicator", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY+1, &xHandleIndicatorTask );
  xTaskCreate( barometric_sens_main, "bamoretric", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY+1, &xHandleBaroSensTask );
  xTaskCreate( imu_main, "imu", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY+1, &xHandleImuTask );

#if defined(DEBUG) || defined(DEBUG_BARO)
  xTaskCreate( debug_main_task, "debug", 2048, ( void * ) NULL, tskIDLE_PRIORITY+1, &xHandleDebugTask );
#endif

  vTaskStartScheduler();
  // Should never come here!!!
  NVIC_SystemReset();

//  debug_exit(0);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  i2c_data_received = true;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  xQueueSendToBackFromISR( xQueueNewAdcSample,  (void *) &adc_data, NULL );
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  if(UartHandle == &huart3)
  {
    ublox_trasnmit_complete = APP_TRUE;
  }
  else if (UartHandle == &huart6)
  {
    debug_trasnmit_complete = APP_TRUE;
  }
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  if(UartHandle == &huart3)
  {
    xQueueSendToBackFromISR( xQueueUbloxReceive, uart3_rec_buffer, NULL );
    HAL_UART_Receive_IT(&huart3, uart3_rec_buffer, sizeof( uint8_t ));
  }
  else if (UartHandle == &huart6)
  {
//    xQueueSendToBackFromISR( xQueueUbloxReceive, uart6_rec_buffer, NULL );
//    HAL_UART_Receive_IT(&huart6, uart6_rec_buffer, sizeof( uint8_t ));
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); //5,0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  NVIC_SystemReset(); // Reset mcu
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

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
  NVIC_SystemReset(); // Reset mcu
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
