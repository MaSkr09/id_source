/****************************************************************************
* Copyright (c) 2016, Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
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
* File: battery_voltage_task.c
* Purpose: Measure battery voltage
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2016-10-01 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* Includes */
/***************************************************************************/
#include "main.h"
#include "adc.h"
#include "battery_voltage_task.h"
#include "id_config.h"

/***************************************************************************/
/* Defines */
/***************************************************************************/
#define DELAY_50_MS                 50
#define FILTER_INST_NO              30

/***************************************************************************
 * get power state
/***************************************************************************/
bool get_power_state(void)
{
  bool power_on = false;
  if( xSemaphoreTake( xSemaphorePwrMan, ( TickType_t ) 1000 ) == pdTRUE )
  {
    if(droneid_pwr_state == DRONEID_PWR_ON)
      power_on = true;
    xSemaphoreGive( xSemaphorePwrMan );
  }

  return power_on;
}

/***************************************************************************
 * Get sensor value from queue
/***************************************************************************/
bool get_battery_voltage(float *voltage)
{
  bool success = false;
  uint16_t buf;
  uint32_t sum = 0;
  uint16_t counter = 0;
  
  while( counter <= FILTER_INST_NO)
  {
    if( xQueueReceive( xQueueNewAdcSample, (&buf), ( TickType_t ) portMAX_DELAY ) )
    {
      sum += buf;
      counter++;
      if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_data, 1) != HAL_OK)
      {
        Error_Handler(); 
      }
    }
  }

  if(!(counter <= FILTER_INST_NO))
  {
    *voltage = (float)((double)0.001465201 * (double)sum/(double)counter);
    success = true;
  }
  return success;
}

/***************************************************************************
 * Init voltage measurement
/***************************************************************************/
bool init_voltage_measure(void)
{
  bool success = false;
  uint8_t i;
  float measurement;
  /* Wait until powered up */
  while(!get_power_state())
    vTaskDelay(200 / portTICK_RATE_MS);

  HAL_GPIO_WritePin(GPIO_BATT_VOLT_EN_O_GPIO_Port, GPIO_BATT_VOLT_EN_O_Pin, SET);

  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_data, 1) != HAL_OK)
  {
    Error_Handler();
  }

  for (i=0;i<10;i++)
  {
    success = get_battery_voltage(&measurement);
  }
  return success;
}

/***************************************************************************
 * Power down measurement
/***************************************************************************/
bool power_down_volt_measurement(void)
{
  bool success = false;
  HAL_GPIO_WritePin(GPIO_BATT_VOLT_EN_O_GPIO_Port, GPIO_BATT_VOLT_EN_O_Pin, RESET);
  xSemaphoreGive( xSemaphoreBaroPwrIsDown );
  vTaskDelay(1000 / portTICK_RATE_MS);
  vTaskSuspend( NULL );
  return success;
}

/***************************************************************************
 * Update voltage variable
/***************************************************************************/
bool update_parameter(float *var, float buf)
{
  bool success = false;
  if( xSemaphoreTake( xSemaphoreBattVoltage, ( TickType_t ) 200 ) == pdTRUE )
  {
    *var = buf;
    xSemaphoreGive( xSemaphoreBattVoltage );
  }
  return success;
}

/***************************************************************************
 * Measure battery voltage
/***************************************************************************/
void battery_voltage_main(void *pvParameters)
{
  bool pwr_on = false;
  float measurement;
  uint8_t low_v_cnt = 0;
  if(init_voltage_measure())
  {
    if(get_battery_voltage(&measurement))
      update_parameter(&initialBatteryVoltage, measurement);

    while(!get_power_state())
      vTaskDelay(50/ portTICK_RATE_MS);

    xTimerStart( xHandleVoltAdcGPTimer, 1000 );

    TASK_LOOP
    {
      if(xSemaphoreTake( xSemaphoreVoltAdcTimerExpired, ( TickType_t ) DELAY_50_MS) == pdTRUE)
      {
        if(get_battery_voltage(&measurement))
          update_parameter(&batteryVoltage, measurement);

        if(measurement < LOW_V_LIM)
        {
          low_v_cnt++;
          if(low_v_cnt >= LOW_V_CNT_LIM)
          {
            xSemaphoreGive(xSemaphoreBattLowV);
            vTaskResume( xHandlePwrManagementTask );
          }
        }
        else
        {
          low_v_cnt = 0;
        }
      }
      if(!get_power_state())
        while(!power_down_volt_measurement());
    }
  }
}