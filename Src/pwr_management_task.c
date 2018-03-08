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
* File: pwr_management_task.c
* Purpose: Power management of DroneID devise
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2016-10-01 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* Includes */
/***************************************************************************/

#ifdef DEBUG
#include "debug_task.h"
#endif
#include "main.h"
#include "pwr_management_task.h"

/***************************************************************************/
/* Private defines */
/***************************************************************************/
#define MAXIMUM_CAP_BTN_HIGH_TIME_S     10
#define SUSPENSION_DELAY_MS             1000

#define DEFAULT_SEMAPHORE_TIME_FRAME_MS 5000
#define DEFAULT_TRACE_TIME_FRAME_MS     100
#define DRONEID_BTN_TOUCH_MIN_TIME_MS   2500
#define DRONEID_BTN_TOUCH_MAX_TIME_MS   6000
#define DELAY_1_MS                      1
#define DELAY_1000_MS                   1000
#define TASK_DELAY_50_MS                50

#define GSM_PWR_DOWN_TIME_FRAME_MS      45000
#define IMU_PWR_DOWN_TIME_FRAME_MS      45000

/***************************************************************************/
/* Check for btn error */
/***************************************************************************/
void check_btn_error(void)
{
  /* After a reset the cap btn should go low within a defined time interval 
   * If it doesn't go low then a reset has to be performed by mounting the
   * USB charger in the connector and remove it after a defined time     
   */
  uint8_t counter = 0;
  do
  {
    vTaskDelay(1000 / portTICK_RATE_MS);
    counter++;
  }while((HAL_GPIO_ReadPin(GPIO_CAP_BUTTON_I_GPIO_Port,GPIO_CAP_BUTTON_I_Pin)) && (counter != MAXIMUM_CAP_BTN_HIGH_TIME_S));

  /* Cap btn is in an error state and needs to be reset before it is 
   * ready to use 
   */
  if(counter == MAXIMUM_CAP_BTN_HIGH_TIME_S)
  {
    // Signal LED
    vTaskResume( xHandleCapBtnResetTask );
  }
}

/***************************************************************************/
/* Suspend mcu function */
/***************************************************************************/
void suspend_mcu(void)
{
  vTaskDelay(SUSPENSION_DELAY_MS / portTICK_RATE_MS);
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
// Is done after wake up
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  __HAL_PWR_CLEAR_FLAG(PWR_CR_CSBF);
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

  HAL_PWR_DeInit();
//  HAL_DeInit();
//  
//  HAL_TIM_PWM_MspDeInit();


  HAL_PWR_EnterSTANDBYMode();
}

/***************************************************************************/
/* Check if btn is pressed in wakeup time frame */
/***************************************************************************/
void change_indicator(pwr_task_indicator_t signal)
{
  if( xSemaphoreTake( xSemaphoreTaskIndicators, ( TickType_t ) DEFAULT_SEMAPHORE_TIME_FRAME_MS ) == pdTRUE )
  {
    pwr_task_indicator = signal;
    xSemaphoreGive( xSemaphoreTaskIndicators );
  }
}

/***************************************************************************/
/* Check if btn is pressed in wakeup time frame */
/***************************************************************************/
bool btn_touch_detected()
{
  bool accepted_btn_press_return = true;
  uint32_t cnt = 0;

  // Change indicator signal to btn pushed
  change_indicator(BTN_SIGNAL_HIGH_PWR_MGMT);

  // Time btn pressed before DroneID wakeup
  while(HAL_GPIO_ReadPin(GPIO_CAP_BUTTON_I_GPIO_Port,GPIO_CAP_BUTTON_I_Pin) && cnt < DRONEID_BTN_TOUCH_MIN_TIME_MS)
  {
    cnt++;
    vTaskDelay(DELAY_1_MS / portTICK_RATE_MS);
  }

  // Change indicator signal to release time frame 
  if(HAL_GPIO_ReadPin(GPIO_CAP_BUTTON_I_GPIO_Port,GPIO_CAP_BUTTON_I_Pin))
    change_indicator(BTN_RELEASE_SIGN_MGMT);
  else
    change_indicator(NO_INDICATION_PWR);

  // Time frame to move finger from touch btn 
  while((HAL_GPIO_ReadPin(GPIO_CAP_BUTTON_I_GPIO_Port,GPIO_CAP_BUTTON_I_Pin)) && cnt < DRONEID_BTN_TOUCH_MAX_TIME_MS)
  {
    cnt++;
    vTaskDelay(DELAY_1_MS / portTICK_RATE_MS);
  }

  // Check btn push time frame for startup
  if((!(cnt > DRONEID_BTN_TOUCH_MIN_TIME_MS-1)) || (cnt > DRONEID_BTN_TOUCH_MAX_TIME_MS-1))
  {
    accepted_btn_press_return = false;
  }
  return accepted_btn_press_return;
}

/***************************************************************************/
/* Power down DroneID CPU */
/***************************************************************************/
void pwr_down_droneid()
{
  change_indicator(NO_INDICATION_PWR);
  // Make sure btn is released before shutdown
  while(HAL_GPIO_ReadPin(GPIO_CAP_BUTTON_I_GPIO_Port,GPIO_CAP_BUTTON_I_Pin)){}

#ifdef DEBUG
  debug_add_ascii_to_queue("pwr management task: Suspending MCU \n");
#endif
  
  vTaskDelay(TASK_DELAY_50_MS / portTICK_RATE_MS);
  suspend_mcu();
}

/***************************************************************************/
/* Change power signal to other tasks */
/***************************************************************************/
void change_pwr_mode(droneid_pwr_state_t mode)
{
  // Pwr management DroneID on/off
  while( !(xSemaphoreTake( xSemaphorePwrMan, ( TickType_t ) DELAY_1000_MS ) == pdTRUE ))
  {}
  droneid_pwr_state = mode;
  xSemaphoreGive( xSemaphorePwrMan );
}

/***************************************************************************/
/* Check if mode is on */
/***************************************************************************/
bool power_mode_is_on(void)
{
  bool pwr_mode_on = false;
  // Pwr management DroneID on/off
  if( xSemaphoreTake( xSemaphorePwrMan, ( TickType_t ) DELAY_1000_MS ) == pdTRUE )
  {
    if(droneid_pwr_state == DRONEID_PWR_ON)
      pwr_mode_on = true;
    xSemaphoreGive( xSemaphorePwrMan );
  }
  return pwr_mode_on;
}

/***************************************************************************/
/* Get power mode */
/***************************************************************************/
droneid_pwr_state_t get_power_mode(void)
{
  droneid_pwr_state_t pwr_mode;
  // Pwr management DroneID on/off
  if( xSemaphoreTake( xSemaphorePwrMan, ( TickType_t ) DELAY_1000_MS ) == pdTRUE )
  {
    pwr_mode = droneid_pwr_state;
    xSemaphoreGive( xSemaphorePwrMan );
  }
  return pwr_mode;
}

/***************************************************************************/
/* Power management task                                                   */
/* Start DroneID if button if btn released in time or shut down again      */
/***************************************************************************/
void pwr_management_main(void *pvParameters)
{
#ifdef DEBUG
  debug_add_ascii_to_queue("pwr management task: Started\n");
#endif
  uint32_t cnt = 0;

  // Check if btn is pushed to start droneID
  if(enter_from_sleep)
  {
    if(!btn_touch_detected())
    {
      // Power down CPU
      pwr_down_droneid();
    }
  }

  // Enter task loop
  TASK_LOOP
  {
    // Pwr management DroneID on
    change_pwr_mode(DRONEID_PWR_ON);
    vTaskDelay(TASK_DELAY_50_MS / portTICK_RATE_MS);
    // Change indicator to show states from other tasks
    change_indicator(NO_INDICATION_PWR);

    if( xSemaphoreTake( xSemaphoreBattLowV, ( TickType_t ) 0) == pdTRUE )
    {
      change_pwr_mode(DRONEID_PWR_OFF_LOW_BATT);
    }

    // Take btn semaphore
    if( xSemaphoreTake( xSemaphoreBtnCtrl, ( TickType_t ) DEFAULT_SEMAPHORE_TIME_FRAME_MS ) == pdTRUE )
    {
      if(btn_touch_detected())
      {
        // Change indicator signal to shutdown
        change_pwr_mode(DRONEID_PWR_OFF);
      }
      xSemaphoreGive( xSemaphoreBtnCtrl );
    }
#ifdef DEBUG
    else
    {
      debug_add_ascii_to_queue("Failed to take semaphore\n");
    }
#endif
    if((get_power_mode() == DRONEID_PWR_OFF) || (get_power_mode() == DRONEID_PWR_OFF_LOW_BATT))
    {
      change_indicator(CPU_PWR_DOWN_PWR_MGMT);
      // Wait for gsm and imu to power down
      xSemaphoreTake( xSemaphoreGsmPwrIsDown, ( TickType_t ) GSM_PWR_DOWN_TIME_FRAME_MS);
      xSemaphoreTake( xSemaphoreBaroPwrIsDown, ( TickType_t ) IMU_PWR_DOWN_TIME_FRAME_MS);

      pwr_down_droneid();
    }
 
    // Suspend task until btn is pushed
    vTaskSuspend( NULL );
  }
}
