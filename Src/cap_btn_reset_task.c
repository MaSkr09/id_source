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
* File: cap_btn_reset.c
* Purpose: Reset touch button
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

#include "cap_btn_reset_task.h"
#include "main.h"

/***************************************************************************/
/* Private defines */
/***************************************************************************/
#define USB_SIGNAL_RESET_MIN_S          15
#define USB_SIGNAL_RESET_MAX_S          25

#define CAP_BTN_RESET_TIME_MS           3000
#define CAP_BTN_RESET_DELAY_MS          3000

#define DEFAULT_SEMAPHORE_TIME_FRAME_MS 50
#define DEFAULT_TRACE_TIME_FRAME_MS     100
#define DALAY_1000_MS                   1000
 
/***************************************************************************
 * Reset cap btn function
/***************************************************************************/
void reset_btn(void)
{
#ifdef DEBUG
  debug_add_ascii_to_queue("cap btn: reset in progress\n");
#endif
  vTaskDelay(DALAY_1000_MS / portTICK_RATE_MS);
  HAL_GPIO_WritePin(GPIO_CAP_BTN_PWR_O_GPIO_Port, GPIO_CAP_BTN_PWR_O_Pin, GPIO_PIN_SET);
  vTaskDelay(CAP_BTN_RESET_TIME_MS / portTICK_RATE_MS);
  HAL_GPIO_WritePin(GPIO_CAP_BTN_PWR_O_GPIO_Port, GPIO_CAP_BTN_PWR_O_Pin, GPIO_PIN_RESET);
  vTaskDelay(CAP_BTN_RESET_DELAY_MS / portTICK_RATE_MS);
#ifdef DEBUG
  debug_add_ascii_to_queue("cap btn: reset done\n");
#endif
}


/***************************************************************************
 * Reset cap btn task
 * Required if it has been touched while initializing 
/***************************************************************************/
void cap_btn_reset_main(void *pvParameters)
{
  vTaskDelay(TASK_STARTUP_DELAY_MS / portTICK_RATE_MS);
#ifdef DEBUG
  debug_add_ascii_to_queue("cap btn: task started\n");
#endif
  uint8_t timer;

  TASK_LOOP
  {
    // The task suspends itself. Wakeup from interrupt if USB is attached
    vTaskSuspend( NULL );
    timer = 0;
    /* Count how long the USB detect signal is high */
    while((HAL_GPIO_ReadPin(GPIO_VUSB_DETECT_I_GPIO_Port, GPIO_VUSB_DETECT_I_Pin)) && (timer < USB_SIGNAL_RESET_MIN_S))
    {
      timer++;
      vTaskDelay(DALAY_1000_MS / portTICK_RATE_MS);
    }
    // If USB still attached then continue reset states 
    if(HAL_GPIO_ReadPin(GPIO_VUSB_DETECT_I_GPIO_Port, GPIO_VUSB_DETECT_I_Pin))
    {
#ifdef DEBUG
      debug_add_ascii_to_queue("cap btn: reset time frame\n");
#endif
      /* Take semaphore to avoid pwr down under reset*/
      if( xSemaphoreTake( xSemaphoreBtnCtrl, ( TickType_t ) DEFAULT_SEMAPHORE_TIME_FRAME_MS ) == pdTRUE )
      {
        // Change indicator signal to show user time frame to reset 
        if( xSemaphoreTake( xSemaphoreTaskIndicators, ( TickType_t ) DEFAULT_SEMAPHORE_TIME_FRAME_MS ) == pdTRUE )
        {
          cap_rst_task_indicator = RESET_TIME_FRAME_CAP_RST;
          xSemaphoreGive( xSemaphoreTaskIndicators );
        }
        // Time fra for user to detach USB
        while((HAL_GPIO_ReadPin(GPIO_VUSB_DETECT_I_GPIO_Port, GPIO_VUSB_DETECT_I_Pin)) && (timer < USB_SIGNAL_RESET_MAX_S))
        {
          timer++;
          vTaskDelay(DALAY_1000_MS / portTICK_RATE_MS);
        }

        // Check if USB is low within time frame defined
        if((timer > USB_SIGNAL_RESET_MIN_S) && (timer < USB_SIGNAL_RESET_MAX_S))
        {
            /* Change indicator signal */
            if( xSemaphoreTake( xSemaphoreTaskIndicators, ( TickType_t ) DEFAULT_SEMAPHORE_TIME_FRAME_MS ) == pdTRUE )
            {
              cap_rst_task_indicator = RESET_BTN_CAP_RST;
              xSemaphoreGive( xSemaphoreTaskIndicators );
            }
            /* Reset btn */
            reset_btn();
          }
          xSemaphoreGive( xSemaphoreBtnCtrl );
      }
    }

    /* Change indicator state to none*/
    if( xSemaphoreTake( xSemaphoreTaskIndicators, ( TickType_t ) DEFAULT_SEMAPHORE_TIME_FRAME_MS ) == pdTRUE )
    {
      cap_rst_task_indicator = NO_INDICATION_CAP_RST;
      xSemaphoreGive( xSemaphoreTaskIndicators );
    }
  }
}


