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
* File: debug_task.c
* Purpose: Debug print when debug queue contains data
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-05-29 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* Includes */
/***************************************************************************/
#include "main.h"
#if defined(DEBUG) || defined(DEBUG_BARO)
#include <__cross_studio_io.h>
#include "debug_task.h"

/***************************************************************************/
/* Private defines */
/***************************************************************************/
#define DEBUG_ARRAY_SIZE  1024
#define TASK_LOOP_MS      5
#define MUTEX_WAIT_LIM_MS 2

/***************************************************************************/
/* Add data to debug queue */
/***************************************************************************/
void debug_add_to_queue(uint8_t *str)
{
  uint16_t offset = 0;
  if( xSemaphoreTake( xSemaphoreDebugMutex, ( TickType_t ) MUTEX_WAIT_LIM_MS ) == pdTRUE )
  {
    while( *(str+offset) != 0 )
    {
      xQueueSendToBack(xQueueDebugQueue, (str+offset), 0);
      offset++;
    }
    xSemaphoreGive(xSemaphoreDebugMutex);
  }
}

/***************************************************************************/
/* Debug main task */
/***************************************************************************/
void debug_main_task(void *pvParameters)
{
  vTaskDelay(TASK_STARTUP_DELAY_MS/portTICK_RATE_MS);
#if defined(DEBUG) || defined(DEBUG_BARO)
  uint16_t offset = 0;
  uint8_t debug_array[DEBUG_ARRAY_SIZE];
  TASK_LOOP
  {
    while( xQueueReceive( xQueueDebugQueue, (debug_array+offset), ( TickType_t ) 1) && (offset < DEBUG_ARRAY_SIZE) )
    {
      offset++;
    }

    if(offset > 0)
    {
      *(debug_array+offset) = 0;
      debug_printf(debug_array);
    }
    vTaskDelay(TASK_LOOP_MS / portTICK_RATE_MS);
    offset = 0;
  }
#endif
  // Suspend task.
  vTaskSuspend( NULL );
}
#endif
