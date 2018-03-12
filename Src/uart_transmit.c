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
* File: uart_transmit.c
* Purpose: Build string to send to ublox SARA G3 module
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-01-02 Martin Skriver, Source written
* Created:  2018-03-07 Martin Skriver, File renamed to be generic
****************************************************************************/
#ifdef DEBUG
#include "debug_task.h"
#endif

#include "uart_transmit.h"
#include "main.h"
#include "usart.h"
#include "ublox_reg.h"
#include "global_defines.h"

#include <string.h>

#define MAX_AURT_TANSMIT_ERROR        5
#define TRANSMIT_TIMEOUT_MS           100
#define TRANSMIT_LOOP_MS              2

/***************************************************************************/
/* Add ascii data to send queue */
/***************************************************************************/
bool add_ascii_to_send_queue(uint8_t *data)
{
  bool return_value = true;
  while(*data != NULL)
  {
    if( xQueueSendToBack( xQueueUartTransmit, ( void * ) data++, ( TickType_t ) TIME_10_MS ) != pdPASS )
    {
      return_value = false;
    }
  }
  return return_value;
}

/***************************************************************************/
/* Add fixed size data to send queue */
/***************************************************************************/
bool add_data_to_send_queue(uint8_t *data, uint16_t data_len)
{
  bool return_value = true;
  uint16_t data_cnt = 0;
  while(data_cnt < data_len)
  {
    if( xQueueSendToBack( xQueueUartTransmit, ( void * ) data++, ( TickType_t ) TIME_10_MS ) != pdPASS )
    {
      return_value = false;
    }
    data_cnt++;
  }
  return return_value;
}

/***************************************************************************/
/* Transmit array */
/***************************************************************************/
bool uart_transmit_data(uint8_t *str, uint8_t array_size)
{
  bool success = false;
  uint16_t timeout_counter = 0;
  if(HAL_UART_Transmit_DMA(&huart3, str, array_size)!= HAL_OK)
  {
    Error_Handler();
  }
  
  while((ublox_trasnmit_complete != APP_TRUE) && (timeout_counter < TRANSMIT_TIMEOUT_MS))
  {
    vTaskDelay(TRANSMIT_LOOP_MS / portTICK_RATE_MS);
    timeout_counter += TRANSMIT_LOOP_MS;
  }

  if(ublox_trasnmit_complete == APP_TRUE)
  {
    success = true;
  }

  ublox_trasnmit_complete = APP_FALSE;
  return success;
}

/***************************************************************************/
/* Sends data via uart if available in queue */
/***************************************************************************/
void uart_data_task(void *pvParameters)
{
  uint8_t msg[TX_ARRAY_SIZE];
  uint16_t msg_len, error_count = 0;
  bool send_msg = false;

  TASK_LOOP
  {
    msg_len = 0;
    if( xQueueReceive( xQueueUartTransmit, (msg), ( TickType_t ) TIME_1000_MS ) )
    {
      msg_len++;
      msg[msg_len] = NULL;
      while(xQueueReceive( xQueueUartTransmit, (msg + msg_len), ( TickType_t ) TIME_1_MS ))
      {
        msg_len++;
        msg[msg_len] = NULL;
      }
      if(!uart_transmit_data(msg, msg_len))
      {
        error_count++;
      }
#ifdef DEBUG
// Do dirty stuff to avoid printing binary protocol msg
      if(!send_msg)
      {
        debug_add_data_to_send_queue(msg, msg_len);

        if(strncmp(msg+3, AT_SEND_TO_SERVER, sizeof(AT_SEND_TO_SERVER)-1)==0)
        {
          send_msg = true;
        }
      }
      else
      {
        send_msg = false;
      }
#endif
    }
    if(error_count >= MAX_AURT_TANSMIT_ERROR)
    {
    // TODO reboot
    }
  }
}