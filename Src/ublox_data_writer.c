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
* File: ublox_data_writer.c
* Purpose: Build string to send to ublox SARA G3 module
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-01-02 Martin Skriver, Source written
* TODO CHANGE NAME
****************************************************************************/
#include "ublox_data_writer.h"
#include "main.h"
#include "usart.h"
#include "ublox_reg.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#define RESET_TIME_MS                 60
#define PWR_ON_TIME_MS                10

#define TRANSMIT_TIMEOUT_MS         100
#define TRANSMIT_LOOP_MS            2

uint8_t transmit_array[256];
uint8_t server_msg_array[256];


/***************************************************************************/
/* Transmit string */
/***************************************************************************/
bool transmit_string(uint8_t *str)
{
  bool success = false;
  uint16_t size;
  uint16_t timeout_counter = 0;
  for(size = 0; str[size]!='\000'; size++){}

  if(HAL_UART_Transmit_DMA(&huart3, str, size)!= HAL_OK)
  {
    Error_Handler();
  }

  while((ublox_trasnmit_complete != APP_TRUE) && (timeout_counter < TRANSMIT_TIMEOUT_MS))
  {
    vTaskDelay(TRANSMIT_LOOP_MS / portTICK_RATE_MS);
    timeout_counter += TRANSMIT_TIMEOUT_MS;
  }

  if(ublox_trasnmit_complete == APP_TRUE)
  {
    success = true;
  }
  ublox_trasnmit_complete = APP_FALSE;
  return success;
}

/***************************************************************************/
/* Transmit array */
/***************************************************************************/
bool transmit_array_bytes(uint8_t *str, uint8_t array_size)
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
/* Build at string */
/***************************************************************************/
void build_at_string(uint8_t *array, uint8_t *command, ...)
{
  va_list ap;
  uint8_t *ptr;
  va_start(ap, command);

  /* Copy msg content into array */
  ptr = command;

  /* Copy start of msg into array */
  strcpy(array, "");

  while(ptr != NULL)
  {
    /* Copy data into array */
    strcat(array, ptr);
    ptr = va_arg(ap, uint8_t*);
  }
  va_end(ap);
}

