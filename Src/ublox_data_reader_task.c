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
* File: ublox_data_reader_task.c
* Purpose: Read data from Ublox SARA G3 module
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-03-29 Martin Skriver, Source written
* Created:  2018-03-09 Martin Skriver, Code cleanup
****************************************************************************/
#ifdef DEBUG
#include "debug_task.h"
#endif
#include "main.h"
#include "usart.h"
#include "dynamic_array.h"
#include "ublox_reg.h"
#include "ublox_data_reader_task.h"
#include "server_settings.h"
#include "id_config.h"
#include "global_defines.h"

#include <string.h>

#define DATA_READER_TASK_DELAY_MS     200
#define UART_RECEIVE_LOOP_MS          TIME_5_MS
#define MAXIMUM_RECEIVE_TIME_MS       200
uint8_t receive_array[512];

/***************************************************************************/
/* Read line from queue */
/***************************************************************************/
bool receive_data_line(uint8_t *array, uint16_t timeout)
{
  bool received_line = false, data = true;
  uint16_t cnt=0, timer_var = 0;
  
  *array = 0;
  while(((*array == '\n') || (*array == '\r') || (*array == 0)) && (timer_var < timeout))
  {
    data = xQueueReceive( xQueueUbloxReceive, array, ( TickType_t ) TIME_5000_MS );
    if(!data)
    {
      timer_var += UART_RECEIVE_LOOP_MS;
    }
  }

  if((*array == '@') && data)
  {
    *(array+1) = '\n';
    *(array+2) = 0;
    received_line = true;
  }
  else if(data && ((*array != '\n') && (*array != '\r') && (*array != 0)))
  {
    timer_var = 0;
    while((MAXIMUM_RECEIVE_TIME_MS > timer_var) && !received_line)
    {
      if(xQueueReceive( xQueueUbloxReceive, (array+cnt+1), ( TickType_t ) UART_RECEIVE_LOOP_MS ))
      {
        cnt++;
        *(array+cnt+1) = 0;
        if(strncmp((array+1), AT_READ_MSG_FROM_SERVER, sizeof(AT_READ_MSG_FROM_SERVER)-1)==0)
        {
          received_line = true;
        }
        else if(*(array+cnt) == '\r')
        {
          *(array+cnt) = '\n';
          received_line = true;
        }
        else if(*(array+cnt) == '\n')
        {
          cnt--;        
        }
      }
      else
      {
        timer_var += UART_RECEIVE_LOOP_MS;
      }
    }
  }
#ifdef DEBUG
  if(received_line)
  debug_add_ascii_to_queue(array);
#endif
  return received_line;
}

/***************************************************************************/
/* Read msg from server */
/***************************************************************************/
bool read_data_from_server(uint8_t *str)
{
  bool success = false;
  if((*str>>4) == UDP_START_MSG_TYPE)
  {
    if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
    {
      echo_start_msg_count = (0x0F &(*str));
      xSemaphoreGive(xSemaphoreServerFlightMsgs);
      success = true;
    }
  }
  else if((*str>>4) == UDP_STOP_MSG_TYPE)
  {
    if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
    {
      echo_stop_msg_count = (0x0F &(*str));
      xSemaphoreGive(xSemaphoreServerFlightMsgs);
      success = true;
    }
  }
  else if((*str>>4) == UDP_TRACK_MSG_TYPE)
  {
    if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
    {
      echo_msg_count_cell_id = (0x0F &(*str));
      new_server_msg_time_interval = *(str+1);
      max_horizontal_server_msg = *(str+2);
      max_vertical_server_msg = *(str+3);
      flight_permission = *(str+4);
//      serv_cmd = 0xf0;
      serv_cmd = *(str+5);
      xSemaphoreGive(xSemaphoreServerFlightMsgs);
      success = true;
    }
  }
  else if((*str>>4) == UDP_GSM_AREA_MSG_TYPE)
  {
    if( xSemaphoreTake( xSemaphoreCellId, ( TickType_t ) TIME_500_MS ) == pdTRUE )
    {
      gsm_signal_and_signal.received_resp_cell_id = true;
      xSemaphoreGive(xSemaphoreCellId);
    }
    echo_cell_info_msg_count = (0x0F &(*str));
    success = true;
  }

  return success;
}

/***************************************************************************/
/* Handle receive msg from server */
/***************************************************************************/
bool receive_msg_from_server(uint8_t *str)
{
  bool received_line = false;
  bool data = true, receive_server_data = false, ignore_msg_read = false;
  uint16_t cnt, timer_var = 0, msg_offset = 0, msg_size = 0;
  uint16_t str_header_size;

  for(cnt = 0; *(str+cnt)!= NULL; cnt++){}

  if(ublox_status_reg.UDP_READING_NO_OF_BYTES == 0)
    ignore_msg_read = true;

  if( xSemaphoreTake( xSemaphoreSocket, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    str_header_size = sizeof(AT_READ_MSG_FROM_SERVER)+1+sizeof(socket_data.SERVER_IP)+sizeof(DRONEID_SERVER_UDP_PORT) + 2;
    xSemaphoreGive(xSemaphoreSocket);
  }

  while((!received_line) && timer_var < MAXIMUM_RECEIVE_TIME_MS )
  {
    if(xQueueReceive( xQueueUbloxReceive, (str+cnt), ( TickType_t ) UART_RECEIVE_LOOP_MS ))
    {
      // Msg read containing msg in modem
      if(ignore_msg_read)
      {
        if(*(str+cnt) == '\r')
        {
          *(str+cnt) = '\n';
          *(str+cnt+1) = 0;
          received_line = true;
          if(cnt>=10)
            ublox_status_reg.UDP_NO_OF_BYTES_TO_READ = atoi(str+10);
        }
        else if(*(str+cnt) == '\n')
        {
          cnt--;        
        }
      }
      else
      {
        if((cnt >= str_header_size) && (*(str+cnt) == ','))
        {
          msg_offset = cnt;
          msg_size = atoi(str+str_header_size);
        }
        if((cnt >= msg_size + str_header_size+3) && (msg_offset != 0) && (*(str+cnt) == '\r'))
        {
          *(str+cnt) = '\n';
          *(str+cnt+1) = 0;
          received_line = true;
          ublox_status_reg.UDP_READING_NO_OF_BYTES = 0;
//          if(0 <= (int16_t)ublox_status_reg.UDP_NO_OF_BYTES_TO_READ - (int16_t)msg_size)
//            ublox_status_reg.UDP_NO_OF_BYTES_TO_READ = ublox_status_reg.UDP_NO_OF_BYTES_TO_READ - msg_size;
//          else
            ublox_status_reg.UDP_NO_OF_BYTES_TO_READ = 0;
          read_data_from_server(str+msg_offset+2);
        }
      }
      cnt++;
    }
    else
    {
      timer_var += UART_RECEIVE_LOOP_MS;
    }
  }
  return received_line;
}

/***************************************************************************/
/* Get received result code and write result to register */
/***************************************************************************/
bool get_result_code(uint8_t *str, ublox_result_code_t * result, SemaphoreHandle_t *mutex)
{
  bool result_code_received = false;
  if(xSemaphoreTake( mutex, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    if(strncmp(str, AT_OK_RESULT_CODE, sizeof(AT_OK_RESULT_CODE)-1)==0)
    {
      *result = OK_RESULT_CODE;
      result_code_received = true;
    }
    else if(strncmp(str, AT_ERROR_RESULT_CODE, sizeof(AT_ERROR_RESULT_CODE)-1)==0)
    {
      *result = ERROR_RESULT_CODE;
      result_code_received = true;
    }
    xSemaphoreGive(mutex);
  }
  return result_code_received;
}

/***************************************************************************/
/* Receive and check response code */
/***************************************************************************/
void get_and_check_response(uint8_t *str, uint16_t max_response_dalay, ublox_result_code_t * result, SemaphoreHandle_t *mutex)
{
  if(receive_data_line(str, max_response_dalay))
    get_result_code(str, result, mutex);
}

/***************************************************************************/
/* Created socket */
/***************************************************************************/
void created_socket(uint8_t *str)
{
  uint8_t offset;
  for(offset = 0; *(str+offset)!= NULL; offset++){}
  if((*(str+(offset-2)) >= '0') && (*(str+(offset-2)) < '7'))
  {
    if( xSemaphoreTake( xSemaphoreSocket, ( TickType_t ) TIME_200_MS ) == pdTRUE )
    {
      socket_data.CREATED_SOCKET_NO = atoi(str+(offset-2));
      xSemaphoreGive( xSemaphoreSocket );
    }
  }
  /* Get result code */
  if(receive_data_line(str, TIME_500_MS))
    get_result_code(str, &ublox_status_reg.CREATED_SOCKET_NO_RESULT_CODE, xSemaphoreUbloxStatusReg);
}

/***************************************************************************/
/* Resolve IP msg */
/***************************************************************************/
void get_server_ip_resp(uint8_t *str)
{
  uint8_t offset;

  for(offset = 0; *(str+offset)!= NULL; offset++){}

  if( xSemaphoreTake( xSemaphoreSocket, ( TickType_t ) TIME_200_MS ) == pdTRUE )
  {
    strncpy(socket_data.SERVER_IP, str+9, offset-10);
    xSemaphoreGive( xSemaphoreSocket );
  }
  if(receive_data_line(str, TIME_500_MS))
    get_result_code(str, &ublox_status_reg.SERVER_IP_RESULT_CODE, xSemaphoreUbloxStatusReg);
}

/***************************************************************************/
/* Received GSM signal quality */
/***************************************************************************/
void signal_q_resp(uint8_t *str)
{
  if( xSemaphoreTake( xSemaphoreCellId, ( TickType_t ) TIME_200_MS ) == pdTRUE )
  {
    gsm_signal_and_signal.SIGNAL_Q_VAR = atoi(str+5);
    xSemaphoreGive( xSemaphoreCellId );
  }
}

/***************************************************************************/
/* Received ready to receive data to transmit */
/***************************************************************************/
void ready_to_transmit(void)
{
  if( xSemaphoreTake( xSemaphoreUbloxStatusReg, ( TickType_t ) TIME_200_MS ) == pdTRUE )
  {
    ublox_status_reg.MODEM_READY_FOR_DATA = OK_RESULT_CODE;
    xSemaphoreGive( xSemaphoreUbloxStatusReg );
  }
}

/***************************************************************************/
/* Received GGA msg */
/***************************************************************************/
void received_gga_msg(uint8_t *str)
{
  if(!nmea_checksum((str+11)))
  {
    if( xSemaphoreTake( xSemaphoreGgaMsg, ( TickType_t ) TIME_200_MS ) == pdTRUE )
    {
      nmea_gpgga_parse((str+11), &gga_data);
      xSemaphoreGive( xSemaphoreGgaMsg );
    }
  }
}

/***************************************************************************/
/* Received GPGSV msg */
/***************************************************************************/
void received_gpgsv_msg(uint8_t *str)
{
  uint8_t offset = 0;

  if(strncmp(str+10, AT_GPGSV_START, sizeof(AT_GPGSV_START)-1)==0)
    str=str+10;
  if(!nmea_checksum((str+1)))
  {
    offset = atoi(str+9);
    if((offset>0) && (offset<=3))
    {
      if( xSemaphoreTake( xSemaphoreGsvMsg, ( TickType_t ) TIME_200_MS ) == pdTRUE )
      {
        nmea_gpgsv_parse(str+1, &gpgsv_data[offset-1]);
        xSemaphoreGive( xSemaphoreGsvMsg );
      }
    }
  }
}

/***************************************************************************/
/* Received GLGSV msg */
/***************************************************************************/
void received_glgsv_msg(uint8_t *str)
{
  uint8_t offset = 0;

  if(!nmea_checksum((str+1)))
  {
    offset = atoi(str+9);
    if((offset>0) && (offset<=3))
    {
      if( xSemaphoreTake( xSemaphoreGsvMsg, ( TickType_t ) TIME_500_MS ) == pdTRUE )
      {
        nmea_gpgsv_parse(str+1, &glgsv_data[offset-1]);
        xSemaphoreGive( xSemaphoreGsvMsg );
      }
    }
  }
}

/***************************************************************************/
/* Received cell location and cell id */
/***************************************************************************/
void read_gsm_loc_and_cell_id(uint8_t *str)
{
  if( xSemaphoreTake( xSemaphoreCellId, ( TickType_t ) TIME_200_MS ) == pdTRUE )
  {
    gsm_signal_and_signal.mob_location_area_code = (uint16_t)strtoul(str+13, NULL, 16);
    gsm_signal_and_signal.mob_cell_id_code = (uint32_t)strtoul(str+20, NULL, 16);
    xSemaphoreGive( xSemaphoreCellId );
  }
}

/***************************************************************************/
/* Received cell location and cell id */
/***************************************************************************/
void received_gsm_country(uint8_t *str)
{
  uint16_t code = atoi(str+11);
  if( xSemaphoreTake( xSemaphoreCellId, ( TickType_t ) TIME_200_MS ) == pdTRUE )
  {
    gsm_signal_and_signal.mob_country_code = code/100;
    gsm_signal_and_signal.mob_network_code = code%100;
    xSemaphoreGive( xSemaphoreCellId );
  }
}

/***************************************************************************/
/* Get received type */
/***************************************************************************/
void get_type(uint8_t *str)
{
  if(strncmp(str, AT_CPIN_READY_CODE, sizeof(AT_CPIN_READY_CODE)-1)==0)
  {
    get_and_check_response(str, TIME_500_MS, &ublox_status_reg.CPIN_RESULT_CODE, xSemaphoreUbloxStatusReg);
  }
  else if((strncmp(str, AT_DISABLE_ECHO, sizeof(AT_DISABLE_ECHO)-1)==0)||(strncmp(str, DISABLE_ECHO, sizeof(DISABLE_ECHO))==0))
  {
    get_and_check_response(str, TIME_500_MS, &ublox_status_reg.DIS_ECHO_RESULT_CODE, xSemaphoreUbloxStatusReg);
  }
  else if(strncmp(str, CGATT_SUCCES, sizeof(CGATT_SUCCES)-1)==0)
  {
    get_and_check_response(str, TIME_500_MS, &ublox_status_reg.CGATT_RESULT_CODE, xSemaphoreUbloxStatusReg);
  }
  else if(get_result_code(str, &ublox_status_reg.GP_RESULT_CODE, xSemaphoreUbloxStatusReg )){}
  else if(strncmp(str+1, AT_CREATE_SOCKET, sizeof(AT_CREATE_SOCKET)-1)==0)
  {
    created_socket(str);
  }
  else if(strncmp(str+1, AT_RESOLVE_SERVER_IP, sizeof(AT_RESOLVE_SERVER_IP)-1)==0)
  {
    get_server_ip_resp(str);
  }
  else if(strncmp(str+1, AT_QUALITY, sizeof(AT_QUALITY)-1)==0)
  {
    signal_q_resp(str);
  }
  else if(*str == '@')
  {
    ready_to_transmit();
  }
  else if(strncmp(str+1, AT_GET_UGGGA, sizeof(AT_GET_UGGGA)-1)==0)
  {
    received_gga_msg(str);
  }
  else if((strncmp(str+10, AT_GPGSV_START, sizeof(AT_GPGSV_START)-1)==0) || (strncmp(str, AT_GPGSV_START, sizeof(AT_GPGSV_START)-1)==0))
  {
    received_gpgsv_msg(str);
  }
  else if(strncmp(str, AT_GLGSV_START, sizeof(AT_GLGSV_START)-1)==0)
  {
    received_glgsv_msg(str);
  }
  else if((strncmp(str, AT_UNREAD_BYTES, sizeof(AT_UNREAD_BYTES)-1)==0) || (strncmp(str, AT_UNREAD_BYTES_D, sizeof(AT_UNREAD_BYTES_D)-1)==0))
  {
    ublox_status_reg.UDP_NO_OF_BYTES_TO_READ = atoi(str+11);
  }
  else if(strncmp(str+1, AT_READ_MSG_FROM_SERVER, sizeof(AT_READ_MSG_FROM_SERVER))==0)
  {
    receive_msg_from_server(str);
  }
  else if(strncmp(str, AT_GPRS_NETWORK_REG_LOC, sizeof(AT_GPRS_NETWORK_REG_LOC)-1)==0)
  {
    read_gsm_loc_and_cell_id(str);
  }
  else if(strncmp(str, AT_DISP_OPR, sizeof(AT_DISP_OPR)-1)==0)
  {
    received_gsm_country(str);
  }
}

/***************************************************************************/
/* Ublox receive main task */
/***************************************************************************/
void ublox_data_reader_main(void *pvParameters)
{
#ifdef DEBUG
  debug_add_ascii_to_queue("Ublox data reader task: Started\n");
#endif
  vTaskDelay(DATA_READER_TASK_DELAY_MS / portTICK_RATE_MS);

  TASK_LOOP
  {
    if(receive_data_line(receive_array, TIME_200_MS))
    {
      get_type(receive_array);
    }
  }
}
