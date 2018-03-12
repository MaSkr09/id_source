/****************************************************************************
* Copyright (c) 2018, Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
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
* File: id_protocol_v1.c
* Purpose: Server msg read and build msgs 
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2018-03-12 Martin Skriver, Source written
****************************************************************************/

/***************************************************************************/
/* Includes */
/***************************************************************************/
#include "id_protocol_v1.h"
#include "id_config.h"
#include "main.h"
#include "global_defines.h"
#include "crc.h"
#include "slip.h"

/***************************************************************************/
/* Defines */
/***************************************************************************/
#define degs_to_semicircles (double)  11930464.7111

/***************************************************************************/
/* Functions */
/***************************************************************************/

/***************************************************************************/
/* Build msg containing gsv data */
/***************************************************************************/
bool build_nw_loc_msg(uint16_t *lenght, uint8_t *data_msg)
{
  bool success = false;
  uint8_t data_lenght = 0;
  uint32_t msg_buffer;
  uint8_t msg_size = 0, msg_id_var;
  crc checksum;

  msg_id_var = (uint8_t)(UDP_GSM_AREA_MSG_TYPE << 4);
  if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    msg_id_var |= (0x0F & (server_msg_count_id++ ));
    data_msg[msg_size++] = msg_id_var;
    xSemaphoreGive(xSemaphoreServerFlightMsgs);
    data_msg[msg_size++] = (uint8_t)(DRONE_ID_NUMBER >> 16);
    data_msg[msg_size++] = (uint8_t)(DRONE_ID_NUMBER >> 8);
    data_msg[msg_size++] = (uint8_t)DRONE_ID_NUMBER;

    if( xSemaphoreTake( xSemaphoreCellId, ( TickType_t ) TIME_500_MS ) == pdTRUE )
    {
      data_msg[msg_size++] = (uint8_t)(gsm_signal_and_signal.mob_country_code >> 8);
      data_msg[msg_size++] = (uint8_t)gsm_signal_and_signal.mob_country_code;
      data_msg[msg_size++] = (uint8_t)(gsm_signal_and_signal.mob_network_code >> 8);
      data_msg[msg_size++] = (uint8_t)gsm_signal_and_signal.mob_network_code;
      data_msg[msg_size++] = (uint8_t)(gsm_signal_and_signal.mob_location_area_code >> 8);
      data_msg[msg_size++] = (uint8_t)gsm_signal_and_signal.mob_location_area_code;
      data_msg[msg_size++] = (uint8_t)(gsm_signal_and_signal.mob_cell_id_code >> 24);
      data_msg[msg_size++] = (uint8_t)(gsm_signal_and_signal.mob_cell_id_code >> 16);
      data_msg[msg_size++] = (uint8_t)(gsm_signal_and_signal.mob_cell_id_code >> 8);
      data_msg[msg_size++] = (uint8_t)gsm_signal_and_signal.mob_cell_id_code;

      xSemaphoreGive(xSemaphoreCellId);

      crcInit();
      checksum = crcFast(data_msg, msg_size);
      data_msg[msg_size++] = (uint8_t)(checksum >> 8);
      data_msg[msg_size++] = (uint8_t)checksum;

      *lenght = slip_generate_package(data_msg, msg_size);
      success = true;
    }
  }
  return success;
}
/***************************************************************************/
/* Build msg containing gsv data */
/***************************************************************************/
bool build_gsv_msg(uint16_t *lenght, uint8_t *data_msg, gpgsv_t *gsv_data, uint8_t type)
{
  bool success = false;
  uint32_t msg_buffer;
  uint8_t i,j, msg_size = 0;
  crc checksum;

  data_msg[msg_size++] = (uint8_t)type;
  data_msg[msg_size++] = (uint8_t)(DRONE_ID_NUMBER >> 16);
  data_msg[msg_size++] = (uint8_t)(DRONE_ID_NUMBER >> 8);
  data_msg[msg_size++] = (uint8_t)DRONE_ID_NUMBER; 

  if( xSemaphoreTake( xSemaphoreGsvMsg, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    if(gsv_data[0].sats_in_view)
    {
      data_msg[msg_size++] = (uint8_t)gsv_data[0].sats_in_view; 
      for(i=0; i<4; i++)
      {
        for(j=0; j<4; j++)
        {
          if(gsv_data[i].sat[j].prn != 0xFF)
          {
            data_msg[msg_size++] = (uint8_t)gsv_data[i].sat[j].prn;
            data_msg[msg_size++] = (uint8_t)gsv_data[i].sat[j].elevation;
            data_msg[msg_size++] = (uint8_t)(gsv_data[i].sat[j].azimuth >> 8);
            data_msg[msg_size++] = (uint8_t)gsv_data[i].sat[j].azimuth; 
            data_msg[msg_size++] = (uint8_t)gsv_data[i].sat[j].snr;
          }
        }
      }
      xSemaphoreGive( xSemaphoreGsvMsg );

      checksum = crcFast(data_msg, msg_size);
      data_msg[msg_size++] = (uint8_t)(checksum >> 8);
      data_msg[msg_size++] = (uint8_t)checksum;
      success = true;

      if(success)
        *lenght = slip_generate_package(data_msg, msg_size);
    }
    else
    {
      xSemaphoreGive( xSemaphoreGsvMsg );
    }
  }
  return success;
}
/***************************************************************************/
/* Build udp tracking msg string */
/***************************************************************************/
bool build_tracking_msg(uint16_t *lenght, uint8_t *data_msg)
{
  bool success = false;
  uint32_t msg_buffer;
  uint8_t msg_size = 0, msg_id_var;
  crc checksum;

  if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    msg_id_var = (uint8_t)(UDP_TRACK_MSG_TYPE << 4);
    msg_id_var |= (0x0F & (server_msg_count_id++ ));
    data_msg[msg_size++] = msg_id_var;
    xSemaphoreGive(xSemaphoreServerFlightMsgs);
    data_msg[msg_size++] = (uint8_t)(DRONE_ID_NUMBER >> 16);
    data_msg[msg_size++] = (uint8_t)(DRONE_ID_NUMBER >> 8);
    data_msg[msg_size++] = (uint8_t)DRONE_ID_NUMBER; 

    if( xSemaphoreTake( xSemaphoreGgaMsg, ( TickType_t ) TIME_500_MS ) == pdTRUE )
    {
      data_msg[msg_size++] = (uint8_t)(gga_data.time >> 8);
      data_msg[msg_size++] = (uint8_t)gga_data.time;
      data_msg[msg_size++] = (uint8_t)gga_data.sat;
      data_msg[msg_size++] = (uint8_t)(gga_data.hdop*10);
      data_msg[msg_size++] = (uint8_t)0;// gnss SNR
      msg_buffer = (uint32_t)((double)(gga_data.lat * degs_to_semicircles));
      data_msg[msg_size++] = (uint8_t)(msg_buffer >> 24);
      data_msg[msg_size++] = (uint8_t)(msg_buffer >> 16);
      data_msg[msg_size++] = (uint8_t)(msg_buffer >> 8);
      data_msg[msg_size++] = (uint8_t)msg_buffer;
      msg_buffer = (uint32_t)((double)(gga_data.lon * degs_to_semicircles));
      data_msg[msg_size++] = (uint8_t)(msg_buffer >> 24);
      data_msg[msg_size++] = (uint8_t)(msg_buffer >> 16);
      data_msg[msg_size++] = (uint8_t)(msg_buffer >> 8);
      data_msg[msg_size++] = (uint8_t)msg_buffer;
      msg_buffer = (uint32_t)((gga_data.alt + 1000)*2);
      xSemaphoreGive( xSemaphoreGgaMsg );
      data_msg[msg_size++] = (uint8_t)(msg_buffer >> 8);
      data_msg[msg_size++] = (uint8_t)msg_buffer;

      data_msg[msg_size++] = (uint8_t)0; // ATTI_R
      data_msg[msg_size++] = (uint8_t)0; // ATTI_P
      data_msg[msg_size++] = (uint8_t)0; // ATTI_Y

      if( xSemaphoreTake( xSemaphoreBattVoltage, ( TickType_t ) TIME_500_MS ) == pdTRUE )
      {
        msg_buffer = ((batteryVoltage-2.9)*255/1.4);
        xSemaphoreGive( xSemaphoreBattVoltage );
        data_msg[msg_size++] = msg_buffer;

        if( xSemaphoreTake( xSemaphoreCellId, ( TickType_t ) TIME_500_MS ) == pdTRUE )
        {
          data_msg[msg_size++] = gsm_signal_and_signal.SIGNAL_Q_VAR;
          xSemaphoreGive( xSemaphoreCellId );

          checksum = crcFast(data_msg, msg_size);
          data_msg[msg_size++] = (uint8_t)(checksum >> 8);
          data_msg[msg_size++] = (uint8_t)checksum;
          success = true;
        }
      }
    }
  }
  if(success)
    *lenght = slip_generate_package(data_msg, msg_size);

  return success;
}

/***************************************************************************/
/* Build stop msg */
/***************************************************************************/
bool build_stop_msg(uint16_t *lenght, uint8_t *data_msg, uint8_t stop_condition)
{
  bool success = false;
  uint8_t msg_size = 0, msg_id_var;
  crc checksum;

  msg_id_var = (uint8_t)(UDP_STOP_MSG_TYPE << 4);
  if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    msg_id_var |= (0x0F & (server_msg_count_id++ ));
    xSemaphoreGive(xSemaphoreServerFlightMsgs);
    data_msg[msg_size++] = msg_id_var;
    data_msg[msg_size++] = (uint8_t)(DRONE_ID_NUMBER >> 16);
    data_msg[msg_size++] = (uint8_t)(DRONE_ID_NUMBER >> 8);
    data_msg[msg_size++] = (uint8_t)DRONE_ID_NUMBER;
    data_msg[msg_size++] = stop_condition; // stop code

    crcInit();
    checksum = crcFast(data_msg, msg_size);
    data_msg[msg_size++] = (uint8_t)(checksum >> 8);
    data_msg[msg_size++] = (uint8_t)checksum;

    *lenght = slip_generate_package(data_msg, msg_size);
    success = true;
  }
  return success;
}
/***************************************************************************/
/* Build start msg */
/***************************************************************************/
bool build_start_msg(uint16_t *lenght, uint8_t *data_msg)
{
  bool success = false;
  uint32_t msg_buffer;
  uint8_t msg_size = 0, msg_id_var;
  crc checksum;

    if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
    {
      msg_id_var = (uint8_t)(UDP_START_MSG_TYPE << 4);
      msg_id_var |= (0x0F & (server_msg_count_id++ ));
      data_msg[msg_size++] = msg_id_var;
      xSemaphoreGive(xSemaphoreServerFlightMsgs);

      data_msg[msg_size++] = (uint8_t)PROTOCOL_VERSION;
      data_msg[msg_size++] = (uint8_t)(DRONE_ID_NUMBER >> 16);
      data_msg[msg_size++] = (uint8_t)(DRONE_ID_NUMBER >> 8);
      data_msg[msg_size++] = (uint8_t)DRONE_ID_NUMBER;

      data_msg[msg_size++] = (uint8_t) (FIRMWARE_VERSION_NO*10);
      data_msg[msg_size++] = (uint8_t) (HW_VERSION_NO*10);

      if( xSemaphoreTake( xSemaphoreBattVoltage, ( TickType_t ) TIME_500_MS ) == pdTRUE )
      {
        msg_buffer = ((initialBatteryVoltage-2.9)*255/1.4);
        xSemaphoreGive( xSemaphoreBattVoltage );
        data_msg[msg_size++] = msg_buffer;
  
        crcInit();
        checksum = crcFast(data_msg, msg_size);
        data_msg[msg_size++] = (uint8_t)(checksum >> 8);
        data_msg[msg_size++] = (uint8_t)checksum;

        *lenght = slip_generate_package(data_msg, msg_size);
        success = true;
      }
    }
  return success;
}

/***************************************************************************/
/* Read start msg from server */
/***************************************************************************/
bool read_start_msg(uint8_t *str)
{
  bool success = false;
  if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    echo_start_msg_count = (0x0F &(*str));
    xSemaphoreGive(xSemaphoreServerFlightMsgs);
    success = true;
  }
  return success;
}

/***************************************************************************/
/* Read stop msg from server */
/***************************************************************************/
bool read_stop_msg(uint8_t *str)
{
  bool success = false;
  if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    echo_stop_msg_count = (0x0F &(*str));
    xSemaphoreGive(xSemaphoreServerFlightMsgs);
    success = true;
  }
  return success;
}

/***************************************************************************/
/* Read tracking msg from server */
/***************************************************************************/
bool read_tracking_msg(uint8_t *str)
{
  bool success = false;
  if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    echo_msg_count_cell_id = (0x0F &(*str));
    new_server_msg_time_interval = *(str+1);
    max_horizontal_server_msg = *(str+2);
    max_vertical_server_msg = *(str+3);
    flight_permission = *(str+4);
    serv_cmd = *(str+5);
    xSemaphoreGive(xSemaphoreServerFlightMsgs);
    success = true;
  }
  return success;
}

/***************************************************************************/
/* Read cell tracking msg from server */
/***************************************************************************/
bool read_cell_tracking_msg(uint8_t *str)
{
  bool success = false;
  if( xSemaphoreTake( xSemaphoreCellId, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    gsm_signal_and_signal.received_resp_cell_id = true;
    xSemaphoreGive(xSemaphoreCellId);
    success = true;
  }
  echo_cell_info_msg_count = (0x0F &(*str));
  return success;
}

/***************************************************************************/
/* Read header msg from server */
/***************************************************************************/
bool read_server_msg(uint8_t *str)
{
  bool success = false;
  if((*str>>4) == UDP_START_MSG_TYPE)
  {
    success = read_start_msg(str);
  }
  else if((*str>>4) == UDP_STOP_MSG_TYPE)
  {
    success = read_stop_msg(str);
  }
  else if((*str>>4) == UDP_TRACK_MSG_TYPE)
  {
    success = read_tracking_msg(str);
  }
  else if((*str>>4) == UDP_GSM_AREA_MSG_TYPE)
  {
    success = read_cell_tracking_msg(str);
  }
  return success;
}