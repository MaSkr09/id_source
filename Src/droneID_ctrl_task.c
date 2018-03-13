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
* File: droneID_ctrl_task.c
* Purpose: Setup droneID and controls when to send and method
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-02-15 Martin Skriver, Source written
****************************************************************************/
#include <stdbool.h>
#ifdef DEBUG
#include "debug_task.h"
#endif
#include "pwr_management_task.h"
#include "main.h"
#include "ublox_gnss_gsm_driver.h"
#include "uart_transmit.h"
#include "droneID_ctrl_task.h"
#include "id_config.h"
/***************************************************************************/
/* Defines */
/***************************************************************************/
#define SOCKET_CREATE_MAX_ATTEMPT   3
#define DELAY_5_MS                  5
#define DELAY_20_MS                 20
#define DELAY_30_MS                 30
#define DELAY_50_MS                 50
#define DELAY_100_MS                100
#define DELAY_500_MS                500
#define DELAY_1000_MS               1000

#define UDP_TRAKING_LOOP_DELAY_MS   20
#define MAX_TRACKING_UDP_ERRORS     3

#define MAX_GPRS_CONNECT            3
#define MAX_GNSS_CONNECT            3

#define NO_SERVER_CONNECTION        0
#define SMS_SERVER_CONNECTION       1
#define SOCKET_SERVER_CONNECTION    2

#define MAX_MSG_READ                3
#define MSG_COUNT_DIF_WARN          4
#define MAX_RESP_DIFF_ERROR         10

#define MAX_SEND_ERROR              2
#define MAX_READ_GGA                2
#define MAX_READ_GSV                2
#define MAX_SEND_GSM_LOC            2
#define MAX_READ_GSM_LOC            2
#define MAX_READ_RSSI               2

#define TIME_5000_MS                5000
#define RESPONSE_LOOP_TIME          100

/***************************************************************************/
/* Shared variables */
/***************************************************************************/
bool gprs_mode_flag = false;
uint8_t msg_mode = NO_SERVER_CONNECTION;
bool gnss_pwr_on_flag = false;
uint8_t error_code;

gpgga_t pre_gga_msg;


/***************************************************************************/
/* Indication mode */
/***************************************************************************/
void set_indication_start_mode(bool start_mode)
{

  if(xSemaphoreTake( xSemaphoreTaskIndicators, ( TickType_t ) DELAY_500_MS) == pdTRUE)
  {
    if(start_mode)
      ctrl_power_on = true;
    else
      ctrl_power_on = false;
    xSemaphoreGive( xSemaphoreTaskIndicators );
  }
}

/***************************************************************************/
/* Reset var */
/***************************************************************************/
bool reset_utm_var()
{
  bool success = false;
  if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) DELAY_50_MS ) == pdTRUE )
  {
    server_msg_count_id = 0;
    server_msg_time_interval = 1;
    new_server_msg_time_interval = 1;
    max_horizontal_server_msg = 50;
    max_vertical_server_msg = 20;
    flight_permission = SERVER_NOT_CONNECTED;
    serv_cmd = 0;
    xSemaphoreGive(xSemaphoreServerFlightMsgs);

    if( xSemaphoreTake( xSemaphoreCellId, ( TickType_t ) DELAY_50_MS ) == pdTRUE )
    {
      gsm_signal_and_signal.received_resp_cell_id = false;
      echo_msg_count_cell_id = 0;
      send_msg_count_cell_id = 5;
      gsm_signal_and_signal.mob_country_code = 0;
      gsm_signal_and_signal.mob_network_code = 0;
      gsm_signal_and_signal.mob_location_area_code = 0;
      gsm_signal_and_signal.mob_cell_id_code = 0;
      gsm_signal_and_signal.pre_mob_location_area_code = 0;
      gsm_signal_and_signal.pre_mob_cell_id_code = 0;
      xSemaphoreGive(xSemaphoreCellId);
      success = true;
    }
  }

  set_indication_start_mode(true);
  
  error_code = ERROR_CODE_USER_SHUT_DOWN;
  return success;
}

/***************************************************************************/
/* Power down gnss and gsm */
/***************************************************************************/
bool power_down_gnss_gsm(void)
{
  bool success = false;

  if(gnss_pwr_on_flag)
  {
    if(power_off_gnss())
      success = true;
  }
  else
      success = true;

  vTaskDelay(DELAY_20_MS / portTICK_RATE_MS);
  success &= power_down_gsm_modem();

  return success;
}

/***************************************************************************/
/* Build gsm tracking msg string */
/***************************************************************************/
void change_timer_period(uint32_t timer_period)
{
//  if( xTimerChangePeriod( xHandleVoltAdcGPTimer, timer_period / portTICK_PERIOD_MS, 1000 ) == pdPASS )
//  {
////    xTimerStart( xHandleCtrlGPTimer, 1000 );
//  }
}

/***************************************************************************/
/* Error reset modems */
/***************************************************************************/
void error_reset_mcu(void)
{
  if(power_mode_is_on())
    NVIC_SystemReset();
}

/***************************************************************************/
/* Error reset modems */
/***************************************************************************/
bool error_reset_modems(void)
{
  bool success = false;

  if(reset_gsm_modem())
  {
    success = true;
    gprs_mode_flag = false;
    gnss_pwr_on_flag = false;
  }
  else
  {
    error_reset_mcu();
  }

  return success;
}

/***************************************************************************/
/* Update rssi */
/***************************************************************************/
bool update_rssi_values(void)
{
  bool success = false;

  vTaskDelay(DELAY_20_MS / portTICK_RATE_MS);
  if(update_rssi())
  {
    success = true;
  }
  return success;
}

/***************************************************************************/
/* Update rssi */
/***************************************************************************/
bool update_gga_data(void)
{
  bool success = false;

  vTaskDelay(DELAY_20_MS / portTICK_RATE_MS);
  if(get_gnss_gga())
  {
    success = true;
  }
  return success;
}

/***************************************************************************/
/* Check and read if modem contains msg from server */
/***************************************************************************/
bool read_udp_server_msg(void)
{
  bool success = false;
  uint16_t bytes = 1;
  uint8_t i=0;

    if( xSemaphoreTake( xSemaphoreUbloxStatusReg, ( TickType_t ) DELAY_5_MS ) == pdTRUE )
    {
      bytes = ublox_status_reg.UDP_NO_OF_BYTES_TO_READ;
      xSemaphoreGive(xSemaphoreUbloxStatusReg);
    }
    if(bytes != 0)
    {
      vTaskDelay(DELAY_20_MS / portTICK_RATE_MS);
      if(read_udp_no_of_msg_bytes())
        success = true;
    }
    // check counter value and update parameter if missing response from server
    else if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) DELAY_5_MS ) == pdTRUE )
    {
      server_msg_count_id &= 0x0F;
      if(((server_msg_count_id >= echo_msg_count_cell_id) && ((server_msg_count_id - echo_msg_count_cell_id) > MSG_COUNT_DIF_WARN))
      || ((server_msg_count_id < echo_msg_count_cell_id) && ((server_msg_count_id + 15 - echo_msg_count_cell_id) > MSG_COUNT_DIF_WARN)))
      {
        xSemaphoreGive( xSemaphoreServerFlightMsgs );
        if(read_udp_no_of_msg_bytes())
          success = true;
      }
      else
      {
        xSemaphoreGive( xSemaphoreServerFlightMsgs );
        success = true;
      }
    }
  return success;
}

/***************************************************************************/
/* Send gsm info */
/***************************************************************************/
bool gsm_info_update(void)
{
  bool success = false;

  vTaskDelay(DELAY_20_MS);
  if(send_gsm_nw_msg())
  {
    if( xSemaphoreTake( xSemaphoreCellId, ( TickType_t ) DELAY_5_MS ) == pdTRUE )
    {
      gsm_signal_and_signal.pre_mob_cell_id_code = gsm_signal_and_signal.mob_cell_id_code;
      gsm_signal_and_signal.pre_mob_location_area_code = gsm_signal_and_signal.mob_location_area_code;
      xSemaphoreGive( xSemaphoreCellId );
      success = true;
    }
  }
  return success;
}

/***************************************************************************/
/* Send gnss gsv data */
/***************************************************************************/
bool get_and_send_gsv(void)
{
  bool success = false;
  vTaskDelay(DELAY_20_MS / portTICK_RATE_MS);
  if(get_gnss_gsv())
  {
    if(send_udp_gpgsv())
    {
      if(send_udp_glgsv())
      {
        success = true;
      }
    }
  }
  return success;
}

/***************************************************************************/
/* GNSS fix */
/***************************************************************************/
bool gnss_fix(void)
{
  bool fix = false;

  if( xSemaphoreTake( xSemaphoreGgaMsg, ( TickType_t ) DELAY_500_MS ) == pdTRUE )
  {
    if(gga_data.sat > GNSS_MIN_SAT)
    {
      fix = true;
    }  
    xSemaphoreGive( xSemaphoreGgaMsg );
  }
  return fix;
}            

/***************************************************************************/
/* Change timer value */
/***************************************************************************/
void update_tracking_timer_value(void)
{
  if(gnss_fix())
  {
    if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) DELAY_20_MS ) == pdTRUE )
    {
      if(new_server_msg_time_interval != server_msg_time_interval)
      {
        server_msg_time_interval = new_server_msg_time_interval;
        xTimerChangePeriod(xHandleTransmitTimer, (1000*server_msg_time_interval), DELAY_20_MS);
        xTimerReset( xHandleTransmitTimer, DELAY_20_MS);
      }
      xSemaphoreGive(xSemaphoreServerFlightMsgs);
    }
  }
  else
  {
    if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) DELAY_20_MS ) == pdTRUE )
    {
      if(GMS_MSG_PERIOD_S != server_msg_time_interval)
      {
        server_msg_time_interval = GMS_MSG_PERIOD_S;
        xTimerChangePeriod(xHandleTransmitTimer, (1000*server_msg_time_interval), DELAY_20_MS);
        xTimerReset( xHandleTransmitTimer, DELAY_20_MS);
      }
      xSemaphoreGive( xSemaphoreServerFlightMsgs );
    }
  }
}

bool droneID_has_moved(void)
{
  bool return_value = false;
  if( xSemaphoreTake( xSemaphoreGgaMsg, ( TickType_t ) DELAY_500_MS ) == pdTRUE )
  {
    if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) DELAY_500_MS ) == pdTRUE )
    {
      if( (max_horizontal_server_msg <= degree_to_meter_conv(&gga_data, &pre_gga_msg)) || (max_vertical_server_msg <= abs(pre_gga_msg.alt - gga_data.alt)))
      {
        return_value = true;
      }
      xSemaphoreGive( xSemaphoreServerFlightMsgs );
    }
    xSemaphoreGive( xSemaphoreGgaMsg );
  }
  return return_value;
}

/***************************************************************************/
/* Copy current position */
/***************************************************************************/
void update_pre_pos()
{
  if( xSemaphoreTake( xSemaphoreGgaMsg, ( TickType_t ) DELAY_500_MS ) == pdTRUE )
  {
    pre_gga_msg.alt = gga_data.alt; 
    pre_gga_msg.lat = gga_data.lat; 
    pre_gga_msg.lon = gga_data.lon; 
    xSemaphoreGive( xSemaphoreGgaMsg );
  }
}

/***************************************************************************/
/* Check if server response to start msg */
/***************************************************************************/
bool start_response_from_server()
{
  //TODO change when Kjeld update server
  bool response_ok = true; //false;
  int16_t timeout = TIME_5000_MS;
  
  while((!(response_ok)) && (timeout > 0))
  {
    read_udp_server_msg();
    if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) DELAY_500_MS ) == pdTRUE )
    {
      if(echo_start_msg_count == (server_msg_count_id-1))
        response_ok = true;
      xSemaphoreGive( xSemaphoreServerFlightMsgs );
    }
    timeout -= RESPONSE_LOOP_TIME;
    vTaskDelay(RESPONSE_LOOP_TIME / portTICK_RATE_MS);
  }
  return response_ok;
}

/***************************************************************************/
/* Enter tracking mode */
/***************************************************************************/
void udp_tracking_mode(void)
{
  uint8_t i = 0, error_cnt_read_loc = 0, error_cnt_send_loc = 0, 
          error_cnt_read_gsv = 0, error_cnt_read_gga = 0, 
          error_counter_send_track = 0, error_cnt_get_rssi = 0,
          msg_count_cell_id, cell_info_msg_count;

  if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) DELAY_500_MS ) == pdTRUE )
  {
    msg_count_cell_id = server_msg_count_id;
    cell_info_msg_count = server_msg_count_id;
    xSemaphoreGive( xSemaphoreServerFlightMsgs );
  }

  if(start_response_from_server())
  {
    while(power_mode_is_on() 
          && (error_counter_send_track < MAX_SEND_ERROR) 
          && (error_cnt_read_gga < MAX_READ_GGA)
          && (error_cnt_read_gsv < MAX_READ_GSV)  
          && (error_cnt_send_loc < MAX_SEND_GSM_LOC)  
          && (error_cnt_read_loc < MAX_READ_GSM_LOC)
          && (error_cnt_get_rssi < MAX_READ_RSSI))
    {
      read_udp_server_msg();

      if(xSemaphoreTake( xSemaphoreGSMInfoTimerExpired, ( TickType_t ) 0) == pdTRUE)
      {
        error_cnt_read_loc++;
        if(read_netw_reg_loc())
        {
          error_cnt_read_loc = 0;
          error_cnt_send_loc++;
          if(gsm_info_update())
          {
            error_cnt_send_loc = 0;
          }
        }
      }

      if(xSemaphoreTake( xSemaphoreGetGgga, ( TickType_t ) 0) == pdTRUE)
      {
        error_cnt_read_gga++;
        if(update_gga_data())
        {
          error_cnt_read_gga = 0;
          if(gnss_fix())
          {
            if(droneID_has_moved())
            {
              xSemaphoreGive( xSemaphoreTransmitPos );
              xTimerReset( xHandleTransmitTimer, DELAY_20_MS);
              update_pre_pos();
            }
          }
        }
      }

      if(xSemaphoreTake( xSemaphoreTransmitPos, ( TickType_t ) UDP_TRAKING_LOOP_DELAY_MS) == pdTRUE)
      {
        error_cnt_get_rssi++;
        if(update_rssi_values())
        {
          error_cnt_get_rssi = 0;
          update_tracking_timer_value();
          error_counter_send_track++;
          if(send_udp_tracking_msg())
          {
            error_counter_send_track = 0;
          }
        }
      }

      if((xSemaphoreTake( xSemaphoreGsvTimerExpired, ( TickType_t ) 0) == pdTRUE) && (serv_cmd<<7))
      {
        error_cnt_read_gsv++;
        if(get_and_send_gsv())
        {
          error_cnt_read_gsv = 0;
        }
      }
    }
    if(!(error_counter_send_track < MAX_SEND_ERROR))
      error_code = ERROR_CODE_ID_TRACK_POS;
    else if(!(error_cnt_read_gga < MAX_READ_GGA))
      error_code = ERROR_CODE_GET_GGA;
    else if(!(error_cnt_read_gsv < MAX_READ_GSV))
      error_code = ERROR_CODE_GET_GSV;  
    else if(!(error_cnt_send_loc < MAX_SEND_GSM_LOC))
      error_code = ERROR_CODE_SEND_GSM_LOC;  
    else if(!(error_cnt_read_loc < MAX_READ_GSM_LOC))
      error_code = ERROR_CODE_GET_GSM_LOC;
    else if(!(error_cnt_get_rssi < MAX_READ_RSSI))
      error_code = ERROR_CODE_GSM_RSSI;
  }
  else
    error_code = ERROR_CODE_MISSING_RESP;
}

/***************************************************************************/
/* Get position of gsm */
/***************************************************************************/
bool enable_get_gsm_pos(void)
{
  bool success = false;
  vTaskDelay(20);
  while(!set_netw_reg_loc_urc())
    vTaskDelay(20);

// FIX FOR TESTING SETUP WITHOUT COUNTER IN SEND
//  if(set_netw_reg_loc_urc())
//  {
    vTaskDelay(20);
    if(set_disp_operator())
    {
      vTaskDelay(20);
      if(read_netw_reg_loc())
      {
        success = true;
      }
    }
//  }
  return success;
}

/***************************************************************************/
/* Main function for controlling network related */
/***************************************************************************/
bool init_droneid(void)
{
  bool success = false;
  uint8_t i;

  reset_utm_var();
  reset_gsm_modem();

  if(init_gsm_modem())
  {
    msg_mode = SMS_SERVER_CONNECTION;
    // Start timers
    if( (xTimerStart( xHandleTransmitTimer, DELAY_1000_MS ) == pdPASS) &&  (xTimerStart( xHandleGsvGPTimer, DELAY_1000_MS ) == pdPASS)
         &&  (xTimerStart( xHandleGSMInfoTimer, DELAY_1000_MS ) == pdPASS) &&  (xTimerStart( xHandleGetGgaTimer, DELAY_1000_MS ) == pdPASS))
    {
      for(i = 0; (power_mode_is_on() && (msg_mode != SOCKET_SERVER_CONNECTION) && (i < MAX_GPRS_CONNECT)); i++)
      {
        if(power_mode_is_on())
        {
          if(connect_socket())
          {
            msg_mode = SOCKET_SERVER_CONNECTION;
          }
          else
          {
            reset_gsm_modem();
            init_gsm_modem();
          }
        }
      }

      if(enable_get_gsm_pos())
      {
        for(i = 0; (power_mode_is_on() && (!success) && (i < MAX_GNSS_CONNECT)); i++)
        {
          gnss_pwr_on_flag = true;
          if(setup_and_start_gnss())
          {
            success = true;
          }
        }
      }
    }
  }
  return success;
}

/***************************************************************************/
/* Get stop condition */
/***************************************************************************/
uint8_t get_stop_condition(void)
{
  droneid_pwr_state_t mode = get_power_mode();
  uint8_t stop_code;

  if( mode == DRONEID_PWR_OFF)
  {
    stop_code = ERROR_CODE_USER_SHUT_DOWN;
  }
  else if( mode == DRONEID_PWR_OFF_LOW_BATT)
  {
    stop_code = ERROR_CODE_LOW_BATT_V;
  }
  else
    stop_code = error_code;

  return stop_code;
}

/***************************************************************************/
/* Loop for connecting, transmitting and rebooting in case of errors */
/***************************************************************************/
void enter_droneid_crtl_loop(void)
{
  TASK_LOOP
  {
    pre_gga_msg.lat = 0.0;
    pre_gga_msg.lon = 0.0;
    pre_gga_msg.alt = 0.0;

    if(!init_droneid())
    {
      if(power_mode_is_on())
        error_reset_mcu(); // Reboot
    }

    if(msg_mode == SOCKET_SERVER_CONNECTION)
    {
      if(send_udp_start_tracking_msg())
      {
        udp_tracking_mode();
        set_indication_start_mode(false);
        if(send_udp_stop_msg(get_stop_condition()))
        {
          power_down_gnss_gsm();
          if(!(get_stop_condition()>>7))
          {
            xSemaphoreGive(xSemaphoreGsmPwrIsDown);
            vTaskDelay(DELAY_20_MS / portTICK_RATE_MS);
            vTaskSuspend( NULL );
          }
        }
      }
    }
    else
    {
      error_reset_mcu(); // Reboot until sms is written
    }
  }
}

/***************************************************************************/
/* Main function for controlling network related */
/***************************************************************************/
void droneid_ctrl_main(void *pvParameters)
{
  /* Wait until powered up */
  while(!power_mode_is_on())
  {
    vTaskDelay(DELAY_100_MS / portTICK_RATE_MS);
  }

#ifdef DEBUG
  debug_add_ascii_to_queue("DroneID ctrl task: Started\n");
#endif
  
  enter_droneid_crtl_loop();
  vTaskSuspend( NULL );
}
