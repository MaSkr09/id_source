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
#include "global_defines.h"
#include "id_protocol_v1.h"

/***************************************************************************/
/* Defines */
/***************************************************************************/
transmit_error_counter_t transmit_error_counters;
#define SOCKET_CREATE_MAX_ATTEMPT   3

#define GPRS_CON_SLEEP_MS           50
#define MAX_GPRS_ITERATION          600

#define UDP_TRAKING_LOOP_DELAY_MS   50
#define MAX_TRACKING_UDP_ERRORS     3

#define MAX_GPRS_CONNECT            3
#define MAX_GNSS_CONNECT            3

#define NO_SERVER_CONNECTION        0
#define SMS_SERVER_CONNECTION       1
#define SOCKET_SERVER_CONNECTION    2

#define MSG_COUNT_DIF_WARN          4
#define MAX_RESP_DIFF_ERROR         5

#define MAX_SEND_ERROR              3
#define MAX_READ_GGA                5
#define MAX_READ_GSV                5
#define MAX_SEND_GSM_LOC            5
#define MAX_READ_GSM_LOC            5
#define MAX_READ_RSSI               5

#define TIME_5000_MS                5000
#define RESPONSE_LOOP_TIME          100

#define MAX_SYNC_ITERATION      50
#define SYNC_SLEEP_MS           50
#define MAX_DATA_MSG_SIZE         256
/***************************************************************************/
/* Shared variables */
/***************************************************************************/
bool gprs_mode_flag = false;
uint8_t msg_mode = NO_SERVER_CONNECTION;
bool gnss_pwr_on_flag = false;

gpgga_t pre_gga_msg;


/***************************************************************************/
/* Indication mode */
/***************************************************************************/
void set_indication_start_mode(bool start_mode)
{

  if(xSemaphoreTake( xSemaphoreTaskIndicators, ( TickType_t ) TIME_500_MS) == pdTRUE)
  {
    if(start_mode)
      ctrl_power_on = true;
    else
      ctrl_power_on = false;
    xSemaphoreGive( xSemaphoreTaskIndicators );
  }
}

/***************************************************************************/
/* Reset gsv data */
/***************************************************************************/
bool reset_gsv_data(gpgsv_t * gsv)
{
  bool return_value = false;
  if( xSemaphoreTake( xSemaphoreGsvMsg, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    return_value = !clear_gsv_msg(gsv);
    xSemaphoreGive( xSemaphoreGsvMsg );
  }
  return return_value;
}

/***************************************************************************/
/* Reset gga data */
/***************************************************************************/
bool reset_gga_data(gpgga_t * gga)
{
  bool return_value = false;
  if( xSemaphoreTake( xSemaphoreGgaMsg, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    return_value = !clear_gga_msg(gga);
    xSemaphoreGive( xSemaphoreGgaMsg );
  }
  return return_value;
}

/***************************************************************************/
/* Check for transmit error */
/***************************************************************************/
bool error_count_accepted(void)
{
    bool success = false;
    if((transmit_error_counters.error_counter_send_track < MAX_SEND_ERROR) 
    && (transmit_error_counters.error_cnt_read_gga < MAX_READ_GGA)
    && (transmit_error_counters.error_cnt_read_gsv < MAX_READ_GSV)  
    && (transmit_error_counters.error_cnt_send_loc < MAX_SEND_GSM_LOC)  
    && (transmit_error_counters.error_cnt_read_loc < MAX_READ_GSM_LOC)
    && (transmit_error_counters.error_cnt_get_rssi < MAX_READ_RSSI))
    {
      success = true;
    }
  return success;
}

/***************************************************************************/
/* Check for transmit error */
/***************************************************************************/
bool set_error_count(uint8_t *inst, uint8_t count_no)
{
  bool success = false;
  *inst = count_no;
  success = true;
  return success;
}

/***************************************************************************/
/* Error counter add  */
/***************************************************************************/
bool add_error_count(uint8_t *inst)
{
  bool success = false;
  *inst = (*inst) +1;
  success = true;
  vTaskDelay(TIME_1000_MS / portTICK_RATE_MS);
  return success;
}

/***************************************************************************/
/* Get transmit error */
/***************************************************************************/
uint8_t get_error_code()
{
  uint8_t return_code = ERROR_CODE_ID_TRACK_POS;

  if(transmit_error_counters.error_counter_send_track < MAX_SEND_ERROR)
  {
    return_code = ERROR_CODE_ID_TRACK_POS;
  }
  else if(transmit_error_counters.error_cnt_read_gga < MAX_READ_GGA)
  {
    return_code = ERROR_CODE_GET_GGA;
  }
  else if(transmit_error_counters.error_cnt_read_gsv < MAX_READ_GSV)
  {
    return_code = ERROR_CODE_GET_GSV;
  }
  else if(transmit_error_counters.error_cnt_send_loc < MAX_SEND_GSM_LOC)
  {
    return_code = ERROR_CODE_SEND_GSM_LOC;
  }
  else if(transmit_error_counters.error_cnt_read_loc < MAX_READ_GSM_LOC)
  {
    return_code = ERROR_CODE_GET_GSM_LOC;
  }
  else if(transmit_error_counters.error_cnt_get_rssi < MAX_READ_RSSI)
  {
    return_code = ERROR_CODE_GSM_RSSI;
  }
  return return_code;
}

/***************************************************************************/
/* Reset var */
/***************************************************************************/
bool reset_paramters(void)
{
  bool success = false;
  if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_50_MS ) == pdTRUE )
  {
    server_msg_count_id = 0;
    server_msg_time_interval = 1;
    new_server_msg_time_interval = 1;
    max_horizontal_server_msg = 50;
    max_vertical_server_msg = 20;
    flight_permission = SERVER_NOT_CONNECTED;
    serv_cmd = 0;
    xSemaphoreGive(xSemaphoreServerFlightMsgs);

    if( xSemaphoreTake( xSemaphoreCellId, ( TickType_t ) TIME_50_MS ) == pdTRUE )
    {
      gsm_signal_and_signal.SIGNAL_Q_VAR = 99;
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

        reset_gsv_data(gpgsv_data);
        reset_gsv_data(glgsv_data);
        
        reset_gga_data(&pre_gga_msg);
        reset_gga_data(&gga_data);

        set_error_count(&transmit_error_counters.cell_info_msg_count, 0);
        set_error_count(&transmit_error_counters.error_cnt_get_rssi, 0);
        set_error_count(&transmit_error_counters.error_cnt_read_gga, 0);
        set_error_count(&transmit_error_counters.error_cnt_read_gsv, 0);
        set_error_count(&transmit_error_counters.error_cnt_read_loc, 0);
        set_error_count(&transmit_error_counters.error_cnt_send_loc, 0);
        set_error_count(&transmit_error_counters.error_counter_send_track, 0);
        set_error_count(&transmit_error_counters.msg_count_cell_id, 0);
    }
  }

  set_indication_start_mode(true);
  
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

  vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
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

  vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
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

  vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
  if(get_gnss_gga())
  {
    success = true;
  }
  return success;
}

/***************************************************************************/
/* Send gsm info */
/***************************************************************************/
uint8_t msg_response_delay(void)
{
  uint8_t msg_delay = 0xFF;
  if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_5_MS ) == pdTRUE )
  {
    server_msg_count_id &= 0x0F;
    if(server_msg_count_id >= echo_msg_count_cell_id)
    {
      msg_delay = server_msg_count_id - echo_msg_count_cell_id;
    }
    else if(server_msg_count_id < echo_msg_count_cell_id)
    {
      msg_delay = server_msg_count_id + 15 - echo_msg_count_cell_id;
    }
    xSemaphoreGive( xSemaphoreServerFlightMsgs );
  }
  return msg_delay;
}

/***************************************************************************/
/* Check and read if modem contains msg from server */
/***************************************************************************/
bool read_udp_server_msg(void)
{
  bool success = false;
  uint16_t bytes = 1;
  uint8_t i=0;

    if( xSemaphoreTake( xSemaphoreUbloxStatusReg, ( TickType_t ) TIME_5_MS ) == pdTRUE )
    {
      bytes = ublox_status_reg.UDP_NO_OF_BYTES_TO_READ;
      xSemaphoreGive(xSemaphoreUbloxStatusReg);
    }
    if(bytes != 0)
    {
      vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
      if(read_udp_no_of_msg_bytes())
        success = true;
    }
    // check counter value and update parameter if missing response from server
    else if(msg_response_delay() <= MSG_COUNT_DIF_WARN)
    {
      success = read_udp_no_of_msg_bytes();
    }
  return success;
}

/***************************************************************************/
/* Send gsm info */
/***************************************************************************/
bool gsm_info_update(void)
{
  bool success = false, send_msg = false;

  vTaskDelay(TIME_20_MS);

    if( xSemaphoreTake( xSemaphoreCellId, ( TickType_t ) TIME_5_MS ) == pdTRUE )
    {
      if((gsm_signal_and_signal.pre_mob_cell_id_code != gsm_signal_and_signal.mob_cell_id_code) ||
        (gsm_signal_and_signal.pre_mob_location_area_code != gsm_signal_and_signal.mob_location_area_code))
      {
        send_msg = true;
        gsm_signal_and_signal.pre_mob_cell_id_code = gsm_signal_and_signal.mob_cell_id_code;
        gsm_signal_and_signal.pre_mob_location_area_code = gsm_signal_and_signal.mob_location_area_code;
      }
      else
      {
        success = true;
      }
      xSemaphoreGive( xSemaphoreCellId );
    }

  if(send_msg)
  {
    if(send_gsm_nw_msg())
    {
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
  vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
  reset_gsv_data(gpgsv_data);
  reset_gsv_data(glgsv_data);

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

  if( xSemaphoreTake( xSemaphoreGgaMsg, ( TickType_t ) TIME_500_MS ) == pdTRUE )
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
    if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_20_MS ) == pdTRUE )
    {
      if(new_server_msg_time_interval != server_msg_time_interval)
      {
        server_msg_time_interval = new_server_msg_time_interval;
        xTimerChangePeriod(xHandleTransmitTimer, (1000*server_msg_time_interval), TIME_20_MS);
        xTimerReset( xHandleTransmitTimer, TIME_20_MS);
      }
      xSemaphoreGive(xSemaphoreServerFlightMsgs);
    }
  }
  else
  {
    if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_20_MS ) == pdTRUE )
    {
      if(GMS_MSG_PERIOD_S != server_msg_time_interval)
      {
        server_msg_time_interval = GMS_MSG_PERIOD_S;
        xTimerChangePeriod(xHandleTransmitTimer, (1000*server_msg_time_interval), TIME_20_MS);
        xTimerReset( xHandleTransmitTimer, TIME_20_MS);
      }
      xSemaphoreGive( xSemaphoreServerFlightMsgs );
    }
  }
}

/***************************************************************************/
/* Check if droneid has moved */
/***************************************************************************/
bool droneID_has_moved(void)
{
  bool return_value = false;
  if( xSemaphoreTake( xSemaphoreGgaMsg, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
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
  if( xSemaphoreTake( xSemaphoreGgaMsg, ( TickType_t ) TIME_500_MS ) == pdTRUE )
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
  bool response_ok = true;
  int16_t timeout = TIME_5000_MS;
  
  while((!(response_ok)) && (timeout > 0))
  {
    read_udp_server_msg();
    if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
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
  else if(msg_response_delay() <= MAX_RESP_DIFF_ERROR)
  {
    stop_code = ERROR_CODE_MISSING_RESP;
  }
  else
  {
    stop_code = get_error_code();
  }

  return stop_code;
}

/***************************************************************************/
/* Get position of gsm */
/***************************************************************************/
bool enable_get_gsm_pos(void)
{
  bool success = false;
  vTaskDelay(TIME_20_MS);
  while(!set_netw_reg_loc_urc())
    vTaskDelay(TIME_20_MS);

  vTaskDelay(TIME_20_MS);
  if(set_disp_operator())
  {
    vTaskDelay(TIME_20_MS);
    if(read_netw_reg_loc())
    {
      success = true;
    }
  }
  return success;
}

/***************************************************************************/
/* Init gsm modem */
/***************************************************************************/
bool init_gsm_modem(void)
{
  bool success = false, gsm_modem_con = false;
  uint16_t counter = 0;

  while((counter++ < MAX_SYNC_ITERATION) && (power_mode_is_on()) && !gsm_modem_con )
  {
    gsm_modem_con = sync_gsm_modem();
    if(!gsm_modem_con)
    {
      vTaskDelay(SYNC_SLEEP_MS);
    }
  }
  if(gsm_modem_con)
  {
    vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
    if(disable_gsm_modem_echo())
    {
      vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
      if(pin_code_check())
      {
        success = true;
      }
    }
  }
  return success;
}

/***************************************************************************/
/* Connect GPRS */
/***************************************************************************/
bool connect_gprs(void)
{
  bool gprs_con = false;
  uint16_t counter = 0;

  while((counter++ < MAX_GPRS_ITERATION) && (power_mode_is_on()) && !gprs_con )
  {
    gprs_con = check_gprs_attached();
    if(!gprs_con)
    {
      vTaskDelay(GPRS_CON_SLEEP_MS);
    }
  }
  return gprs_con;
}

/***************************************************************************/
/* Start transmit timers */
/***************************************************************************/
bool start_timers(void)
{
  bool success = false;
  if( (xTimerStart( xHandleTransmitTimer, TIME_1000_MS ) == pdPASS) &&  (xTimerStart( xHandleGsvGPTimer, TIME_1000_MS ) == pdPASS)
         &&  (xTimerStart( xHandleGSMInfoTimer, TIME_1000_MS ) == pdPASS) &&  (xTimerStart( xHandleGetGgaTimer, TIME_1000_MS ) == pdPASS))
  {
    success = true;
  }
  return success;
}

/***************************************************************************/
/* Setup socket */
/***************************************************************************/
bool setup_socket(void)
{
  bool success = false;
  uint8_t i;

  vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
  for(i = 0; (power_mode_is_on() && (success == false) && (i < MAX_GPRS_CONNECT)); i++)
  {
    if(PSD_PROF_CONF())
    {
      vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
      if(psd_conf_ip())
      {
        vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
        if(psd_pdp_activate())
        {
          success = true;
        }
      }
    }
  }
  return success;
}

/***************************************************************************/
/* Open socket to server */
/***************************************************************************/
bool open_id_socket_server(void)
{
  bool success = false;
  vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
  if(open_socket())
  {
    vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
    if(get_server_ip())
    {
      success = true;
    }
  }
}

/***************************************************************************/
/* Setup and start GNSS with aiding server etc. */
/***************************************************************************/
bool setup_and_start_gnss(void)
{
  bool success = false;
  vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
  if(setup_profile())
  {
    vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
    if(setup_aiding_server())
    {
      vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
      if(setup_store_gga())
      {
        vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
        if(setup_store_gsv())
        {
          vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
          if(setup_unsolicited_indication())
          {
            vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
            if(power_on_gnss())
            {
              success = true;
            }
          }
        }
      }
    }
  }
  return success;
}

/***************************************************************************/
/* Main function for controlling network related */
/***************************************************************************/
bool init_droneid(void)
{
  bool success = false;

  if(init_gsm_modem())
  {
    msg_mode = SMS_SERVER_CONNECTION;
    if(connect_gprs())
    {
      if(start_timers())
      {
        if(setup_socket())
        {
          if(open_id_socket_server())
          {
            msg_mode = SOCKET_SERVER_CONNECTION;            
            if(setup_and_start_gnss())
            {
              gnss_pwr_on_flag = true;
              if(enable_get_gsm_pos())
              {
                success = true;
              }
            }
          }
        }
      }
    }
  }
  return success;
}

/***************************************************************************/
/* Send start package to server */
/***************************************************************************/
bool send_udp_start_tracking_msg()
{
  bool success = false;
  uint16_t msg_len = 0;
  uint8_t data_msg[MAX_DATA_MSG_SIZE], array_buffer[3], socket_no[2];
  if(build_start_msg(&msg_len, data_msg))
  {
    if(msg_len != 0)
    {
      vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
      if(send_udp_msg(data_msg, msg_len))
      {
        success = true;
      }
    }
  }
  return success;
}


/***************************************************************************/
/* Update gsm info */
/***************************************************************************/
void update_gsm_info()
{
  if(read_netw_reg_loc())
  {
    set_error_count(&transmit_error_counters.error_cnt_read_loc, 0);
    if(gsm_info_update())
    {
      set_error_count(&transmit_error_counters.error_cnt_send_loc, 0);
    }
    else
    {
      add_error_count(&transmit_error_counters.error_cnt_send_loc);
    }
  }
  else
  {
    add_error_count(&transmit_error_counters.error_cnt_read_loc);
  }
}

/***************************************************************************/
/* Get gga msg  */
/***************************************************************************/
void get_gga(void)
{
  if(update_gga_data())
  {
    set_error_count(&transmit_error_counters.error_cnt_read_gga, 0);
    if(gnss_fix())
    {
      if(droneID_has_moved())
      {
        xSemaphoreGive( xSemaphoreTransmitPos );
        xTimerReset( xHandleTransmitTimer, TIME_20_MS);
      }
    }
  }
  else
  {
      add_error_count(&transmit_error_counters.error_cnt_read_gga);
  }

}

/***************************************************************************/
/* Send dronid pos to server */
/***************************************************************************/
void transmit_position(void)
{
  update_pre_pos();
  if(update_rssi_values())
  {
    set_error_count(&transmit_error_counters.error_cnt_get_rssi, 0);
    update_tracking_timer_value();
    if(send_udp_tracking_msg())
    {
      set_error_count(&transmit_error_counters.error_counter_send_track, 0);
    }
    else
    {
      add_error_count(&transmit_error_counters.error_counter_send_track);
    }
  }
  else
  {
    add_error_count(&transmit_error_counters.error_cnt_get_rssi);
  }
}

/***************************************************************************/
/* Get gsv data  */
/***************************************************************************/
void get_gsv(void)
{
  if(get_and_send_gsv())
  {
    set_error_count(&transmit_error_counters.error_cnt_read_gsv , 0);
  }
  else
  {
    add_error_count(&transmit_error_counters.error_cnt_read_gsv);
  }
}

/***************************************************************************/
/* Send stop msg and power down id */
/***************************************************************************/
bool stop_droneid(void)
{
  bool success = false;
  send_udp_stop_msg(get_stop_condition());
  power_down_gnss_gsm();
  if(!(get_stop_condition()>>7))
  {
    success = true;
  }
  return success;
}

/***************************************************************************/
/* Enter tracking mode. Get positioning updates, transmit to server and    */
/* read server msgs                                                        */
/***************************************************************************/
bool tracking_mode(void)
{
  bool success = false;
  uint8_t msg_delay = 0;
  if(start_response_from_server())
  {
    success = true;

    while(power_mode_is_on() && error_count_accepted())// && (msg_delay <= MAX_RESP_DIFF_ERROR)) does not work proper and delay is to big
    {
//      if(xSemaphoreTake( xSemaphoreGSMInfoTimerExpired, ( TickType_t ) 0) == pdTRUE)
//      {
//        read_udp_server_msg();
//      }

      if(xSemaphoreTake( xSemaphoreGetGgga, ( TickType_t ) 0) == pdTRUE)
      {
        read_udp_server_msg();
        get_gga();
      }

      if(xSemaphoreTake( xSemaphoreTransmitPos, ( TickType_t ) UDP_TRAKING_LOOP_DELAY_MS) == pdTRUE)
      {
        read_udp_server_msg();
        update_gsm_info();
        transmit_position();
        msg_delay = msg_response_delay();
      }

      if((xSemaphoreTake( xSemaphoreGsvTimerExpired, ( TickType_t ) 0) == pdTRUE) && (serv_cmd<<7))
      {
        get_gsv();
      }
      gsm_info_update();
    }
  }
  return success;
}

/***************************************************************************/
/* Loop for connecting, transmitting and rebooting in case of errors */
/***************************************************************************/
bool enter_droneid_crtl_loop(void)
{
  bool success = false;
  if(send_udp_start_tracking_msg())
  {
    if(gsm_info_update())
    {
      if(tracking_mode())
      {
        if(power_mode_is_on())
        {
          success = true;
        }
      }
    }
  }

  return success;
}

/***************************************************************************/
/* Loop for connecting, transmitting and rebooting in case of errors */
/***************************************************************************/
void enter_droneid_loop(void)
{
  if(power_on_gsm_modem())
  {
    TASK_LOOP
    {
      if(reset_paramters())
      {
        if(init_droneid())
        {
          if(enter_droneid_crtl_loop())
          {
          }
        }
      }
      if(power_mode_is_on())
      {
        // Error has occurred that requires rebooting. Try sending stop msg first
        set_indication_start_mode(false);
        send_udp_stop_msg(get_stop_condition());
        vTaskDelay(TIME_15_S / portTICK_RATE_MS);
        error_reset_mcu();
      }
      else
      {
        stop_droneid();
        set_indication_start_mode(false);
        xSemaphoreGive(xSemaphoreGsmPwrIsDown);
        vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
        vTaskSuspend( NULL );
      }

    }
  }
  error_reset_mcu();
}

/***************************************************************************/
/* Main function for controlling network related */
/***************************************************************************/
void droneid_ctrl_main(void *pvParameters)
{
  /* Wait until powered up */
  while(!power_mode_is_on())
  {
    vTaskDelay(TIME_100_MS / portTICK_RATE_MS);
  }

#ifdef DEBUG
  debug_add_ascii_to_queue("DroneID ctrl task: Started\n");
#endif
  
  enter_droneid_loop();
  error_reset_mcu(); // Reboot
}
