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
* File: ublox_gnss_gsm_driver.c
* Purpose: Driver for connecting to Ublox SARA G3 and CAM-M8Q modules and transmit data
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-05-29 Martin Skriver, Source written
****************************************************************************/
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>

#include "pwr_management_task.h"
#include "slip.h"
#include "crc.h"
#include "main.h"
#include "uart_transmit.h"
#include "ublox_gnss_gsm_driver.h"
#include "ublox_private_token.h"
#include "server_settings.h"
#include "id_config.h"
#include "global_defines.h"
#include "id_protocol_v1.h"


/***************************************************************************/
/* Defines */
/***************************************************************************/
#define RESP_LOOP_TIME_MS             10
#define NUMBER_0                      0
#define MAX_DATA_SIZE                 512

// GSM
#define RESET_TIME_MS                 60
#define PWR_DOWN_RESP_TIME_MS         20000
#define POWER_UP_TIME_MS              3000
#define SYNC_RESP_TIME_MS             200
#define DIS_ECHO_RESP_TIME_MS         2000
#define PIN_CHECK_RESP_TIME_MS        10000
#define GPRS_ATTACH_RESP_TIME_MS      1000
#define PSD_RESP_TIME_MS              5000
#define PSD_IP_RESP_TIME_MS           5000
#define PSD_PDP_RESP_TIME_MS          5000
#define open_sock_RESP_TIME_MS        10000
#define GET_SERV_IP_RESP_TIME_MS      10000

#define GET_NO_OF_BYTES_TIME_MS       5000

#define RSSI_TIME_MS                  2000 
#define OPEN_MSG_TIME_MS              15000
#define MAX_MSG_SEND_REPONSE_MS       10000

#define NETW_REG_LOC_TIME_MS          1000
#define OPR_NAME_TIME_MS              1000

// GNSS
#define GNSS_PROFILE_TIME_MS          1000
#define GNSS_PROFILE_RESP_MAX_RETRY   3
#define SETUP_AIDING_TIME_MS          1000
#define SETUP_AIDING_RESP_MAX_RETRY   2
#define UNSOL_INDIC_TIME_MS           2000
#define UNSOL_INDIC_RESP_MAX_RETRY    3
#define STORE_GGA_TIME_MS             1000
#define STORE_GGA_RESP_MAX_RETRY      2
#define STORE_GSV_TIME_MS             1000
#define STORE_GSV_RESP_MAX_RETRY      2
#define GNSS_PWR_ON_TIME_MS           10000
#define GNSS_PWR_ON_RESP_MAX_RETRY    3
#define GNSS_PWR_OFF_TIME_MS          10000
#define GNSS_PWR_OFF_RESP_MAX_RETRY   3
#define UPDATE_GNSS_DATA_TIME_MS      1000
#define UPDATE_GNSS_DATA_MAX_RETRY    4
#define degs_to_semicircles (double)  11930464.7111

#define MAX_DATA_MSG_SIZE         256


/***************************************************************************/
/* Reset GSM modem parameters */
/***************************************************************************/
bool reset_modem_parameters()
{
  bool success = false;
  uint8_t i, j;
  if(xSemaphoreTake( xSemaphoreUbloxStatusReg, ( TickType_t ) TIME_500_MS ) == pdTRUE)
  {
    ublox_status_reg.DIS_ECHO_RESULT_CODE = NONE_RESULT_CODE;
    ublox_status_reg.CPIN_RESULT_CODE = NONE_RESULT_CODE;
    ublox_status_reg.CGATT_RESULT_CODE = NONE_RESULT_CODE;
    ublox_status_reg.GP_RESULT_CODE = NONE_RESULT_CODE;
    ublox_status_reg.RECEIVED_GSM_PWR_DOWN_RESULT_CODE = NONE_RESULT_CODE;
    ublox_status_reg.CREATED_SOCKET_NO_RESULT_CODE = NONE_RESULT_CODE;
    ublox_status_reg.SERVER_IP_RESULT_CODE = NONE_RESULT_CODE;
    ublox_status_reg.MODEM_READY_FOR_DATA = NONE_RESULT_CODE;
    ublox_status_reg.UDP_NO_OF_BYTES_TO_READ = 0;
    ublox_status_reg.UDP_READING_NO_OF_BYTES = 0;
    xSemaphoreGive(xSemaphoreUbloxStatusReg);
  }
  return success;
}

/***************************************************************************/
/* Clear OK result code flag */
/***************************************************************************/
bool clear_result_code(ublox_result_code_t * instance)
{
  bool result_code = false;
  if(xSemaphoreTake( xSemaphoreUbloxStatusReg, ( TickType_t ) TIME_30_MS ) == pdTRUE)
  {
    *instance = NONE_RESULT_CODE;
    result_code = true;
    xSemaphoreGive(xSemaphoreUbloxStatusReg);
  }
  return result_code;
}

/***************************************************************************/
/* Check if there is response from gsm modem */
/***************************************************************************/
bool check_response_timeout(uint16_t max_time, SemaphoreHandle_t *mutex, ublox_result_code_t *control)
{
  bool success = false;
  uint16_t timer = NUMBER_0;

  while(timer < max_time && !success )
  {
    if(xSemaphoreTake( mutex, ( TickType_t ) TIME_500_MS ) == pdTRUE)
    {
      if(*control == OK_RESULT_CODE)
        success = true;
      else
        timer += RESP_LOOP_TIME_MS;

      xSemaphoreGive(mutex);
    }
    vTaskDelay(RESP_LOOP_TIME_MS / portTICK_RATE_MS);
  }
  return success;
}

/***************************************************************************/
/* Send command to gsm modem */
/***************************************************************************/
bool send_and_receive_at(uint8_t *msg, uint16_t max_time, SemaphoreHandle_t *mutex, ublox_result_code_t *control)
{
  bool success = false;

  if(add_ascii_to_send_queue(msg))
  {
    if(check_response_timeout(max_time, mutex, control))
    {
      success = true;
    }
  }
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

/***************************************************************************/
/* Send string, clear flag and receive ok from server */
/***************************************************************************/
bool build_and_send_msg(uint16_t max_time, ublox_result_code_t *control, SemaphoreHandle_t *mutex, uint8_t *command, ...)
{
  bool success = false;
  uint8_t server_tx_array[MAX_DATA_SIZE];
  va_list ap;
  uint8_t *ptr;

  va_start(ap, command);
  ptr = command;
  strcpy(server_tx_array, "");
  while(ptr != NULL)
  {
    strcat(server_tx_array, ptr);
    ptr = va_arg(ap, uint8_t*);
  }
  va_end(ap);

  if(clear_result_code(control))
  {
    success = send_and_receive_at(server_tx_array, max_time, mutex, control);
  }
  return success;
}

/***************************************************************************/
/* Transmit msg to server */
/***************************************************************************/
bool send_udp_msg(uint8_t *data_msg, uint8_t data_len)
{
  bool success = false;
  uint8_t array_buffer[3], socket_no[2], ip_var[18];
  sprintf(array_buffer, "%d", data_len);

  if( xSemaphoreTake( xSemaphoreUbloxStatusReg, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    strcpy(ip_var, socket_data.SERVER_IP);
    xSemaphoreGive( xSemaphoreUbloxStatusReg );
    if( xSemaphoreTake( xSemaphoreSocket, ( TickType_t ) TIME_500_MS ) == pdTRUE )
    {
      sprintf(socket_no, "%d", socket_data.CREATED_SOCKET_NO);
      xSemaphoreGive( xSemaphoreSocket );

      clear_result_code(&ublox_status_reg.GP_RESULT_CODE);
      // Removed if() to avoid open msg waiting for data
      build_and_send_msg(OPEN_MSG_TIME_MS, 
                            &ublox_status_reg.MODEM_READY_FOR_DATA,
                            xSemaphoreUbloxStatusReg, 
                            AT_COM_START, AT_PLUS, AT_SEND_TO_SERVER, AT_EQUALS, 
                            socket_no, AT_COMMA, ip_var, AT_COMMA, 
                            DRONEID_SERVER_UDP_PORT, AT_COMMA, array_buffer, AT_END_OF_MSG, NULL);
      vTaskDelay(TIME_20_MS / portTICK_RATE_MS);

      if(add_data_to_send_queue(data_msg, data_len))
      {
        if(check_response_timeout(MAX_MSG_SEND_REPONSE_MS, xSemaphoreUbloxStatusReg, &ublox_status_reg.GP_RESULT_CODE))
        {
          success = true;
        }
      }
    }
  }
  return success;
}

/***************************************************************************/
/* sync baud and check connection to GSM modem */
/***************************************************************************/
bool sync_gsm_modem()
{
  bool success;
  success =  build_and_send_msg(SYNC_RESP_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Reset GSM modem */
/***************************************************************************/
bool reset_gsm_modem()
{
  bool success = false;
  HAL_GPIO_WritePin(GPIO_SARA_G340_RESET_O_GPIO_Port, GPIO_SARA_G340_RESET_O_Pin, SET);
  vTaskDelay(RESET_TIME_MS / portTICK_RATE_MS);
  HAL_GPIO_WritePin(GPIO_SARA_G340_RESET_O_GPIO_Port, GPIO_SARA_G340_RESET_O_Pin, RESET);
  vTaskDelay(POWER_UP_TIME_MS / portTICK_RATE_MS);

  return success;
}

/***************************************************************************/
/* Power on GSM modem */
/***************************************************************************/
bool power_on_gsm_modem()
{
  bool success = true;
  HAL_GPIO_WritePin(GPIO_SARA_G340_PWR_ON_O_GPIO_Port, GPIO_SARA_G340_PWR_ON_O_Pin, SET);
  vTaskDelay(POWER_UP_TIME_MS / portTICK_RATE_MS);
  HAL_GPIO_WritePin(GPIO_SARA_G340_PWR_ON_O_GPIO_Port, GPIO_SARA_G340_PWR_ON_O_Pin, RESET);

  return success;
}

/***************************************************************************/
/* Get server IP */
/***************************************************************************/
bool get_server_ip(void)
{
  bool success;
    success =  build_and_send_msg(GET_SERV_IP_RESP_TIME_MS, 
                                  &ublox_status_reg.SERVER_IP_RESULT_CODE,
                                  xSemaphoreUbloxStatusReg, 
                                  AT_COM_START, 
                                  AT_PLUS, 
                                  AT_RESOLVE_SERVER_IP, 
                                  AT_EQUALS, 
                                  AT_DOMAIN_NAME_TO_IP, 
                                  AT_COMMA, 
                                  DRONEID_SERVER_URL, 
                                  AT_END_OF_MSG, 
                                  NULL);
  return success;
}

/***************************************************************************/
/* Open socket */
/***************************************************************************/
bool open_socket(void)
{
  bool success;
  success =  build_and_send_msg(open_sock_RESP_TIME_MS, 
                              &ublox_status_reg.CREATED_SOCKET_NO_RESULT_CODE,
                              xSemaphoreUbloxStatusReg, 
                              AT_COM_START, 
                              AT_PLUS, 
                              AT_CREATE_SOCKET, 
                              AT_EQUALS, 
                              AT_UDP_PROTOCOL, 
                              AT_END_OF_MSG, 
                              NULL);
  return success;
}

/***************************************************************************/
/* PSD activate PDP */
/***************************************************************************/
bool psd_pdp_activate(void)
{
  bool success;
  success =  build_and_send_msg(PSD_PDP_RESP_TIME_MS, 
                              &ublox_status_reg.GP_RESULT_CODE,
                              xSemaphoreUbloxStatusReg, 
                              AT_COM_START, 
                              AT_PLUS, 
                              AT_PACK_SWITCH_DATA_ACTION, 
                              AT_EQUALS, 
                              AT_PSD_PROFILE, 
                              AT_COMMA, 
                              AT_PDP_ACTION, 
                              AT_END_OF_MSG, 
                              NULL);
  return success;
}

/***************************************************************************/
/* Config psd to dynamic IP */
/***************************************************************************/
bool psd_conf_ip(void)
{
  bool success;
  success =  build_and_send_msg(PSD_IP_RESP_TIME_MS, 
                              &ublox_status_reg.GP_RESULT_CODE,
                              xSemaphoreUbloxStatusReg, 
                              AT_COM_START, 
                              AT_PLUS, 
                              AT_PACK_SWITCH_CONFIG, 
                              AT_EQUALS, 
                              AT_PSD_PROFILE, 
                              AT_COMMA, 
                              AT_IP_ADDR_PARA, 
                              AT_COMMA, 
                              AP_VAR_IP_ADDR, 
                              AT_END_OF_MSG, 
                              NULL);
  return success;
}

/***************************************************************************/
/* Check PSD APN configured */
/***************************************************************************/
bool PSD_PROF_CONF(void)
{
  bool success;
  success =  build_and_send_msg(PSD_RESP_TIME_MS, 
                              &ublox_status_reg.GP_RESULT_CODE,
                              xSemaphoreUbloxStatusReg, 
                              AT_COM_START, 
                              AT_PLUS, 
                              AT_PACK_SWITCH_CONFIG, 
                              AT_EQUALS, 
                              AT_PSD_PROFILE, 
                              AT_COMMA, 
                              AT_IPV4_PROTOCOL, 
                              AT_COMMA, 
                              AT_APN, 
                              AT_END_OF_MSG, 
                              NULL);
  return success;
}

/***************************************************************************/
/* Check GPRS attached */
/***************************************************************************/
bool check_gprs_attached(void)
{
  bool success;
  if( build_and_send_msg(GPRS_ATTACH_RESP_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_PLUS, 
                                GPRS_ATTACHED, 
                                AT_READ, 
                                AT_END_OF_MSG, 
                                NULL))
  {
    success =  check_response_timeout(TIME_20_MS, xSemaphoreUbloxStatusReg, &ublox_status_reg.CGATT_RESULT_CODE);
  }
  return success;
}

/***************************************************************************/
/* Pin code check */
/***************************************************************************/
bool pin_code_check(void)
{
  bool success;
  success =  build_and_send_msg(PIN_CHECK_RESP_TIME_MS, 
                                &ublox_status_reg.CPIN_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START,
                                AT_PLUS, 
                                CPIN_CHECK, 
                                AT_READ, 
                                AT_END_OF_MSG,
                                NULL);
  return success;
}

/***************************************************************************/
/* Disable GSM modem echo */
/***************************************************************************/
bool disable_gsm_modem_echo(void)
{
  bool success;
  success =  build_and_send_msg(DIS_ECHO_RESP_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
//                                &ublox_status_reg.DIS_ECHO_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_DISABLE_ECHO, 
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Power down GSM modem */
/***************************************************************************/
bool power_down_gsm_modem()
{
  bool success = false;
  success =  build_and_send_msg(PWR_DOWN_RESP_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_PLUS, 
                                AT_POWER_OFF_GSM, 
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Setup gnss aiding server */
/***************************************************************************/
bool setup_store_gsv(void)
{
  bool success;
  success =  build_and_send_msg(STORE_GSV_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_PLUS, 
                                AT_GET_UGGSV, 
                                AT_EQUALS, 
                                AT_GNSS_ENABLE, 
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Setup gnss aiding server */
/***************************************************************************/
bool setup_store_gga(void)
{
  bool success;
  success =  build_and_send_msg(STORE_GGA_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_PLUS, 
                                AT_GET_UGGGA, 
                                AT_EQUALS, 
                                AT_GNSS_ENABLE, 
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Setup gnss aiding server */
/***************************************************************************/
bool setup_unsolicited_indication(void)
{
  bool success;
  success =  build_and_send_msg(UNSOL_INDIC_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_PLUS, 
                                AT_UNSOLICITED_GNSS_MSG, 
                                AT_EQUALS, 
                                AT_UNSOLICITED_EN, 
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Setup gnss aiding server */
/***************************************************************************/
bool setup_aiding_server(void)
{
  bool success;
  success =  build_and_send_msg(SETUP_AIDING_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                  AT_COM_START, AT_PLUS, AT_AIDING_CONFIG, AT_EQUALS, AT_AIDING_SERVER_1,
                  AT_COMMA, AT_AIDING_SERVER_2, AT_COMMA, AT_UBLOX_PRIVATE_TOKEN, AT_COMMA, AT_VALID_DAYS,
                  AT_COMMA, AT_VALID_WEEKS_PERIOD, AT_COMMA, AT_RESOLUTION_DAYS, AT_COMMA, AT_GNSS_TYPE,
                  AT_COMMA, AT_ASSIST_OPERATING_MODE, AT_COMMA, AT_AIDING_DATA_TYPE, AT_END_OF_MSG, NULL);
  return success;
}

/***************************************************************************/
/* setup gnss profile */
/***************************************************************************/
bool setup_profile(void)
{
  bool success;
  success =  build_and_send_msg(GNSS_PROFILE_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_PLUS, 
                                AT_GNSS_PROFILE, 
                                AT_EQUALS, 
                                AT_GNSS_PROFILE_NUMBER, 
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Power on gnss */
/***************************************************************************/
bool power_on_gnss(void)
{
  bool success;
  success =  build_and_send_msg(GNSS_PWR_ON_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_PLUS, 
                                AT_GNSS_PWR_CMD, 
                                AT_EQUALS, 
                                AT_GNSS_PWR_ON, 
                                AT_COMMA, 
                                AT_ASSIST_LOCAL_AUTONOM_ONLINE, 
                                AT_COMMA, 
                                GNSS_SYSTEM_GLO_GPS, 
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Power off gnss */
/***************************************************************************/
bool power_off_gnss(void)
{
  bool success = false;
  success =  build_and_send_msg(GNSS_PWR_ON_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_PLUS, 
                                AT_GNSS_PWR_CMD, 
                                AT_EQUALS, 
                                AT_GNSS_PWR_OFF, 
                                AT_END_OF_MSG,
                                NULL);
  return success;
}

/***************************************************************************/
/* Get gsm modem rssi */
/***************************************************************************/
bool update_rssi(void)
{
  bool success = false;
  success =  build_and_send_msg(RSSI_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_PLUS, 
                                AT_QUALITY,
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Set urc registration network */
/***************************************************************************/
bool set_netw_reg_loc_urc(void)
{
  bool success = false;
  success =  build_and_send_msg(NETW_REG_LOC_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_GPRS_NETWORK_REG_LOC,
                                AT_EQUALS,
                                AT_EN_NETW_REG_URC
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Read registration network */
/***************************************************************************/
bool read_netw_reg_loc(void)
{
  bool success = false;
  success =  build_and_send_msg(NETW_REG_LOC_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_GPRS_NETWORK_REG_LOC,
                                AT_READ,
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Set display operator name */
/***************************************************************************/
bool set_disp_operator(void)
{
  bool success = false;
  success =  build_and_send_msg(OPR_NAME_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_DISP_OPR,
                                AT_EQUALS,
                                AT_OPR_NUM_FORM,
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Get latest gga msg */
/***************************************************************************/
bool get_gnss_gga(void)
{
  bool success = false;
  success =  build_and_send_msg(UPDATE_GNSS_DATA_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_PLUS, 
                                AT_GET_UGGGA, 
                                AT_READ, 
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Get latest gsv msg */
/***************************************************************************/
bool get_gnss_gsv(void)
{
  bool success = false;
  success =  build_and_send_msg(UPDATE_GNSS_DATA_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_PLUS, 
                                AT_GET_UGGSV, 
                                AT_READ, 
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Send tracking msg */
/***************************************************************************/
bool send_udp_tracking_msg(void)
{
  bool success = false;
  uint16_t msg_len = 0;
  uint8_t data_msg[MAX_DATA_MSG_SIZE], array_buffer[3], socket_no[2];
  vTaskDelay(TIME_20_MS / portTICK_RATE_MS);
  if(build_tracking_msg(&msg_len, data_msg))
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
/* Send stop tracking udp msg */
/***************************************************************************/
bool send_udp_stop_msg(uint8_t stop_condition)
{
  bool success = false;
  uint16_t msg_len = 0;
  uint8_t data_msg[MAX_DATA_MSG_SIZE];

  if(build_stop_msg(&msg_len, data_msg, stop_condition))
  {
    if(msg_len != 0)
    {
      if(send_udp_msg(data_msg, msg_len))
      {
        success = true;
      }
    }
  }
  return success;
}

/***************************************************************************/
/* Read udp number of bytes to be read */
/***************************************************************************/
bool read_udp_no_of_msg_bytes()
{
  bool success = false;
  uint8_t server_tx_array[50];
  uint8_t socket_no[2], byte_to_read[2];
  if( xSemaphoreTake( xSemaphoreUbloxStatusReg, ( TickType_t ) TIME_500_MS ) == pdTRUE )
  {
    if( xSemaphoreTake( xSemaphoreSocket, ( TickType_t ) TIME_500_MS ) == pdTRUE )
    {
      sprintf(socket_no, "%d", socket_data.CREATED_SOCKET_NO);
      ublox_status_reg.UDP_READING_NO_OF_BYTES = ublox_status_reg.UDP_NO_OF_BYTES_TO_READ;
      sprintf(byte_to_read, "%d", ublox_status_reg.UDP_READING_NO_OF_BYTES  );
      xSemaphoreGive(xSemaphoreSocket);
    }
    xSemaphoreGive(xSemaphoreUbloxStatusReg);
  }
  success =  build_and_send_msg(GET_NO_OF_BYTES_TIME_MS, 
                                &ublox_status_reg.GP_RESULT_CODE,
                                xSemaphoreUbloxStatusReg, 
                                AT_COM_START, 
                                AT_PLUS, 
                                AT_READ_MSG_FROM_SERVER, 
                                AT_EQUALS,
                                socket_no,
                                AT_COMMA,
                                byte_to_read,
                                AT_END_OF_MSG, 
                                NULL);
  return success;
}

/***************************************************************************/
/* Send gps sv package */
/***************************************************************************/
bool send_udp_gpgsv(void)
{
  bool success = false;
  uint16_t msg_len = 0;
  uint8_t data_msg[MAX_DATA_MSG_SIZE], array_buffer[3], socket_no[2];
  if(build_gsv_msg(&msg_len, data_msg, gpgsv_data, UDP_GPGSV_MSG_TYPE))
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
/* Send gps sv package */
/***************************************************************************/
bool send_udp_glgsv(void)
{
  bool success = false;
  uint16_t msg_len = 0;
  uint8_t data_msg[MAX_DATA_MSG_SIZE], array_buffer[3], socket_no[2];
  if(build_gsv_msg(&msg_len, data_msg, glgsv_data, UDP_GLGSV_MSG_TYPE))
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
/* Send gsm network package to server */
/***************************************************************************/
bool send_gsm_nw_msg()
{
  bool success = false;
  uint16_t msg_len = 0;
  uint8_t data_msg[MAX_DATA_MSG_SIZE], array_buffer[3], socket_no[2];
  if(build_nw_loc_msg(&msg_len, data_msg))
  {
    if(msg_len != 0)
    {
      if( xSemaphoreTake( xSemaphoreCellId, ( TickType_t ) TIME_500_MS ) == pdTRUE )
      {
        if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) TIME_500_MS ) == pdTRUE )
        {
          gsm_signal_and_signal.received_resp_cell_id = false;
          send_msg_count_cell_id = server_msg_count_id;
          xSemaphoreGive(xSemaphoreServerFlightMsgs);
        }
        
        if(send_udp_msg(data_msg, msg_len))
        {
          success = true;
        }
        xSemaphoreGive(xSemaphoreCellId);
      }
    }
  }
  return success;
}





