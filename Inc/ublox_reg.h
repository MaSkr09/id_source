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
* File: ublox_reg.h
* Purpose: Static data for DroneID v2
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-05-29 Martin Skriver, Source written
****************************************************************************/
#ifndef _UBLOX_REG_H
#define _UBLOX_REG_H

/***************************************************************************/
/* Global types and structs */
/***************************************************************************/

typedef enum 
{
  NONE_RESULT_CODE,
  OK_RESULT_CODE,
  ERROR_RESULT_CODE
} ublox_result_code_t;

typedef struct
{
  ublox_result_code_t CPIN_RESULT_CODE;
  ublox_result_code_t DIS_ECHO_RESULT_CODE;

  ublox_result_code_t CGATT_RESULT_CODE;
  ublox_result_code_t GP_RESULT_CODE;
  ublox_result_code_t MODEM_READY_FOR_DATA;

  uint8_t CREATED_SOCKET_NO;
  ublox_result_code_t CREATED_SOCKET_NO_RESULT_CODE;
  uint8_t SERVER_IP[18];
  ublox_result_code_t SERVER_IP_RESULT_CODE;

  uint8_t SIGNAL_Q_VAR;

  ublox_result_code_t RECEIVED_GSM_PWR_DOWN_RESULT_CODE;

  uint16_t UDP_NO_OF_BYTES_TO_READ;
  uint16_t UDP_READING_NO_OF_BYTES;
} ublox_reg_t;

/***************************************************************************/
/* Global defines */
/***************************************************************************/
/* droneid server settings */
#define GMS_MSG_PERIOD_S              15

#define DRONE_ID_NUMBER              (uint32_t)12
#define PROTOCOL_VERSION              0
#define FIRMWARE_VERSION_NO           2.2
#define HW_VERSION_NO                 2.1

#define UDP_START_MSG_TYPE            1
#define UDP_STOP_MSG_TYPE             2
#define UDP_TRACK_MSG_TYPE            3
#define UDP_GPGSV_MSG_TYPE            4
#define UDP_GLGSV_MSG_TYPE            5
#define UDP_GSM_AREA_MSG_TYPE         6

#define DEBUG_TRANSMIT_TYPE           15

#define SERVER_FLIGHT_PER_REJECTED    0
#define SERVER_FLIGHT_PER_ACCEPTED    1
#define SERVER_NOT_CONNECTED          2

#define GNSS_MIN_SAT                  4


#define LOW_V_LIM                     3.15
#define LOW_V_CNT_LIM                 10

#define ERROR_CODE_USER_SHUT_DOWN   0
#define ERROR_CODE_SERVER_PWR_DOWN  1
#define ERROR_CODE_LOW_BATT_V       2

#define ERROR_CODE_ID_TRACK_POS     129
#define ERROR_CODE_GET_GSV          130
#define ERROR_CODE_GET_GGA          131
#define ERROR_CODE_GET_GSM_LOC      132
#define ERROR_CODE_SEND_GSM_LOC     133
#define ERROR_CODE_GSM_RSSI         134

#define ERROR_CODE_MISSING_RESP     140


#define AT_COM_START            "AT"
#define AT_PLUS                 "+"
#define AT_END_OF_MSG           "\r\n\000"
#define AT_EQUALS               "="
#define AT_READ                 "?"
#define AT_COMMA                ","

#define AT_OK_RESULT_CODE       "OK"
#define AT_ERROR_RESULT_CODE    "ERROR"

#define AT_CPIN_READY_CODE      "+CPIN: READY"
#define CPIN_CHECK              "CPIN"

#define AT_DISABLE_ECHO               "ATE0"
#define DISABLE_ECHO                  "E0\n"

#define GPRS_ATTACHED           "CGATT"
#define CGATT_SUCCES            "+CGATT: 1"


/* Packet switched data configuration +UPSD */
#define AT_PACK_SWITCH_CONFIG         "UPSD"
#define AT_PSD_PROFILE                "0"
#define AT_IPV4_PROTOCOL              "1"
#define AT_APN                        "\"internet\""
#define AT_IP_ADDR_PARA               "7"
#define AP_VAR_IP_ADDR                "\"0.0.0.0\""

/* Packet switched data action */
#define AT_PACK_SWITCH_DATA_ACTION    "UPSDA"
#define AT_PDP_ACTION                 "3"
//#define AT_PACK_SWITCH_ACT_SUCCESS    "UUPSDA: 0"

#define AT_CREATE_SOCKET              "USOCR"
#define AT_UDP_PROTOCOL               "17"

/* Resolve name / IP number through DNS +UDNSRN */
#define AT_RESOLVE_SERVER_IP          "UDNSRN"
#define AT_DOMAIN_NAME_TO_IP          "0"


#define AT_SEND_TO_SERVER             "USOST"

#define AT_READ_MSG_FROM_SERVER       "USORF"

#define AT_UNREAD_BYTES               "+UUSORF:"
#define AT_UNREAD_BYTES_D              "+UUSORD:"

/* GNSS pwr ctrl */
#define AT_GNSS_PWR_CMD               "UGPS"
#define AT_GNSS_PWR_OFF               "0"
#define AT_GNSS_PWR_ON                "1"
#define AT_ASSIST_LOCAL_AUTONOM_ONLINE  "7"
#define GNSS_SYSTEM_GLO_GPS           "67"

/* GNSS profile configuration */
#define AT_GNSS_PROFILE               "UGPRF"
#define AT_GNSS_PROFILE_NUMBER        "16"

//// Get GNSS time and date
//#define AT_GET_UGZDA                  "UGZDA"
// Get GNSS fix data
#define AT_GET_UGGGA                  "UGGGA"
//// Get geographic position
//#define AT_GET_UGGLL                  "UGGLL"
// Get number of GNSS satellites in view
#define AT_GET_UGGSV                  "UGGSV"
#define AT_GPGSV_START                "$GPGSV"
#define AT_GLGSV_START                "$GLGSV"
//// Get recommended minimum GNSS data
//#define AT_GET_UGRMC                  "UGRMC"
//// Get course over ground and ground speed
//#define AT_GET_UGVTG                  "UGVTG"
//// Get satellite information
//#define AT_GET_UGGSA                  "UGGSA"
#define AT_GNSS_ENABLE                "1"
//#define AT_GNSS_DISABLE               "0"

/* Assisted GNSS unsolicited indication */
#define AT_UNSOLICITED_GNSS_MSG       "UGIND"
#define AT_UNSOLICITED_EN             "1"

#define AT_AIDING_CONFIG              "UGSRV"
#define AT_AIDING_SERVER_1            "\"cell-live1.services.u-blox.com\""
#define AT_AIDING_SERVER_2            "\"cell-live2.services.u-blox.com\""
#define AT_VALID_DAYS                 "1"
#define AT_VALID_WEEKS_PERIOD         "5"
#define AT_RESOLUTION_DAYS            "1"
#define AT_GNSS_TYPE                  "65"
#define AT_ASSIST_OPERATING_MODE      "0"
#define AT_AIDING_DATA_TYPE           "15"
//
///* Operator selection */
//#define AT_OP_SELECT                  "ULOCCELL"
//#define AT_EXT_NETW_SEARCH            "1"
//
///* GNSS sensor configuration */
//#define AT_GNSS_SENSOR_CONF           "ULOCGNSS"
//#define AT_GNSS_AIDING_TYPE           "5" //"15"
//#define AT_GNSS_MIN_SAT               "4"
//
///* Localization information request status unsolicited indication */
//#define AT_AIDING_LOCALIZE_INFO       "ULOCIND"
//#define AT_LOCALIZE_INFO_EN           "1"
//
///* GNSS aiding request command */ 
//#define AT_AIDING_REQ                 "UGAOS"
//#define AT_AID_MODE_SAVE_LOCAL        "0"
//#define AT_AID_MODE_ONLINE            "4"

#define AT_QUALITY                    "CSQ"

/* GPRS network registration status */
#define AT_GPRS_NETWORK_REG_LOC       "+CGREG"
#define AT_EN_NETW_REG_URC            "2"

/* Set display operator name */
#define AT_DISP_OPR                   "+UDOPN"
#define AT_OPR_NUM_FORM               "0"

/* Power off gsm modem */
#define AT_POWER_OFF_GSM              "CPWROFF"

/* Enable HEX mode */
#define AT_HEX_MODE                   "+UDCONF=1"
#define AT_EN_HEX_MODE                "1"


#endif /* _UBLOX_REG_H */
