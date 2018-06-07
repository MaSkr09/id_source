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
* File: id_config.h
* Purpose: Static data for DroneID v2
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2018-03-07 Martin Skriver, Source written
****************************************************************************/
#ifndef _ID_CONFIG_H
#define _ID_CONFIG_H

/***************************************************************************/
/* Global defines */
/***************************************************************************/
/* DroneID ID number */
#define DRONE_ID_NUMBER              (uint32_t)505
//#define DRONE_ID_NUMBER              (uint32_t)1005

/* droneid server settings */
#define GMS_MSG_PERIOD_S              15

/* Protocol */
#define PROTOCOL_VERSION              0
#define FIRMWARE_VERSION_NO           2.2
#define HW_VERSION_NO                 2.1

#define UDP_START_MSG_TYPE            1
#define UDP_STOP_MSG_TYPE             2
#define UDP_TRACK_MSG_TYPE            3
#define UDP_GPGSV_MSG_TYPE            4
#define UDP_GLGSV_MSG_TYPE            5
#define UDP_GSM_AREA_MSG_TYPE         6
#define UDP_SERVER_MSG_TYPE           7

#define REG_KILL_DRONEID              0


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

/***************************************************************************/
#endif /* _ID_CONFIG_H */
