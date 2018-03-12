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
* File: id_protocol_v1.h
* Purpose: Server msg read and build msgs 
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2018-03-12 Martin Skriver, Source written
****************************************************************************/

#ifndef _ID_PROTOCOL_V1_H
#define _ID_PROTOCOL_V1_H

/***************************************************************************/
/* Includes */
/***************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "nmea.h"
/***************************************************************************/
/* Global functions */
/***************************************************************************/
bool build_start_msg(uint16_t *lenght, uint8_t *data_msg);
bool build_stop_msg(uint16_t *lenght, uint8_t *data_msg, uint8_t stop_condition);
bool build_tracking_msg(uint16_t *lenght, uint8_t *data_msg);
bool build_gsv_msg(uint16_t *lenght, uint8_t *data_msg, gpgsv_t *gsv_data, uint8_t type);
bool build_nw_loc_msg(uint16_t *lenght, uint8_t *data_msg);

bool read_server_msg(uint8_t *str);

#endif /* _ID_PROTOCOL_V1_H */
