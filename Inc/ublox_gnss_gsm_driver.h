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
* File: ublox_gnss_gsm_driver.h
* Purpose: Driver for connecting to Ublox SARA G3 and CAM-M8Q modules and transmit data
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-05-29 Martin Skriver, Source written
****************************************************************************/
#ifndef _UBLOX_CTRL_TASK_H
#define _UBLOX_CTRL_TASK_H

/***************************************************************************/
/* Includes */
/***************************************************************************/

/***************************************************************************/
/* Global types and functions */
/***************************************************************************/

/***************************************************************************/
/* Global functions */
/***************************************************************************/
bool init_gsm_modem();
bool connect_socket();
bool setup_and_start_gnss();
bool reset_gsm_modem();
bool update_rssi();
bool set_netw_reg_loc_urc();
bool read_netw_reg_loc();
bool set_disp_operator();
bool send_udp_tracking_msg();
bool get_gnss_gga();
bool get_gnss_gsv();
bool send_udp_gpgsv();
bool send_udp_glgsv();
void build_at_string(uint8_t *array, uint8_t *command, ...);

bool send_gsm_nw_msg();
bool send_udp_start_tracking_msg();
bool send_udp_stop_msg(uint8_t stop_condition);
bool read_udp_no_of_msg_bytes();
bool power_off_gnss();

bool power_down_gsm_modem();
#endif /* _UBLOX_CTRL_TASK_H */
