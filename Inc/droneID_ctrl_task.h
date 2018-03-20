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
#ifndef _DRONEID_CTRL_TASK_H
#define _DRONEID_CTRL_TASK_H

/***************************************************************************/
/* Includes */
/***************************************************************************/

/***************************************************************************/
/* Global types and functions */
/***************************************************************************/
/***************************************************************************/
typedef struct
{
  uint8_t error_cnt_read_loc;
  uint8_t error_cnt_send_loc;
  uint8_t error_cnt_read_gsv;
  uint8_t error_cnt_read_gga;
  uint8_t error_counter_send_track; 
  uint8_t error_cnt_get_rssi;
  uint8_t msg_count_cell_id;
  uint8_t cell_info_msg_count;
} transmit_error_counter_t;

/***************************************************************************/
/* Global functions */
/***************************************************************************/
void droneid_ctrl_main(void *pvParameters);
#endif /* _DRONEID_CTRL_TASK_H */
