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
* File: indicator_task.h
* Purpose: Shows status on a RGB LED
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-05-29 Martin Skriver, Source written
****************************************************************************/
#ifndef _INDICATOR_TASK_H
#define _INDICATOR_TASK_H

/***************************************************************************/
/* Includes */
/***************************************************************************/

/***************************************************************************/
/* Global types and functions */
/***************************************************************************/

//typedef enum{
//  NO_INDICATION_CNTR,
//  CONNECTING_CNTR,
//  CONNECTED_GPRS_CNTR,
//}cnt_task_indicator_t;

typedef enum{
  NO_INDICATION_PWR,
  BTN_RELEASE_SIGN_MGMT,
  BTN_SIGNAL_HIGH_PWR_MGMT,
  CPU_PWR_DOWN_PWR_MGMT
}pwr_task_indicator_t;

typedef enum{
  NO_INDICATION_CAP_RST,
  RESET_TIME_FRAME_CAP_RST,
  RESET_BTN_CAP_RST
}cap_rst_task_indicator_t;

typedef enum{
  NO_INDICATION_DATA_TRANS,
  GNSS_FIX_DATA_TRANS,
  NO_GNSS_FIX_DATA_TRANS
}data_transmit_indicator_t;

/***************************************************************************/
/* Global functions */
/***************************************************************************/
void indicator_main(void *pvParameters);

#endif /* _INDICATOR_TASK_H */
