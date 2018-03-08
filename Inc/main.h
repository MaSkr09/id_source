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
* File: main.h
* Purpose: DroneID main task
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-01-01 Martin Skriver, Source written
****************************************************************************/
#ifndef _MAIN_H
#define _MAIN_H

/***************************************************************************/
/* Includes */
/***************************************************************************/
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "ublox_reg.h" 
#include "nmea.h"
#include "indicator_task.h"
#include <stdbool.h>

/***************************************************************************/
/* Pwr management signals */
/***************************************************************************/
typedef enum {DRONEID_PWR_OFF, 
              DRONEID_PWR_OFF_LOW_BATT,
              DRONEID_PWR_ON
              } droneid_pwr_state_t;

/***************************************************************************/
/* Global defines */
/***************************************************************************/
#define	TASK_LOOP               for (;;)
#define TASK_STARTUP_DELAY_MS   100

#define APP_TRUE                (int8_t)0
#define APP_FALSE               (int8_t)-1

/***************************************************************************/
/* Global variables */
/***************************************************************************/
/* variables to define indication with LEDs */
//extern cnt_task_indicator_t cnt_task_indicator;
extern pwr_task_indicator_t pwr_task_indicator;
extern cap_rst_task_indicator_t cap_rst_task_indicator;
extern data_transmit_indicator_t data_transmit_indicator;
extern bool ctrl_power_on;
extern SemaphoreHandle_t xSemaphoreTaskIndicators;

/* Binary semaphore to shut down if low voltage */
extern SemaphoreHandle_t xSemaphoreBattLowV;

// int var magnetometer
extern bool newMagData;

/* Pwr management DroneID on/off */
extern droneid_pwr_state_t droneid_pwr_state;
extern SemaphoreHandle_t xSemaphorePwrMan;

/* Binary semaphore to shut down barometer */
extern SemaphoreHandle_t xSemaphoreBaroPwrIsDown;

/* Binary semaphore to shut down imu */
extern SemaphoreHandle_t xSemaphoreImuPwrIsDown;

extern bool enter_from_sleep;

/*Protected barometric data */
extern SemaphoreHandle_t xSemaphoreBarometricData;
extern float pressure_baro_var;
extern float temperature_baro_var;

/* Mutex to stop from pwr off when reset btn */
extern SemaphoreHandle_t xSemaphoreBtnCtrl;

/* I2C periph mutex and callback variable */
extern bool i2c_data_received;
extern SemaphoreHandle_t xSemaphoreI2CPeriph;

/* ublox tx data and semaphore */
extern SemaphoreHandle_t xSemaphoreUbloxTransmitState;

extern uint8_t ublox_trasnmit_complete;

/* Queue to add data msg */
extern QueueHandle_t xQueueSlipData;

/* Ublox mutex protected data register */
extern SemaphoreHandle_t xSemaphoreUbloxStatusReg;
extern ublox_reg_t ublox_status_reg;

/* Protected gga msg */
extern SemaphoreHandle_t xSemaphoreGgaMsg;
extern gpgga_t gga_data;

/* Protected gsv msg */
extern SemaphoreHandle_t xSemaphoreGsvMsg;
extern gpgsv_t gpgsv_data[4];
extern gpgsv_t glgsv_data[4];

/* Protected gsv msg */
//extern SemaphoreHandle_t xSemaphoreGsvMsg;
//extern gpgsv_t gsv_data[6];

/* Binary semaphore to shut down gsm and gnss modem before entering standby */
extern SemaphoreHandle_t xSemaphoreGsmPwrIsDown;

/*Protected msg ID */
//extern SemaphoreHandle_t xSemaphoreMsgId;
extern SemaphoreHandle_t xSemaphoreServerFlightMsgs;
extern uint8_t echo_msg_count_cell_id;
extern uint8_t echo_start_msg_count;
extern uint8_t echo_stop_msg_count;
extern uint8_t echo_cell_info_msg_count;
extern uint8_t send_msg_count_cell_id;
extern uint8_t server_msg_count_id;
//extern uint8_t server_echo_msg_count_id;
extern uint8_t server_msg_time_interval;
extern uint8_t new_server_msg_time_interval;
extern uint8_t max_horizontal_server_msg;
extern uint8_t max_vertical_server_msg;
extern uint8_t flight_permission;
extern uint8_t serv_cmd;


/* Protected cell id and area code */
extern SemaphoreHandle_t xSemaphoreCellId;
extern bool received_resp_cell_id;
extern uint16_t mob_country_code;
extern uint16_t mob_network_code;
extern uint16_t mob_location_area_code;
extern uint32_t mob_cell_id_code;
extern uint16_t pre_mob_location_area_code;
extern uint32_t pre_mob_cell_id_code;

// Timer to update gsm info
extern TimerHandle_t xHandleGSMInfoTimer;
/* Mutex to indicate timer expired */
extern SemaphoreHandle_t xSemaphoreGSMInfoTimerExpired;

// Timer to flash indicator leds
extern TimerHandle_t xHandleIndicatorGPTimer;
/* Mutex to indicate timer expired */
extern SemaphoreHandle_t xSemaphoreIndicatorTimerExpired;

// Timer to wait for gsm response
extern TimerHandle_t xHandleTransmitTimer;
/* Mutex to indicate timer expired */
extern SemaphoreHandle_t xSemaphoreTransmitPos;


// Timer to wait for ggs fetch
extern TimerHandle_t xHandleGetGgaTimer;
/* Mutex to indicate timer expired */
extern SemaphoreHandle_t xSemaphoreGetGgga;

// Timer to wait for transmit pos
extern TimerHandle_t xHandleVoltAdcGPTimer;
/* Mutex to indicate timer expired */
extern SemaphoreHandle_t xSemaphoreVoltAdcTimerExpired;

#ifdef TRANSMIT_GSV_DATA
/* Timer and binary semaphore to wait for transmit gsv */
// Timer to wait for transmit gsv
extern TimerHandle_t xHandleGsvGPTimer;
/* Mutex to indicate timer expired */
extern SemaphoreHandle_t xSemaphoreGsvTimerExpired;
#endif

/* Binary semaphore to indicate new ADC sample ready */
extern QueueHandle_t xQueueNewAdcSample;
extern uint16_t adc_data;
extern SemaphoreHandle_t xSemaphoreBattVoltage;
extern float batteryVoltage;
extern float initialBatteryVoltage;

/* Debug queue */
#if defined(DEBUG) || defined(DEBUG_BARO)
extern QueueHandle_t xQueueDebugQueue;
extern SemaphoreHandle_t xSemaphoreDebugMutex;
#endif


/***************************************************************************/
/* Buffers */
/***************************************************************************/
extern uint8_t uart3_rec_buffer[10];
extern uint8_t uart6_rec_buffer[10];

/***************************************************************************/
/* Global queue handles */
/***************************************************************************/
extern QueueHandle_t xQueueUbloxReceive;
extern QueueHandle_t xQueueAuxReceive;
extern QueueHandle_t xQueueUartTransmit;

/***************************************************************************/
/* Global task handles */
/***************************************************************************/
extern TaskHandle_t xHandlePwrManagementTask;
extern TaskHandle_t xHandleCapBtnResetTask;
extern TaskHandle_t xHandleUbloxDataReaderTask;
extern TaskHandle_t xHandleUartTransmitTask;
extern TaskHandle_t xHandleDroneidCtrlTask;
extern TaskHandle_t xHandleBatteryVoltageTask;
extern TaskHandle_t xHandleIndicatorTask;
extern TaskHandle_t xHandleBaroSensTask;
extern TaskHandle_t xHandleImuTask;
extern TaskHandle_t xHandlePosEstTask;
extern TaskHandle_t xHandleDebugTask;

/***************************************************************************/
/* Global functions */
/***************************************************************************/
extern uint8_t trace(uint16_t time_out_ms, const char* str, ...);

extern void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle);
extern void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle);

#endif /* _MAIN_H */
