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
* File: imu_task.c
* Purpose: Setup MPU9250 and read data
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-01-05 Martin Skriver, Source written
****************************************************************************/
#include "main.h"
#include "gpio.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"

#include "imu_task.h"

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};

static struct hal_s hal = {0};

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}

void imu_main(void *pvParameters)
{
  vTaskDelay(100 / portTICK_RATE_MS);
  bool pwr_on = false;


/*********************** Delete this for using IMU *****************/
    mpu_sleep();
    xSemaphoreGive( xSemaphoreImuPwrIsDown );
    // The task suspends itself.
    vTaskDelay(1000 / portTICK_RATE_MS);
    vTaskSuspend( NULL ); // until implemented right
/********************************************************************/









  /* Wait until powered up */
  while(!pwr_on)
  {
    if( xSemaphoreTake( xSemaphorePwrMan, ( TickType_t ) 1000 ) == pdTRUE )
    {
      if(droneid_pwr_state == DRONEID_PWR_ON)
        pwr_on = true;
      xSemaphoreGive( xSemaphorePwrMan );
    }
    vTaskDelay(200 / portTICK_RATE_MS);
  }

  HAL_GPIO_WritePin(GPIO_MPU_9250_FSYNC_O_GPIO_Port, GPIO_MPU_9250_FSYNC_O_Pin, GPIO_PIN_RESET);

//  inv_error_t result;
//    unsigned char accel_fsr,  new_temp = 0;
//    unsigned short gyro_rate, gyro_fsr;
    struct int_param_s int_param;

    int_param.cb = NULL;

  if( xSemaphoreTake( xSemaphoreI2CPeriph, ( TickType_t ) 500 ) == pdTRUE )
  {
    mpu_init(&int_param);
    xSemaphoreGive( xSemaphoreI2CPeriph );
  }

  TASK_LOOP
  {
    if( xSemaphoreTake( xSemaphorePwrMan, ( TickType_t ) 5 ) == pdTRUE )
    {
      if(droneid_pwr_state == DRONEID_PWR_OFF)
        pwr_on = false;
      xSemaphoreGive( xSemaphorePwrMan );
    }

    if(!pwr_on)
    {
      mpu_sleep();
      xSemaphoreGive( xSemaphoreImuPwrIsDown );
      // The task suspends itself.
      vTaskDelay(1000 / portTICK_RATE_MS);
      vTaskSuspend( NULL ); // until implemented right
    }
    else if(newMagData)
    {
      if( xSemaphoreTake( xSemaphoreI2CPeriph, ( TickType_t ) 500 ) == pdTRUE )
      {
        read_imu_data(); 
        xSemaphoreGive( xSemaphoreI2CPeriph );
      }
    }
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}
