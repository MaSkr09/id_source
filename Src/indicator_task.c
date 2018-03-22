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
* File: indicator_task.c
* Purpose: Shows status on a RGB LED
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-05-29 Martin Skriver, Source written
****************************************************************************/
#include "tim.h"
#include "main.h"
#include "indicator_task.h"
#include "id_config.h"

#define LOOP_SLEEP_TIME_MS      10
#define DELAY_500_MS            500

typedef enum 
{
  NO_INDICATION,
  DRONEID_CONNECT,
  DRONEID_PWR_DOWN,
  BTN_REBOOT_RELEASE_TIME_FRAME,
  FLIGHT_ACCEPTED,
  FLIGHT_REJECTED,
  MISSING_GNSS,
  BTN_PUSHED,
  BTN_RELEASE,
  RESET_BTN_TIME_FRAME,
  RESET_BTN_IN_PROGRESS
} indicator_signals_t;

indicator_signals_t get_highest_prio_signal(void)
{
  indicator_signals_t return_indication = NO_INDICATION;

  if( xSemaphoreTake( xSemaphoreTaskIndicators, ( TickType_t ) DELAY_500_MS ) == pdTRUE )
  {
    /* Highest priority */    
    /* Btn reset time frame */
    if(pwr_task_indicator == CPU_PWR_DOWN_PWR_MGMT)
    {
      return_indication = DRONEID_PWR_DOWN;
    }
    /* Btn reset time frame */
    else if(cap_rst_task_indicator == RESET_TIME_FRAME_CAP_RST)
    {
      return_indication = RESET_BTN_TIME_FRAME;
    }
    /* Release for reboot */
    else if(pwr_task_indicator == BTN_REBOOT_RELEASE_SIGN_MGMT)
    {
      return_indication = BTN_REBOOT_RELEASE_TIME_FRAME;
    }
    /* Reset btn in progress */
    else if(cap_rst_task_indicator == RESET_BTN_CAP_RST)
    {
      return_indication = RESET_BTN_IN_PROGRESS;
    }
    /* Btn pressed */
    else if(pwr_task_indicator == BTN_SIGNAL_HIGH_PWR_MGMT)
    {
        return_indication = BTN_PUSHED;
    }

    /* Btn release time */
    else if(pwr_task_indicator == BTN_RELEASE_SIGN_MGMT)
    {
        return_indication = BTN_RELEASE;
    }
    /* Connecting GPRS */
    else if(ctrl_power_on)
    {
      if( xSemaphoreTake( xSemaphoreServerFlightMsgs, ( TickType_t ) DELAY_500_MS ) == pdTRUE )
      {
        if(flight_permission == SERVER_FLIGHT_PER_ACCEPTED)
        {
          return_indication = FLIGHT_ACCEPTED;
        }
        else if(flight_permission == SERVER_FLIGHT_PER_REJECTED)
        {
          return_indication = FLIGHT_REJECTED;
          if( xSemaphoreTake( xSemaphoreGgaMsg, ( TickType_t ) DELAY_500_MS ) == pdTRUE )
          {
            if(gga_data.sat < 3)
            {
              return_indication = MISSING_GNSS;
            }
            xSemaphoreGive( xSemaphoreGgaMsg );
          }
        }
        else if(flight_permission == SERVER_NOT_CONNECTED)
        {
          return_indication = DRONEID_CONNECT;
        }
        xSemaphoreGive(xSemaphoreServerFlightMsgs);
      }
    }
    else
    {
      return_indication = DRONEID_PWR_DOWN;
    }
  xSemaphoreGive( xSemaphoreTaskIndicators );
  }
  return return_indication;
}

/***************************************************************************/
/* Set LED duty */
/***************************************************************************/
void set_led_duty(uint16_t red_led_duty, uint16_t green_led_duty, uint16_t blue_led_duty)
{
  htim2.Instance->CCR2 = red_led_duty;
  htim2.Instance->CCR3 = green_led_duty;
  htim2.Instance->CCR4 = blue_led_duty;
}
/***************************************************************************/
/* Main indicator task */
/***************************************************************************/
void indicator_main(void *pvParameters)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  uint16_t pwm_duty = 0;

  /* Start software timer */
  xTimerStart( xHandleIndicatorGPTimer, 10000 );
  bool indicator_high = false;
  uint8_t cnt = 0;

  indicator_signals_t pre_indicator_signal, nx_indicator_signal;
  TASK_LOOP
  {
    nx_indicator_signal = get_highest_prio_signal();
    switch(nx_indicator_signal)
    {
      case DRONEID_PWR_DOWN:
        if(pre_indicator_signal != nx_indicator_signal)
        {
          xSemaphoreTake( xSemaphoreIndicatorTimerExpired, ( TickType_t ) 0);
          xTimerChangePeriod(xHandleIndicatorGPTimer, 1, 1000 );
          xTimerReset( xHandleIndicatorGPTimer, 100);
        }

        if(xSemaphoreTake( xSemaphoreIndicatorTimerExpired, ( TickType_t ) 0) == pdTRUE)
        {
          if(indicator_high)
          {
            indicator_high = false;
            set_led_duty(0xFFFF, 0xFFFF, 0x0000);
            xTimerChangePeriod(xHandleIndicatorGPTimer, 50, 100 );
            xTimerReset( xHandleIndicatorGPTimer, 100);
          }
          else
          {
            indicator_high = true;
            set_led_duty(0x0000, 0x0000, 0x0000);
            xTimerChangePeriod(xHandleIndicatorGPTimer, 1500, 100 );
            xTimerReset( xHandleIndicatorGPTimer, 100);
          }
        }
      break;

      case RESET_BTN_TIME_FRAME:
        htim2.Instance->CCR2 = 0xFFFF;
        htim2.Instance->CCR3 = 0xFFFF;
        htim2.Instance->CCR4 = 0x0000;
      break;

      case BTN_REBOOT_RELEASE_TIME_FRAME:
        if(pre_indicator_signal != nx_indicator_signal)
        {
          xSemaphoreTake( xSemaphoreIndicatorTimerExpired, ( TickType_t ) 0);
          xTimerChangePeriod(xHandleIndicatorGPTimer, 1, 200 );
          xTimerReset( xHandleIndicatorGPTimer, 100);
          cnt = 0;
        }

        if(xSemaphoreTake( xSemaphoreIndicatorTimerExpired, ( TickType_t ) 0) == pdTRUE)
        {
          if(cnt == 0)
          {
            cnt = 1;
            set_led_duty(0xFFFF, 0x0000, 0x0000);
          }
          else if(cnt == 1)
          {
            cnt = 2;
            set_led_duty(0x0000, 0xFFFF, 0x0000);
          }
          else
          {
            cnt = 0;
            set_led_duty(0x0000, 0x0000, 0xFFFF);
          }
        }
      break;

      case RESET_BTN_IN_PROGRESS:
        htim2.Instance->CCR2 = 0xFFFF;
        htim2.Instance->CCR3 = 0xFFFF;
        htim2.Instance->CCR4 = 0xFFFF;
      break;

      case BTN_PUSHED:
        set_led_duty(0x0000, 0x0000, 0xFFFF);
      break;

      case BTN_RELEASE:
        set_led_duty(0x5FFF, 0xFFFF, 0x5FFF);
      break;

      case DRONEID_CONNECT:
        if(pre_indicator_signal != nx_indicator_signal)
        {
          xSemaphoreTake( xSemaphoreIndicatorTimerExpired, ( TickType_t ) 0);
          xTimerChangePeriod(xHandleIndicatorGPTimer, 1, 100 );
          xTimerReset( xHandleIndicatorGPTimer, 100);
        }

        if(xSemaphoreTake( xSemaphoreIndicatorTimerExpired, ( TickType_t ) 0) == pdTRUE)
        {
          if(indicator_high)
          {
            indicator_high = false;
            set_led_duty(0xFFFF, 0x0000, 0x0000);
            xTimerChangePeriod(xHandleIndicatorGPTimer, 50, 100 );
            xTimerReset( xHandleIndicatorGPTimer, 100);
          }
          else
          {
            indicator_high = true;
            set_led_duty(0x0000, 0x0000, 0x0000);
            xTimerChangePeriod(xHandleIndicatorGPTimer, 1500, 100 );
            xTimerReset( xHandleIndicatorGPTimer, 100);
          }
        }
      break;

      case FLIGHT_ACCEPTED:
        if(pre_indicator_signal != nx_indicator_signal)
        {
          pwm_duty = 0;
          xSemaphoreTake( xSemaphoreIndicatorTimerExpired, ( TickType_t ) 0);
          xTimerChangePeriod(xHandleIndicatorGPTimer, 1, 100 );
          xTimerReset( xHandleIndicatorGPTimer, 100);
        }

        if(xSemaphoreTake( xSemaphoreIndicatorTimerExpired, ( TickType_t ) 0) == pdTRUE)
        {
          if(indicator_high)
          {
            if(pwm_duty < 63000)
            {
              pwm_duty += 3000;
              set_led_duty(0x0000, pwm_duty, 0x0000);
              xTimerChangePeriod(xHandleIndicatorGPTimer, 10, 100 );
              xTimerReset( xHandleIndicatorGPTimer, 100);
            }
            else
            {
              pwm_duty = 0;
              set_led_duty(0x5FFF, 0xFFFF, 0x5FFF);
              xTimerChangePeriod(xHandleIndicatorGPTimer, 50, 100 );
              xTimerReset( xHandleIndicatorGPTimer, 100);              
              indicator_high = false;
            }
          }
          else
          {
            indicator_high = true;
            set_led_duty(0x0000, 0x0000, 0x0000);
            xTimerChangePeriod(xHandleIndicatorGPTimer, 1500, 100 );
            xTimerReset( xHandleIndicatorGPTimer, 100);
          }
        }
      break;

      case FLIGHT_REJECTED:
        if(pre_indicator_signal != nx_indicator_signal)
        {
          pwm_duty = 0;
          xSemaphoreTake( xSemaphoreIndicatorTimerExpired, ( TickType_t ) 0);
          xTimerChangePeriod(xHandleIndicatorGPTimer, 1, 100 );
          xTimerReset( xHandleIndicatorGPTimer, 100);
        }

        if(xSemaphoreTake( xSemaphoreIndicatorTimerExpired, ( TickType_t ) 0) == pdTRUE)
        {
          if(indicator_high)
          {
            if(pwm_duty < 63000)
            {
              pwm_duty += 3000;
              set_led_duty(pwm_duty, 0x0000, 0x0000);
              xTimerChangePeriod(xHandleIndicatorGPTimer, 10, 100 );
              xTimerReset( xHandleIndicatorGPTimer, 100);
            }
            else
            {
              set_led_duty(0x5FFF, 0xFFFF, 0x5FFF);
              xTimerChangePeriod(xHandleIndicatorGPTimer, 50, 100 );
              xTimerReset( xHandleIndicatorGPTimer, 100);              
              indicator_high = false;
            }
          }
          else
          {
            indicator_high = true;
            set_led_duty(0x0000, 0x0000, 0x0000);
            xTimerChangePeriod(xHandleIndicatorGPTimer, 1500, 100 );
            xTimerReset( xHandleIndicatorGPTimer, 100);
          }
        }
      break;

      case MISSING_GNSS:
        if(pre_indicator_signal != nx_indicator_signal)
        {
          pwm_duty = 0;
          xSemaphoreTake( xSemaphoreIndicatorTimerExpired, ( TickType_t ) 0);
          xTimerChangePeriod(xHandleIndicatorGPTimer, 1, 100 );
          xTimerReset( xHandleIndicatorGPTimer, 100);
        }

        if(xSemaphoreTake( xSemaphoreIndicatorTimerExpired, ( TickType_t ) 0) == pdTRUE)
        {
          if(indicator_high)
          {
            if(pwm_duty < 63000)
            {
              pwm_duty += 1500;
              set_led_duty(pwm_duty, 0x0000, 0x0000);
              xTimerChangePeriod(xHandleIndicatorGPTimer, 10, 100 );
              xTimerReset( xHandleIndicatorGPTimer, 100);
            }
            else
            {
              pwm_duty = 0;
              set_led_duty(0x5FFF, 0xFFFF, 0x5FFF);
              xTimerChangePeriod(xHandleIndicatorGPTimer, 50, 100 );
              xTimerReset( xHandleIndicatorGPTimer, 100);              
              indicator_high = false;
            }
          }
          else
          {
            indicator_high = true;
            set_led_duty(0x0000, 0x0000, 0x0000);
            xTimerChangePeriod(xHandleIndicatorGPTimer, 2000, 100 );
            xTimerReset( xHandleIndicatorGPTimer, 100);
          }
        }
      break;

      case NO_INDICATION:
        htim2.Instance->CCR2 = 0x0000;
        htim2.Instance->CCR3 = 0x0000;
        htim2.Instance->CCR4 = 0x0000;
      break;

      default:
        break;
    }
    pre_indicator_signal = nx_indicator_signal;
    vTaskDelay(LOOP_SLEEP_TIME_MS/ portTICK_RATE_MS);
  }
}

