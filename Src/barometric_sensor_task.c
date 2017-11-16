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
* File: barometric_sensor_task.c
* Purpose: Setup sensor and fetch data
* Project: DroneID v2
* Author: Martin Skriver <MaSkr@mmmi.sdu.dk> & <MaSkr09@gmail.com>
* ****************************************************************************
* Log:
* Created:  2017-03-20 Martin Skriver, Source written
****************************************************************************/
#include "i2c.h"
#include "FreeRTOS.h"
#include "main.h"
#include "debug_task.h"
#include <stdio.h>
#include <math.h>

#include "barometric_sensor_task.h"

/***************************************************************************/
/* Global variable */
/***************************************************************************/
uint8_t i2c_tx_buffer[10];
uint8_t i2c_rx_buffer[10];

/***************************************************************************/
/* Barometric sensor defines and variables */
/***************************************************************************/
uint16_t barometric_calibration_data[7];

#define BAROMETRIC_SENSOR_ADDR      0b11101100

#define BAROMETRIC_RESET_ADDR       0b00011110
#define I2C_TRANSMIT_TIMEOUT_MS     10

#define BAROMETRIC_PROM_START_ADDR  0b10100000
#define BAROMETRIC_PROM_LAST_ADDR   7
#define BAROMETRIC_PROM_TIMEOUT_MS  100

// Setup Conversion 
#define BAROMETRIC_ADC_READ         0b00000000
#define ADC_PRESSURE_RATIO_8192 
#define ADC_TEMP_RATIO_8192

#ifdef ADC_PRESSURE_RATIO_256
/* over-sampling ratio 256 for D1(pressure) and D2(temperature) */
#define BAROMETRIC_PRESS_OS         0b01000000
#define PRESS_CONVENTION_TIME_MS    1

#elif ADC_PRESSURE_RATIO_512
/* over-sampling ratio 512 for D1(pressure) and D2(temperature) */
#define BAROMETRIC_PRESS_OS         0b01000010
#define PRESS_CONVENTION_TIME_MS    2

#elif ADC_PRESSURE_RATIO_1024
/* over-sampling ratio 1024 for D1(pressure) and D2(temperature) */
#define BAROMETRIC_PRESS_OS         0b01000100
#define PRESS_CONVENTION_TIME_MS    3

#elif ADC_PRESSURE_RATIO_2048
/* over-sampling ratio 2048 for D1(pressure) and D2(temperature) */
#define BAROMETRIC_PRESS_OS         0b01000110
#define PRESS_CONVENTION_TIME_MS    5

#elif ADC_PRESSURE_RATIO_4096
/* over-sampling ratio 4096 for D1(pressure) and D2(temperature) */
#define BAROMETRIC_PRESS_OS         0b01001000
#define PRESS_CONVENTION_TIME_MS    10

#else //ADC_PRESSURE_RATIO_8192
/* over-sampling ratio 8192 for D1(pressure) and D2(temperature) */
#define BAROMETRIC_PRESS_OS         0b01001010
#define PRESS_CONVENTION_TIME_MS    18
#endif


#ifdef ADC_TEMP_RATIO_256
/* over-sampling ratio 256 for D1(pressure) and D2(temperature) */
#define BAROMETRIC_TEMP_OS          0b01010000
#define TEMP_CONVENTION_TIME_MS     1

#elif ADC_TEMP_RATIO_512
/* over-sampling ratio 512 for D1(pressure) and D2(temperature) */
#define BAROMETRIC_TEMP_OS          0b01010010
#define TEMP_CONVENTION_TIME_MS     2

#elif ADC_TEMP_RATIO_1024
/* over-sampling ratio 1024 for D1(pressure) and D2(temperature) */
#define BAROMETRIC_TEMP_OS          0b01010100
#define TEMP_CONVENTION_TIME_MS     3

#elif ADC_TEMP_RATIO_2048
/* over-sampling ratio 2048 for D1(pressure) and D2(temperature) */
#define BAROMETRIC_TEMP_OS          0b01010110
#define TEMP_CONVENTION_TIME_MS     5

#elif ADC_TEMP_RATIO_4096
/* over-sampling ratio 4096 for D1(pressure) and D2(temperature) */
#define BAROMETRIC_TEMP_OS          0b01011000
#define TEMP_CONVENTION_TIME_MS     10

#else //ADC_TEMP_RATIO_8192
/* over-sampling ratio 8192 for D1(pressure) and D2(temperature) */
#define BAROMETRIC_TEMP_OS          0b01011010
#define TEMP_CONVENTION_TIME_MS     18
#endif

#define DT_MIN                      -16776960
#define DT_MAX                      16777216

#define TEMP_MIN                    -4000
#define TEMP_MAX                    8500

#define OFFSET_MIN                  -17179344900
#define OFFSET_MAX                  25769410560

#define SENSITIVITY_MIN             -8589672450
#define SENSITIVITY_MAX             12884705280

#define PRESSURE_COMP_MIN           1000
#define PRESSURE_COMP_MAX           120000

#define HIGH_LOW_TEMPERATURE_LIM      2000
#define LOW_VERY_LOW_TEMPERATURE_LIM  -1500

/***************************************************************************/
/* Write to barometric sensor */
/***************************************************************************/
bool write_to_barometric_sensor(uint8_t *write_addr)
{
  bool succeeded = true;
  uint8_t timer = 0;
  /* Send reset to barometric sensor */
  // TODO add timeout
  
  if(HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)BAROMETRIC_SENSOR_ADDR, write_addr, sizeof(uint8_t)) != HAL_OK)
    succeeded = false;

  /* Wait transfer to complete */
  // TODO add timeout
  while((HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) && (I2C_TRANSMIT_TIMEOUT_MS > timer))
  {
    vTaskDelay(1);
    timer++;
  }

  if(!(I2C_TRANSMIT_TIMEOUT_MS > timer))
    succeeded &= false;    

  return succeeded;
}

/***************************************************************************/
/* Reset barometric sensor */
/***************************************************************************/
bool reset_barometric_sensor(void)
{
  bool succeeded;
  uint8_t reset_addr = BAROMETRIC_RESET_ADDR;
  succeeded = write_to_barometric_sensor(&reset_addr);

  return succeeded;
}

/***************************************************************************/
/* Read PROM calibration data */
/***************************************************************************/
bool read_barometric_calibration_data(void)
{
  bool succeeded = true;

  uint8_t i;
  uint16_t timer;
  uint8_t prom_cmd = BAROMETRIC_PROM_START_ADDR;
  HAL_StatusTypeDef status;


  for(i=0; i < BAROMETRIC_PROM_LAST_ADDR; ++i)
  {
    timer = 0;
    /* Setup addr to read */
    prom_cmd = BAROMETRIC_PROM_START_ADDR | (i<<1);
    
    succeeded = write_to_barometric_sensor(&prom_cmd);

    /* Set I2C in reception mode */
    status = HAL_I2C_Master_Receive_DMA(&hi2c1, (uint8_t)BAROMETRIC_SENSOR_ADDR, (uint8_t *)i2c_rx_buffer, 2);

    /* Wait for data via DMA */
    while(!(i2c_data_received) && (timer < BAROMETRIC_PROM_TIMEOUT_MS))
    {
      timer++;
      vTaskDelay(1);
    }

    /* If data received, save it */
    if(i2c_data_received)
    {
      barometric_calibration_data[i] = (uint16_t) (((int16_t)i2c_rx_buffer[0] << 8) | (int16_t)i2c_rx_buffer[1]);
      i2c_data_received = false;
      succeeded &= true;
    }
    /* Else return error bit */
    else
    {
      succeeded = false;
    }
  }
  return succeeded;
}


/***************************************************************************/
/* Read adc from barometric sensor */
/***************************************************************************/
bool read_barometric_adc(uint8_t * osr, uint32_t delay, uint32_t * return_data)
{
  bool succeeded = true;

  uint16_t timer;
  timer = 0;

  uint8_t start = BAROMETRIC_ADC_READ;

  /* Set ADC to sensor and sample ratio */
  succeeded = write_to_barometric_sensor(osr);

  /* Read ADC delay */
  vTaskDelay(delay);

  /* Initiate ADC seq */
  succeeded &= write_to_barometric_sensor(&start);

  /* Set I2C in reception mode */
  if( HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t)BAROMETRIC_SENSOR_ADDR, (uint8_t*) i2c_rx_buffer, 3) != HAL_OK)
    succeeded = false;

  /* Wait for data via DMA */
  while(!(i2c_data_received) && (timer < BAROMETRIC_PROM_TIMEOUT_MS))
  {
    timer++;
    vTaskDelay(1);
  }

  /* If data received, save it */
  if(i2c_data_received)
  {
    i2c_data_received = false;
    succeeded &= true;
    *return_data = ((i2c_rx_buffer[0] << 16) | (i2c_rx_buffer[1] << 8) | i2c_rx_buffer[2]);
  }
  /* Else return error bit */
  else
  {
    succeeded = false;
  }
  return succeeded;
}


/***************************************************************************/
/* Calculate temperature compensated pressure */
/***************************************************************************/
bool calc_2_order_temp_compensation(int32_t *temperature, int32_t dT, int64_t *offset, int64_t *sens )
{
  bool succeeded = true;

  int32_t t2;
  int64_t off2;
  int64_t sens2;

  /* Low temperature */
  if(*temperature < HIGH_LOW_TEMPERATURE_LIM)
  {
    t2 = (int32_t)((((int64_t)3*(int64_t)dT*(int64_t)dT))>>33);
    off2 = ((int64_t)61*(((int64_t)(*temperature)-2000)*((int64_t)(*temperature)-2000))>>4);
    sens2 = ((int64_t)29*(((int64_t)(*temperature)-2000)*((int64_t)(*temperature)-2000))>>4);
    /*Very low temperature */
    if(*temperature < HIGH_LOW_TEMPERATURE_LIM)
    {
      off2 = off2 + (int64_t)17*(((int64_t)(*temperature)+1500)*((int64_t)(*temperature)+1500));
      sens2 = sens2 + (int64_t)9*(((int64_t)(*temperature)+1500)*((int64_t)(*temperature)+1500));
    }
  }
  /* High temperature */
  else
  {
    t2 = (int32_t)((((int64_t)5*(int64_t)dT*(int64_t)dT))>>38);
    off2 = 0;
    sens2 = 0;
  }

  *temperature = *temperature-t2;
  *offset = *offset-off2;
  *sens = *sens-sens2;

  return succeeded;
}

/***************************************************************************/
/* Calculate pressure and temperature */
// TODO change to use FPU and pressure has a big offset but works. Uncomment pressure check when it works
/***************************************************************************/
bool temperature_presure_calculation(int32_t *pressure, uint32_t pressure_raw, int32_t *temperature, uint32_t temperature_raw)
{
  bool succeeded = true;
  int32_t dT_var;

  int64_t offset;
  int64_t sens_temp;

  int64_t temperature_buf;

  /* Calculate different between actual and ref temperature */
  dT_var = temperature_raw - ((int32_t)barometric_calibration_data[5] << 8);
//  dT_var = temperature_raw - ((int32_t)barometric_calibration_data[5] * (int32_t)pow(2,8));

  /* Calculate actual linear temperature */ 
  *temperature = (int32_t)((int64_t)2000+(((int64_t)dT_var*(int64_t)barometric_calibration_data[6])>>23));
//  *temperature = (int64_t)((int64_t)2000+(((int64_t)dT_var*(int64_t)barometric_calibration_data[6])))/(int64_t)pow(2,23);

  /* Calculate initial offset */
//  offset = (int64_t)(((int64_t)barometric_calibration_data[2]<<17)+((int64_t)barometric_calibration_data[4]*(int64_t)dT_var)>>6);
  offset = (int64_t)((int64_t)barometric_calibration_data[2]*(int64_t)pow(2,17)+((int64_t)barometric_calibration_data[4]*(int64_t)dT_var)/(int64_t)pow(2,6));

  /* Calculate initial Sensitivity */
  sens_temp = ((int64_t)barometric_calibration_data[1]<<16) + (int64_t)(((int64_t)barometric_calibration_data[3]*(int64_t)dT_var)>>7);

  /* Get temperature compensated offset, sensitivity and temperature */
//  calc_2_order_temp_compensation(temperature, dT_var, &offset, &sens_temp);

  /* Calculate temperature compensated pressure */
  *pressure = (int32_t)((((int64_t)pressure_raw*sens_temp>>21)-offset)>>15);

  /* Check if values are within range is in range */ 

  if((dT_var > DT_MIN) && (dT_var < DT_MAX) &&
    (TEMP_MIN < *temperature) && (TEMP_MAX > *temperature) &&
    (OFFSET_MIN < offset) && (OFFSET_MAX > offset) &&
    (SENSITIVITY_MIN < sens_temp) && (SENSITIVITY_MAX > sens_temp) //&&
//    (PRESSURE_COMP_MIN < *pressure) && (PRESSURE_COMP_MAX > *pressure)
    )
  {
    if( xSemaphoreTake( xSemaphoreBarometricData, ( TickType_t ) 500 ) == pdTRUE )
    {
      pressure_baro_var = (float)*pressure/100;
      temperature_baro_var = (float)*temperature/100;
      xSemaphoreGive( xSemaphoreBarometricData );
    }
//      debug_printf("Presure: %d \t Temperature: %d\n", *pressure, *temperature );
  }
  else
  {
    succeeded = false;
  }

  return succeeded;

}

/***************************************************************************/
/* Read sensor loop */
/***************************************************************************/
bool imu_main_func(void)
{
  bool succeeded = true;

  uint8_t addr;
  uint8_t convention_delay;

  uint32_t raw_pressure_buffer;
  uint32_t raw_temperature_buffer;
  int32_t calculated_pressure;
  int32_t calculated_temperature;

//  while(succeeded)
//  {
  /* Read temperature from sensor*/
  addr = BAROMETRIC_TEMP_OS;
  convention_delay = TEMP_CONVENTION_TIME_MS;
  succeeded &= read_barometric_adc( &addr, TEMP_CONVENTION_TIME_MS, &raw_temperature_buffer );
  /* Read pressure from sensor*/
  addr = BAROMETRIC_PRESS_OS;
  convention_delay = PRESS_CONVENTION_TIME_MS;
  succeeded &= read_barometric_adc( &addr, convention_delay, &raw_pressure_buffer );

  succeeded &= temperature_presure_calculation(&calculated_pressure, raw_pressure_buffer, &calculated_temperature, raw_temperature_buffer);

  return succeeded;
}

/***************************************************************************/
/* Reset barometric sensor */
/***************************************************************************/
bool reset_baro_sens(void)
{
  bool success = false;
  if( xSemaphoreTake( xSemaphoreI2CPeriph, ( TickType_t ) 500 ) == pdTRUE )
  {
    if(reset_barometric_sensor())
      if(read_barometric_calibration_data())
        success = true;
    xSemaphoreGive( xSemaphoreI2CPeriph );
  }
  return success;
}

/***************************************************************************/
/* Barometric sensor main */
/***************************************************************************/
void barometric_sens_main(void *pvParameters)
{
/*********************** Delete this for using BARO *****************/
  vTaskSuspend( NULL ); // until implemented right
/********************************************************************/


  vTaskDelay(100 / portTICK_RATE_MS);
  bool no_error = true;
  char output[32];
  
  while(!reset_baro_sens())
    vTaskDelay(10);

  TASK_LOOP
  {
    if( xSemaphoreTake( xSemaphoreI2CPeriph, ( TickType_t ) 500 ) == pdTRUE )
    {
      if(!imu_main_func())
      {
        xSemaphoreGive( xSemaphoreI2CPeriph );
        reset_baro_sens();
      }
      xSemaphoreGive( xSemaphoreI2CPeriph );

#ifdef DEBUG_BARO
      if( xSemaphoreTake( xSemaphoreBarometricData, ( TickType_t ) 500 ) == pdTRUE )
      {
        sprintf(output, "%f", pressure_baro_var);
        xSemaphoreGive( xSemaphoreBarometricData );
        debug_add_to_queue(output);
        debug_add_to_queue("\n");
      }
#endif
    }
    vTaskDelay(10);
  }
}
