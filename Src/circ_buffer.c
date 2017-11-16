/****************************************************************************
 # Circular buffer
 # Copyright (c) 2016 Martin Skriver <maskr09@gmail.com>
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
 #
 *****************************************************************************
 # File: circ_buffer.c
 # Purpose: Circular buffer
 # Project: DroneID
 # Author: Martin Skriver <maskr09@gmail.com>
 # Created:  2016-04-29 Martin Skriver, Source written
 ****************************************************************************/
/* system includes */
#include <string.h>
#include <stdio.h>

/***************************************************************************/
/* application includes */
#include "circ_buffer.h"

/***************************************************************************/
/* #defines */

/***************************************************************************/
/* function prototypes */

/***************************************************************************/
/* Set pointers to beginning and set status to empty */
void init_ring_buffer(circ_buffer_t *buffer)
{
  buffer->head = 0;
  buffer->tail = buffer->head;
  buffer->status = RING_BUFFER_EMPTY;
}

/***************************************************************************/
/* Update status for buffer */
void update_circ_buffer_status(circ_buffer_t *buffer)
{
  // Check if buffer is empty
  if(buffer->tail == buffer->head)
    buffer->status = RING_BUFFER_EMPTY;
  // Check if buffer in full
  else if(buffer->tail == buffer->head+1)
    buffer->status = RING_BUFFER_FULL;
  // If passing buffer size
  else if((buffer->head+1 == RING_BUFFER_SIZE) && (0 == buffer->tail))
  {
    buffer->status = RING_BUFFER_FULL;
  }
  // Must be data in buffer
  else
    buffer->status = RING_BUFFER_CONT_DATA;
}
/***************************************************************************/
/* Save a copy of instance in buffer.
 * Return false if buffer is full else true
*/
bool push_to_ring_buffer(uint8_t instance, circ_buffer_t *buffer)
{
  bool buffer_not_full = false;
  if(buffer->status != RING_BUFFER_FULL)
  {
    buffer_not_full = true;

    // TODO SKAL SLETTES IGEN
    if(instance != '\r')
    *(buffer->buffer+buffer->head++) = instance;

    if(buffer->head == RING_BUFFER_SIZE)
      buffer->head = 0;
    
    update_circ_buffer_status(buffer);
  }
  return buffer_not_full;

}
/***************************************************************************/
/* Save a copy of the first data point in buffer.
 * Return false if buffer is empty else true
*/
bool pop_from_ring_buffer(uint8_t *instance, circ_buffer_t *buffer)
{
  bool data_in_buffer = false;
  if(buffer->status != RING_BUFFER_EMPTY)
  {
    data_in_buffer = true;
    *instance = *(buffer->buffer+buffer->tail++);
    if(buffer->tail == RING_BUFFER_SIZE)
      buffer->tail = 0;

    update_circ_buffer_status(buffer);
  }
  return data_in_buffer;
}
/***************************************************************************/
/* Remove all instance of buffer */
void clear_ring_buffer(circ_buffer_t *buffer)
{
  buffer->tail = buffer->head;
  buffer->status = RING_BUFFER_EMPTY;
}
/***************************************************************************/
/* Copy all instances of circular buffer to array. 
 * Return false if buffer is empty, else true 
*/
bool fetch_ring_buffer(uint8_t *fetch_buffer, uint16_t fetch_buffer_size, circ_buffer_t *buffer)
{
  bool data_in_buffer = false;
  uint16_t fetch_buffer_index = 0;
  if(buffer->status != RING_BUFFER_EMPTY)
  {
    data_in_buffer = true;
    while((fetch_buffer_index<fetch_buffer_size) && pop_from_ring_buffer(fetch_buffer+fetch_buffer_index, buffer))
    {
      (fetch_buffer_index)++;
    }
  }
  (fetch_buffer[fetch_buffer_index]) = '\000';
  return data_in_buffer;
}

/***************************************************************************/
/* Copy all instances of circular buffer until identifier is found. 
 * Return false if there is no identical identifier in buffer, else true 
*/
bool fetch_until_id(uint8_t *fetch_buffer, uint16_t fetch_buffer_size, circ_buffer_t *buffer, uint8_t id)
{
  bool id_in_buffer = false;
  int32_t pos_cnt = 0;
  uint16_t fetch_buffer_index = 0;

  while(((buffer->tail+pos_cnt) != buffer->head) && (!id_in_buffer))
  {
    if(id == *(buffer->buffer+buffer->tail+pos_cnt))
    {
      id_in_buffer = true;
    }
    pos_cnt++;
    if(buffer->tail+pos_cnt == RING_BUFFER_SIZE)
      pos_cnt = -(buffer->tail);

    update_circ_buffer_status(buffer);
  }
  if(id_in_buffer)
  {
    while((id != *(buffer->buffer+buffer->tail)) && (fetch_buffer_index < fetch_buffer_size))
    {
      pop_from_ring_buffer(fetch_buffer + fetch_buffer_index, buffer);
      fetch_buffer_index++;
    }
    pop_from_ring_buffer(fetch_buffer + fetch_buffer_index, buffer);
    (fetch_buffer_index)++;
  }
  (fetch_buffer[fetch_buffer_index]) = '\000';
  return id_in_buffer;
}
/***************************************************************************/
/* Copy all instances of string to circular buffer. 
 * Return false if ring buffer is full else true
*/
bool push_string_to_rbuffer(uint8_t *str, circ_buffer_t *buffer)
{
  bool buffer_not_full = true;

  uint16_t counter;
  for(counter = 0; ((counter < strlen(str)) && buffer_not_full); counter++)
  {
    buffer_not_full = push_to_ring_buffer(str[counter],buffer);
  }
  return buffer_not_full;
}