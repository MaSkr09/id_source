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
 # File: circ_buffer.h
 # Purpose: Circular buffer
 # Project: DroneID
 # Author: Martin Skriver <maskr09@gmail.com>
 # Created:  2016-04-29 Martin Skriver, Source written
 *****************************************************************************
 # TODO:
 # make peak functions.
 # pop from head.
 # make array size dynamic
****************************************************************************/
#ifndef CIRC_BUFFER_H_
#define CIRC_BUFFER_H_

/***************************************************************************/
/* system includes */
#include <stdint.h>
#include <stdbool.h>

/***************************************************************************/
/* application includes */

/***************************************************************************/
/* buffer type */
#ifndef RING_BUFFER_SIZE
#define RING_BUFFER_SIZE       (uint16_t)100      /* Size of ring buffer*/
#endif

#define RING_BUFFER_CONT_DATA  (uint8_t)0
#define RING_BUFFER_EMPTY      (uint8_t)1
#define RING_BUFFER_FULL       (uint8_t)2

typedef struct
{
  uint8_t status;                                 /* Status of buffer */
  uint8_t buffer[RING_BUFFER_SIZE];               /* circular buffer */
  uint16_t head;                                  /* Pointer to last element in buffer */
  uint16_t tail;                                  /* Pointer to first element in buffer */
} circ_buffer_t;

/***************************************************************************/

/* function prototypes */
void init_ring_buffer(circ_buffer_t *buffer);
bool push_to_ring_buffer(uint8_t instance, circ_buffer_t *buffer);
bool pop_from_ring_buffer(uint8_t *instance, circ_buffer_t *buffer);
void clear_ring_buffer(circ_buffer_t *buffer);
bool fetch_ring_buffer(uint8_t *fetch_buffer, uint16_t fetch_buffer_size, circ_buffer_t *buffer);
bool fetch_until_id(uint8_t *fetch_buffer, uint16_t fetch_buffer_size, circ_buffer_t *buffer, uint8_t id);
bool push_string_to_rbuffer(uint8_t *str, circ_buffer_t *buffer);
/***************************************************************************/
#endif /* CIRC_BUFFER_H_ */