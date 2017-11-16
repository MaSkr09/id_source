/****************************************************************************
# AVR Serial Line Internet Protocol implementation
#
# Copyright (c) 2015-2016, Kjeld Jensen <kj@kjen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************
# File: slip.c
# Project: AVR Serial Line Internet Protocol implementation
#          https://www.ietf.org/rfc/rfc1055.txt
# Platform: AVR
# Microcontroller: Atmel AVR
# Author: Kjeld Jensen <kj@kjen.dk>
# Created:  2015-03-30 Kjeld Jensen, only slip_tx_packet works for now
# Modified: 2015-06-22 Kjeld Jensen, added rx functionality
# Modified: 2016-05-15 Kjeld Jensen, corrected a bug in slip_rx_update()
#                                    reported by Mathias Neerup
# Modified: 2017-06-01 Martin Skriver, Modified to fit DroneID
****************************************************************************/
/* includes */
//#include "avr_serial.h"
//#include "nano_def.h"
#include "main.h"

/* SLIP special character codes  */
#define SLIP_END		192    /* indicates end of packet */
#define SLIP_ESC		219    /* indicates byte stuffing */
#define SLIP_ESC_END	220    /* ESC ESC_END means END data byte */
#define SLIP_ESC_ESC	221    /* ESC ESC_ESC means ESC data byte */

#define SLIP_INBUF_LEN	20

#define SLIP_STATE_STD	0
#define SLIP_STATE_ESC	1

/* global variables */
unsigned char slip_in[SLIP_INBUF_LEN];
int slip_in_ptr;

/* local variables */
static unsigned char c;
static char slip_state;

/***************************************************************************/
void slip_init (void)
{
//	serial_init();
	slip_state = SLIP_STATE_STD;
	slip_in_ptr = -1;
}
/***************************************************************************/
char slip_rx_update (void)
{
//	char new_packet = FALSE;
//
//	while (serial_rx_avail() && new_packet == FALSE) /* handle RX buffer */
//	{
//		c = serial_rx();
//		if (slip_state == SLIP_STATE_STD)
//		{		
//			switch (c)
//			{
//				case SLIP_END:
//					if (slip_in_ptr > -1)
//						new_packet = TRUE;
//					break;
//
//				case SLIP_ESC:
//					slip_state = SLIP_STATE_ESC;
//					break;
//
//				default:
//					slip_in[++slip_in_ptr] = c;
//					break;
//			}
//		}
//
//		/* handle ESC state */
//		else
//		{
//			switch(c)
//			{
//				case SLIP_ESC_END:
//					slip_in[++slip_in_ptr] = SLIP_END;
//					break;
//
//				case SLIP_ESC_ESC:
//					slip_in[++slip_in_ptr] = SLIP_ESC;
//					break;
//			}
//
//			slip_state = SLIP_STATE_STD;
//		}
//	}
//
//	/* handle buffer overflow */
//	if (slip_in_ptr == SLIP_INBUF_LEN) 
//	{
//		slip_in_ptr = -1;
//		new_packet = FALSE;
//	}
//
//	return new_packet;
}
/***************************************************************************/

/***************************************************************************/
// Return final lenght
char slip_generate_package (unsigned char *p, int initial_len)
{
  char count, added_chars = 0, buffer1, buffer2, len = initial_len;
  for(len = 0; len < (initial_len + added_chars); len++)//(((len--)+added_chars))
  {
    switch(*p)
    {
    case SLIP_END:
      *p = SLIP_ESC;
      buffer2 = SLIP_ESC_END;
      added_chars++;
      for(count = 1; (len+count) <= (initial_len+added_chars); count++)
      {
        buffer1 = *(p+count);
        *(p+count) = buffer2;
        buffer2 = buffer1;
      }
    break;

    case SLIP_ESC:
      *p = SLIP_ESC;
      buffer2 = SLIP_ESC_ESC;
      added_chars++;
      for(count = 1; (len+count) <= (initial_len+added_chars); count++)
      {
        buffer1 = *(p+count);
        *(p+count) = buffer2;
        buffer2 = buffer1;
      }
    break;

    default:
    break;
    }
    p++;
  }
  added_chars++;
  *p = SLIP_END;
  return initial_len+added_chars;
}
