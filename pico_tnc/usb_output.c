/*
Copyright (c) 2021, Kazuhisa Yokota, JN1DFF
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, 
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, 
  this list of conditions and the following disclaimer in the documentation 
  and/or other materials provided with the distribution.
* Neither the name of the <organization> nor the names of its contributors 
  may be used to endorse or promote products derived from this software 
  without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "class/cdc/cdc_device.h"

// usb_write/usb_write_char are legacy stubs; tty.c now calls usb_multi_write directly.
#include "usb_multi.h"

void usb_output_init(void)
{
    // Initialisation handled by usb_multi_init() in pico-digipeater.c
}

void usb_write(uint8_t const *data, int len)
{
    usb_multi_write(USB_CDC_TNC2_0, data, len);
}

void usb_write_char(uint8_t ch)
{
    usb_multi_write_char(USB_CDC_TNC2_0, ch);
}

void usb_output(void)
{
    // Flushing handled inside usb_multi_task()
}

#if 0
void tud_cdc_rx_wanted_cb(uint8_t itf, char wanted_char)
{
    printf("tud_cdc_rx_wanted_cb(%d), wanted_char = %02x\n", itf, wanted_char);
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    printf("tud_cdc_line_state_cb(%d), dtr = %d, rts = %d\n", itf, dtr, rts);
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p_line_coding)
{
    printf("tud_cdc_line_coding_cb(%d), p_line_coding = %p\n", itf, p_line_coding);
}

void tud_cdc_send_break_cb(uint8_t itf, uint16_t duration_ms)
{
    printf("tud_cdc_send_break_cb(%d), duration_ms = %u\n", itf, duration_ms);
}
#endif
