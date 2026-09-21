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

#include "tnc.h"
#include "send.h"
#include "tty.h"
#include "digipeat.h"

#define FEND KISS_FEND_BYTE
#define FESC KISS_FESC_BYTE
#define TFEND KISS_TFEND_BYTE
#define TFESC KISS_TFESC_BYTE

#define KISS_PACKET_LEN 1024

enum KISS_COMM {
    KISS_DATA = 0,
    KISS_TXDELAY,
    KISS_P,
    KISS_SLOTTIME,
    KISS_TXTAIL,
    KISS_FULLDUPLEX,
    KISS_SETHARDWARE,
};

void kiss_packet(tty_t *ttyp)
{
    if (ttyp->kiss_idx == 0) return; // packet length == 0

    int type = ttyp->kiss_buf[0]; // kiss type indicator

    if (type == 0xff) {

        // exit kiss mode
        ttyp->kiss_mode = 0;
        return;

    }

    if (ttyp->kiss_idx < 2) return;

    int port = type >> 4;   // port No.
    int comm = type & 0x0f;  // command

    if (port >= PORT_N) return;

    if (PICO_TNC_STANDALONE) return; // monitor-only KISS; host cannot alter RF scheduling

    tnc_t *tp = &tnc[port];
    int val = ttyp->kiss_buf[1];

    // kiss command
    switch (comm) {

        case KISS_DATA:
            // send kiss packet
#if !PICO_TNC_STANDALONE
            if (send_packet(tp, &ttyp->kiss_buf[1], ttyp->kiss_idx - 1)) {
                digipeat_record_local_origin(&ttyp->kiss_buf[1], ttyp->kiss_idx - 1);
                digipeat_record_rx(&ttyp->kiss_buf[1], ttyp->kiss_idx - 1);
            }
#endif
            break;

        case KISS_TXDELAY:
            tp->kiss_txdelay = val;
            break;

        case KISS_P:
            tp->kiss_p = val;
            break;

        case KISS_SLOTTIME:
            tp->kiss_slottime = val;
            break;

        case KISS_FULLDUPLEX:
            tp->kiss_fullduplex = val;
            break;
    }
}

static bool kiss_serial_frame(void *ctx, const uint8_t *frame, int len)
{
    tty_t *ttyp = ctx;
    (void)frame;
    ttyp->kiss_idx = len;
    kiss_packet(ttyp);
    return ttyp->kiss_mode != 0;
}

void kiss_input(tty_t * ttyp, int ch)
{
    ttyp->kiss_timeout = tnc_time();
    kiss_stream_input(ttyp->kiss_buf, KISS_PACKET_LEN, &ttyp->kiss_idx,
                      &ttyp->kiss_state, (uint8_t)ch,
                      kiss_serial_frame, ttyp);
}

void kiss_output(tty_t *ttyp, tnc_t *tp, slicer_t *s)
{
    int len = s->data_cnt;
    uint8_t *data = s->data;

    // KISS start
    tty_write_char(ttyp, FEND);

    // kiss type, port, data frame 0
    uint8_t type = tp->port << 4;
    tty_write_char(ttyp, type);

    for (int i = 0; i < len - 2; i++) { // delete FCS

        int ch =  data[i];

        switch (ch) {
            case FEND:
                tty_write_char(ttyp, FESC);
                tty_write_char(ttyp, TFEND);
                break;

            case FESC:
                tty_write_char(ttyp, FESC);
                tty_write_char(ttyp, TFESC);
                break;

            default:
                tty_write_char(ttyp, ch);
        }
    }

    // KISS end
    tty_write_char(ttyp, FEND);
}
