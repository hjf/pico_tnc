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
#include "ax25.h"
#include "send.h"

#include <string.h>
#include "digipeat.h"

void send_unproto(tnc_t *tp, uint8_t *data, int len)
{
    if (!data || len < 0 || len > AX25_MAX_INFO_LEN) return;
    if (!ax25_callsign_valid(&param.mycall) || !ax25_callsign_valid(&param.unproto[0])) return;
    uint8_t frame[AX25_MAX_FRAME_LEN];
    ax25_mkax25addr(frame, &param.unproto[0]);
    frame[6] |= 0x80;
    ax25_mkax25addr(frame+7, &param.mycall);
    int n = 14;
    for (int i = 1; i < UNPROTO_N && param.unproto[i].call[0]; ++i) {
        if (!ax25_callsign_valid(&param.unproto[i])) return;
        ax25_mkax25addr(frame+n, &param.unproto[i]);
        n += 7;
    }
    frame[n-1] |= 1;
    frame[n++] = 3;
    frame[n++] = 0xf0;
    memcpy(frame+n, data, len);
    n += len;
    if (send_packet(tp, frame, n)) digipeat_record_rx(frame, n);
}
