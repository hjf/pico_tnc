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

#pragma once
/*
    send.h
*/
#include "tnc.h"

#define BYTE_BITS 8
#define BIT_STUFF_BITS 5

// Per-packet flag bits stored in the send queue (one byte per packet, ahead
// of the length field). Read in SP_READ_FLAGS and consumed by the rest of
// the state machine.
#define SP_FLAG_SKIP_CSMA  0x01   // skip p-persistence; key the moment DCD clears

int send_byte(tnc_t *tp, uint8_t data, bool bit_stuff);
void send_init(void);
void send(void);
int send_queue_free(tnc_t *tp);
bool send_packet(tnc_t *tp, uint8_t *data, int len);
// Same as send_packet() but bypasses p-persistence/slottime — the TNC keys
// up as soon as the channel is clear.  Use for digipeated frames per
// APRS Protocol Reference §3 / WB2OSZ §3.2.1 (digis rely on FM capture, not
// random backoff).  Not appropriate for KISS/AGWPE-originated traffic.
bool send_packet_now(tnc_t *tp, uint8_t *data, int len);
