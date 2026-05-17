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
#include "tnc.h"

void digipeat(tnc_t *tp, uint8_t *packet, int len);

// Records every valid received frame in the dedup table. Called from
// decode.c output_packet() for all heard frames. `len` is the logical
// packet length without trailing FCS. The hash excludes the digipath
// so a wide-area digi's repeat matches its original.
void digipeat_record_rx(const uint8_t *packet, int len);

// Records source callsigns of packets originating from local clients
// (KISS/AGWPE/serial). RF packets addressed to these stations are
// suppressed in digipeat() for a limited TTL window.
void digipeat_record_local_origin(const uint8_t *packet, int len);

// True iff the 7-byte AX.25 address at `addr` matches a callsign recently
// seen as the source of a locally-originated packet (KISS/AGWPE/serial).
// Used by digipeat() (suppress RF repeat to a local destination) and by
// the iGate (suppress APRS-IS gating of our own echoed transmissions).
bool digipeat_addr_is_local_origin(const uint8_t *addr);

// Drains the hold-off pending-TX slots. Call from the main loop.
// Slots whose hold-off has elapsed are either transmitted (via
// send_packet) or silently suppressed if another digi was heard
// repeating the same packet during the hold-off window.
void digipeat_poll(void);
