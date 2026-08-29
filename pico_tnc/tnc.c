/*
Copyright (c) 2021, JN1DFF
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
#include <string.h>

#include "tnc.h"
#include "ax25.h"
#include "flash.h"
#include "send.h"
#include "digipeat.h"

uint32_t __tnc_time;

tnc_t tnc[PORT_N];

#ifndef DIGI_ENABLE
#define DIGI_ENABLE 0
#endif

typedef struct LEGACY_TNC_PARAM {
    callsign_t mycall;
    callsign_t myalias;
    callsign_t unproto[UNPROTO_N];
    uint8_t btext[BTEXT_LEN + 1];
    uint8_t txdelay;
    uint8_t gps;
    uint8_t mon;
    uint8_t digi;
    uint8_t beacon;
    uint8_t trace;
    uint8_t echo;
} legacy_param_t;

param_t param;

static void param_set_defaults(void)
{
    param = (param_t) {
        .format_magic = PARAM_FORMAT_MAGIC,
        .unproto = {
            { .call = { 'A', 'P', 'R', 'S', ' ', ' ' }, .ssid = 0 },
        },
        .txdelay = 100,
        .echo = 1,
        .digi = DIGI_ENABLE,
        .digi_path = "WIDE1-1",
        .digi_holdoff_ms = 1500,
        .beacon_symbol_table = '/',
        .beacon_symbol_code = '#',
    };
}

static void param_read(void)
{
    uint32_t stored_magic = 0;
    param_set_defaults();

    if (!flash_read(&stored_magic, sizeof(stored_magic))) return;

    if (stored_magic == PARAM_FORMAT_MAGIC) {
        param_t stored;
        if (flash_read(&stored, sizeof(stored))) param = stored;
        return;
    }

    legacy_param_t legacy;
    if (!flash_read(&legacy, sizeof(legacy))) return;
    param.mycall = legacy.mycall;
    param.myalias = legacy.myalias;
    memcpy(param.unproto, legacy.unproto, sizeof(legacy.unproto));
    if (!param.unproto[0].call[0]) {
        memcpy(param.unproto[0].call, "APRS  ", sizeof(param.unproto[0].call));
        param.unproto[0].ssid = 0;
    }
    memcpy(param.btext, legacy.btext, sizeof(legacy.btext));
    param.txdelay = legacy.txdelay;
    param.gps = legacy.gps;
    param.mon = legacy.mon;
    param.digi = legacy.digi;
    param.beacon = legacy.beacon;
    param.trace = legacy.trace;
    param.echo = legacy.echo;
}

void tnc_init(void)
{
    // filter initialization
    // LPF
    static const filter_param_t flt_lpf = {
        .size = FIR_LPF_N,
        .sampling_freq = SAMPLING_RATE,
        .pass_freq = 0,
        .cutoff_freq = 1200,
    };
    int16_t *lpf_an, *bpf_an;

    lpf_an = filter_coeff(&flt_lpf);

#if 0
    printf("LPF coeffient\n");
    for (int i = 0; i < flt_lpf.size; i++) {
        printf("%d\n", lpf_an[i]);
    }
#endif
    // BPF
    static const filter_param_t flt_bpf = {
        .size = FIR_BPF_N,
        .sampling_freq = SAMPLING_RATE,
        .pass_freq = 900,
        .cutoff_freq = 2500,
    };
    bpf_an = filter_coeff(&flt_bpf);
#if 0
    printf("BPF coeffient\n");
    for (int i = 0; i < flt_bpf.size; i++) {
        printf("%d\n", bpf_an[i]);
    }
#endif
    // PORT initialization
    for (int i = 0; i < PORT_N; i++) {
        tnc_t *tp = &tnc[i];

        // receive
        tp->port = i;
        filter_init(&tp->lpf, lpf_an, FIR_LPF_N);
        filter_init(&tp->bpf, bpf_an, FIR_BPF_N);

        // Per-slicer state. Threshold offsets are spread symmetrically around
        // zero on the LPF_THRESHOLD (=4096) scale, mirroring Dire Wolf's
        // exponential gain ladder but applied additively (since our LPF
        // output is already zero-mean, an additive offset is the right knob
        // for biasing the sign-decision point). Tuned empirically; see the
        // host replay harness under tests/replay/.
        static const int16_t slicer_offsets[NUM_SLICERS] = {
#if NUM_SLICERS == 1
            0,
#elif NUM_SLICERS == 3
            -6144, 0, +6144,
#elif NUM_SLICERS == 5
            -8192, -3072, 0, +3072, +8192,
#elif NUM_SLICERS == 9
            -12288, -8192, -5120, -2048, 0, +2048, +5120, +8192, +12288,
#elif NUM_SLICERS == 7
            -10240, -5120, -2048, 0, +2048, +5120, +10240,
#else
#error "Unsupported NUM_SLICERS — add an offset ladder for this count"
#endif
        };
        for (int j = 0; j < NUM_SLICERS; j++) {
            slicer_t *s = &tp->slicer[j];
            s->offset = slicer_offsets[j];
            s->state = FLAG;
        }

        // send queue
        queue_init(&tp->send_queue, sizeof(uint8_t), SEND_QUEUE_LEN);
        tp->send_state = SP_IDLE;

        tp->cdt = 0;
        tp->kiss_txdelay = 50;
        tp->kiss_p = 63;
        tp->kiss_slottime = 10;
        tp->kiss_fullduplex = 0;

        // calibrate
        tp->do_nrzi = true;
    }

    //printf("%d ports support\n", PORT_N);
    //printf("DELAYED_N = %d\n", DELAYED_N);

    // read flash
    param_read();

    // set kiss txdelay
    if (param.txdelay > 0) {
        tnc[0].kiss_txdelay = param.txdelay * 2 / 3;
    }
}

// Inject an AX.25 frame for transmission from outside pico_tnc (e.g. iGate
// IS->RF gating). `data` is the raw AX.25 frame WITHOUT trailing FCS —
// send_packet() computes and appends it. Returns false if the send queue
// is full or the port number is invalid.
//
// The frame is also recorded in the digipeat dedup table so that if it
// loops back through RF reception during the hold-off window, our own
// digipeater won't re-transmit it.
bool tnc_inject_tx(int port, const uint8_t *data, int len)
{
    if (port < 0 || port >= PORT_N) return false;
    if (!send_packet(&tnc[port], (uint8_t *)data, len)) return false;
    digipeat_record_rx(data, len);
    return true;
}
