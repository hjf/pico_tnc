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

#ifndef DIGI_MYCALL
#define DIGI_MYCALL ""
#endif
#ifndef DIGI_PATH
#define DIGI_PATH "WIDE1-1"
#endif
#ifndef DIGI_HOLDOFF_MS
#define DIGI_HOLDOFF_MS 1500
#endif

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
        .digi_path = DIGI_PATH,
        .digi_holdoff_ms = DIGI_HOLDOFF_MS,
        .beacon_symbol_table = '/',
        .beacon_symbol_code = '#',
    };
}

// Blank/corrupt storage uses reviewed build defaults; NOCALL never enables RF.
static void default_call(void)
{
    const char *p = DIGI_MYCALL;
    callsign_t c = { .call = {' ', ' ', ' ', ' ', ' ', ' '} };
    int i = 0;
    while (*p && *p != '-' && i < 6) c.call[i++] = *p++;
    if (*p == '-') {
        ++p;
        if (*p < '0' || *p > '9') return;
        unsigned ssid = 0;
        while (*p >= '0' && *p <= '9') {
            ssid = ssid * 10 + (*p++ - '0');
            if (ssid > 15) return;
        }
        c.ssid = ssid;
    }
    if (*p || !ax25_callsign_valid(&c) || !memcmp(c.call, "NOCALL", 6)) {
        param.digi = 0;
        return;
    }
    param.mycall = c;
}

static bool param_valid(const param_t *p)
{
    if (p->format_magic != PARAM_FORMAT_MAGIC ||
        !memchr(p->btext, 0, sizeof(p->btext)) ||
        !memchr(p->digi_path, 0, sizeof(p->digi_path)) ||
        !digipeat_path_valid(p->digi_path) || p->digi_holdoff_ms > 5000 ||
        p->beacon > 60 || p->digi > 1 || p->echo > 1 || p->mon > MON_OFF || p->gps > 2 ||
        p->trace > 2 || p->txdelay > 200 || p->beacon_position_set > BEACON_POSITION_GPS ||
        p->beacon_lat_e7 < -900000000 || p->beacon_lat_e7 > 900000000 ||
        p->beacon_lon_e7 < -1800000000 || p->beacon_lon_e7 > 1800000000 ||
        p->beacon_symbol_code < '!' || p->beacon_symbol_code > '~' ||
        !(p->beacon_symbol_table == '/' || p->beacon_symbol_table == '\\' ||
          (p->beacon_symbol_table >= '0' && p->beacon_symbol_table <= '9') ||
          (p->beacon_symbol_table >= 'A' && p->beacon_symbol_table <= 'Z'))) return false;
    if (p->mycall.call[0] && !ax25_callsign_valid(&p->mycall)) return false;
    if (p->myalias.call[0] && !ax25_callsign_valid(&p->myalias)) return false;
    for (int i = 0; i < UNPROTO_N; ++i)
        if (p->unproto[i].call[0] && !ax25_callsign_valid(&p->unproto[i])) return false;
    return true;
}

static void param_read(void)
{
    param_set_defaults();
    default_call();
    param_t stored;
    if (flash_read(&stored, sizeof(stored)) && param_valid(&stored)) param = stored;
    if (!ax25_callsign_valid(&param.mycall) || !memcmp(param.mycall.call, "NOCALL", 6)) {
        param.digi = param.beacon = 0;
        memset(&param.mycall, 0, sizeof(param.mycall));
    }
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
