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
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"

//#include "timer.h"
#include "tnc.h"
#include "bell202.h"
#include "ax25.h"
#include "usb_output.h"
#include "tty.h"
#include "digipeat.h"
#include "kiss.h"
#if PICO_TNC_PARENT_INTEGRATION
#include "kiss_tcp.h"
#include "agwpe.h"
#include "igate.h"
#endif

#define FCS_OK 0x0f47
//#define FCS_OK (0x0f47 ^ 0xffff)
#define MIN_LEN (7 * 2 + 1 + 1 + 2) // Address field *2, Control, PID, FCS

#define STR_LEN 64

static uint8_t str[STR_LEN];

static void display_packet(tty_t *ttyp, tnc_t *tp, slicer_t *s)
{
    int i;
    int in_addr = 1;
    int len = s->data_cnt;
    uint8_t *data = s->data;
    int size;

#if PORT_N > 1
    size = snprintf(str, STR_LEN, "(%d) %d:%d:", tnc_time(), tp->port, tp->pkt_cnt);
    tty_write(ttyp, str, size);
#endif

    for (i = 0; i < len - 2; i++) {
	    int c;

        if (i < 7) c = data[i + 7];       // src addr
        else if (i < 14) c = data[i - 7]; // dst addr
        else c = data[i];

	    if (in_addr) {
	        int d = c >> 1;

	        if (i % 7 == 6) { // SSID

	            if (i >= 7) in_addr = !(data[i] & 1);  // check address extension bit

		        if (d & 0x0f) { // SSID
                    size = snprintf(str, STR_LEN, "-%d", d & 0x0f);
                    tty_write(ttyp, str, size);
                }
                if (i >= 14 && (c & 0x80)) tty_write_char(ttyp, '*'); // H bit
		        if (i == 6) tty_write_char(ttyp, '>');
                else if (in_addr) tty_write_char(ttyp, ',');
                else { tty_write_char(ttyp, ':'); i += 2; } // skip AX.25 control + PID bytes

	        } else { // CALLSIGN

		        if (d >= '0' && d <= '9') tty_write_char(ttyp, d);
		        else if (d >= 'A' && d <= 'Z') tty_write_char(ttyp, d);
		        else if (d != ' ') {
                    size = snprintf(str, STR_LEN, "<%02x>", d);
                    tty_write(ttyp, str, size);
                }

	        }
	    } else {
	        if (c >= ' ' && c <= '~') tty_write_char(ttyp, c);
	        else {
                size = snprintf(str, STR_LEN, "<%02x>", c);
                tty_write(ttyp, str, size);
            }
	    }
    }
#if 1
    tty_write_str(ttyp, "\r\n");
#else
    size = snprintf(str, STR_LEN, "<%02x%02x>\r\n", data[len-1], data[len-2]);
    tty_write(ttyp, str, size);
#endif
}

// Return true if this just-decoded frame has been seen recently in any
// slicer's output (cross-slicer dedupe). On miss, insert and return false.
//
// Match criteria: 16-bit trailing FCS + 16-bit length + first 4 frame bytes
// (which start the AX.25 destination call). That's ~64 effective bits of
// entropy per entry — false-positive collisions across distinct real frames
// within the 1.5 s window are negligible. (A 16-bit FCS-only key gave ~17
// false dedupes per WA8LMF Track 1 with 5 slicers × ~1000 frames.)
static bool dedup_check_insert(tnc_t *tp, const uint8_t *data, int len)
{
    uint16_t fcs = (uint16_t)data[len - 2] | ((uint16_t)data[len - 1] << 8);
    uint32_t now = tnc_time();

    for (int i = 0; i < DEDUP_RING; i++) {
        dedup_entry_t *e = &tp->dedup_ring[i];
        if (e->ts == 0) continue;                     // empty
        if ((now - e->ts) >= DEDUP_TIMEOUT_TICKS) {   // expired
            e->ts = 0;
            continue;
        }
        if (e->fcs == fcs && e->len == (uint16_t)len &&
            e->prefix[0] == data[0] && e->prefix[1] == data[1] &&
            e->prefix[2] == data[2] && e->prefix[3] == data[3]) {
            return true;                               // duplicate
        }
    }

    // insert
    dedup_entry_t *e = &tp->dedup_ring[tp->dedup_head];
    e->fcs = fcs;
    e->len = (uint16_t)len;
    e->prefix[0] = data[0]; e->prefix[1] = data[1];
    e->prefix[2] = data[2]; e->prefix[3] = data[3];
    e->ts  = now ? now : 1;  // avoid ts==0
    tp->dedup_head = (tp->dedup_head + 1) & (DEDUP_RING - 1);
    return false;
}

// Diagnostic counters (host harness only; zero cost in firmware build because
// the host harness compiles with -DDIAGNOSTICS but firmware doesn't).
#ifdef DIAGNOSTICS
int diag_fcs_ok_per_slicer[NUM_SLICERS];
int diag_dedup_hit_per_slicer[NUM_SLICERS];
int diag_fixbits_attempts;
int diag_fixbits_recoveries;
int diag_fixbits_skipped;
#endif

#if ENABLE_FIXBITS
// Return true if this exact (FCS-failed) frame has been attempted in the
// last 1.5 s. Cheap reuse of the dedup_entry_t format (fcs/len/prefix/ts).
static bool fixbits_recently_tried(tnc_t *tp, const uint8_t *data, int len)
{
    uint16_t fcs = (uint16_t)data[len - 2] | ((uint16_t)data[len - 1] << 8);
    uint32_t now = tnc_time();

    for (int i = 0; i < FIXBITS_RING; i++) {
        dedup_entry_t *e = &tp->fixbits_ring[i];
        if (e->ts == 0) continue;
        if ((now - e->ts) >= DEDUP_TIMEOUT_TICKS) { e->ts = 0; continue; }
        if (e->fcs == fcs && e->len == (uint16_t)len &&
            e->prefix[0] == data[0] && e->prefix[1] == data[1] &&
            e->prefix[2] == data[2] && e->prefix[3] == data[3]) {
            return true;
        }
    }

    dedup_entry_t *e = &tp->fixbits_ring[tp->fixbits_head];
    e->fcs = fcs;
    e->len = (uint16_t)len;
    e->prefix[0] = data[0]; e->prefix[1] = data[1];
    e->prefix[2] = data[2]; e->prefix[3] = data[3];
    e->ts  = now ? now : 1;
    tp->fixbits_head = (tp->fixbits_head + 1) & (FIXBITS_RING - 1);
    return false;
}

// Single-bit-invert salvage (Dire Wolf "fix_bits" level 1). For an N-byte
// FCS-failed candidate, walk each of N*8 bit positions, flip the bit,
// recompute FCS, accept the first hit. Mutates data on success. O(N) CRC
// runs; with the RP2040 DMA-CRC sniffer at ~5 µs per 256-byte CRC this is
// ~10 ms worst case per failed candidate. Higher-order retries (double,
// triple, two-separated bit inversions) are explicitly skipped — their
// marginal gain on the WA8LMF corpus doesn't justify the O(N²) cost.
static bool fixbits_try(uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        for (int b = 0; b < 8; b++) {
            uint8_t mask = 1 << b;
            data[i] ^= mask;
            if (ax25_fcs(0, data, len) == FCS_OK) return true;
            data[i] ^= mask;  // restore
        }
    }
    return false;
}
#endif

static void output_packet(tnc_t *tp, slicer_t *s)
{
    int len = s->data_cnt;
    uint8_t *data = s->data;

    if (len < MIN_LEN) return;

    // FCS check
    if (ax25_fcs(0, data, len) != FCS_OK) {
#if ENABLE_FIXBITS
        // Only attempt fix-bits once per failed-frame content across slicers
        // (same noise → same input → same outcome; skipping saves CPU).
        if (fixbits_recently_tried(tp, data, len)) {
#ifdef DIAGNOSTICS
            diag_fixbits_skipped++;
#endif
            return;
        }
#ifdef DIAGNOSTICS
        diag_fixbits_attempts++;
#endif
        if (!fixbits_try(data, len)) return;
#ifdef DIAGNOSTICS
        diag_fixbits_recoveries++;
#endif
        // fall through with recovered frame
#else
        return;
#endif
    }

#ifdef DIAGNOSTICS
    int slicer_idx = (int)(s - tp->slicer);
    diag_fcs_ok_per_slicer[slicer_idx]++;
#endif

    // Cross-slicer dedupe: another slicer may have already published the
    // same frame on its own end-of-frame within the past 1.5 s.
    if (dedup_check_insert(tp, data, len)) {
#ifdef DIAGNOSTICS
        diag_dedup_hit_per_slicer[slicer_idx]++;
#endif
        return;
    }

    // count received packet
    ++tp->pkt_cnt;

    // Record every heard frame in the digipeat dedup table BEFORE we
    // schedule a repeat — that way, a wide-area digi's later copy lands
    // on top of the same hash and suppresses our held-off TX.
    digipeat_record_rx(data, len - 2);  // strip trailing FCS for hash

#if PICO_TNC_PARENT_INTEGRATION
    agwpe_monitor_rf_packet(data, len);
    kiss_tcp_monitor_rf_packet(data, len);

    // iGate every heard RF packet (independent of digipeat state).
    // The TNC2 conversion strips the 2-byte FCS internally.
    igate_gate_rf_packet(data, len);
#endif

    // digipeat
    if (param.digi) digipeat(tp, data, len);

    for (int i = TTY_USB; i <= TTY_UART0; i++) {
        tty_t *ttyp = &tty[i];

        if (ttyp->kiss_mode) kiss_output(ttyp, tp, s); // kiss mode
        else {

            // TNC MONitor command
            switch (param.mon) {
                case MON_ALL:
                    display_packet(ttyp, tp, s);
                    break;

                case MON_ME:
                    if (ax25_callcmp(&param.mycall, &data[0])) { // dst addr check
                        display_packet(ttyp, tp, s);
                    }
            }
        }
    }
}

#define AX25_FLAG 0x7e

// DCD watchdog: if we entered DATA state on a noise-induced flag and the
// decoder is stuck without producing further bytes, give up after this many
// 10 ms ticks so CSMA can release the channel.
#define DCD_WATCHDOG_TICKS 20

static void dcd_set(tnc_t *tp, int on)
{
    if (tp->dcd == on) return;
    tp->dcd = on;
}

// LED gating — borrowed from arduino_tnc: only light when an in-frame byte
// equals the AX.25 UI control byte (0x03), which by AX.25 framing rules
// can only legally appear immediately after the address field. We further
// require data_cnt >= AX25_MIN_ADDR_BYTES + 1 (i.e., 15) so a stray 0x03
// in a short noise blip can't trip it. Net effect: the LED tracks "we
// are decoding a real AX.25 UI frame", not "a flag-like pattern arrived".
#define AX25_UI_CONTROL 0x03
#define AX25_MIN_ADDR_BYTES 14   // dst (7) + src (7), no digis

static void led_set(tnc_t *tp, int on)
{
    if (tp->led_on == on) return;
    tp->led_on = on;
    gpio_put(tp->cdt_pin, on ? 1 : 0);
}

// Recompute the global DCD LED as the OR of per-slicer in-frame flags.
// At NUM_SLICERS=1 this is just the single slicer's state.
//
// DCD (logical, used by CSMA) trips on any slicer mid-frame.
// LED only follows slicers that have seen a UI control byte at a valid
// position — that flag is cleared every time a slicer enters DATA, so
// the LED naturally goes dark on end-of-frame / watchdog / packet-too-long.
static void dcd_recompute(tnc_t *tp)
{
    int any_in_frame = 0;
    int any_real_frame = 0;
    for (int j = 0; j < NUM_SLICERS; j++) {
        slicer_t *s = &tp->slicer[j];
        if (s->state == DATA && s->data_cnt > 0) {
            any_in_frame = 1;
            if (s->ui_seen) {
                any_real_frame = 1;
                break;
            }
        }
    }
    dcd_set(tp, any_in_frame);
    led_set(tp, any_real_frame);
}

static void decode_bit_slicer(tnc_t *tp, slicer_t *s, int bit)
{
    s->flag <<= 1;
    s->flag |= bit;

    switch (s->state) {
        case FLAG:
	    if (s->flag == AX25_FLAG) { // found flag
	        s->state = DATA;
	        s->data_cnt = 0;
	        s->data_bit_cnt = 0;
	        s->ui_seen = 0;
	    }
	    break;

        case DATA:
	    if ((s->flag & 0x3f) == 0x3f) { // AX.25 flag, end of packet, six continuous "1" bits
	        output_packet(tp, s);
	        s->state = FLAG;
	        dcd_recompute(tp);
	        break;
	    }

	    if ((s->flag & 0x3f) == 0x3e) break; // delete bit stuffing bit

	    s->data_byte >>= 1;
	    s->data_byte |= bit << 7;
	    s->data_bit_cnt++;
	    if (s->data_bit_cnt >= 8) {
	        if (s->data_cnt < DATA_LEN) s->data[s->data_cnt++] = s->data_byte;
            else {
                printf("packet too long > %d\n", s->data_cnt);
                s->state = FLAG;
                dcd_recompute(tp);
                break;
            }
	        s->data_bit_cnt = 0;
	        // A byte successfully decoded in DATA state — a frame is being
	        // received. Assert DCD on the first byte (preamble is past). The
	        // LED only follows the UI-control-byte gate, which arduino_tnc
	        // showed is a reliable "real AX.25 frame" indicator.
	        s->dcd_last_byte_time = tnc_time();
	        if (!tp->dcd) dcd_set(tp, 1);
	        if (!s->ui_seen &&
	            s->data_byte == AX25_UI_CONTROL &&
	            s->data_cnt > AX25_MIN_ADDR_BYTES) {
	            s->ui_seen = 1;
	            if (!tp->led_on) led_set(tp, 1);
	        }
	    }
    }
}

static void decode_slicer(tnc_t *tp, slicer_t *s, int val)
{
    s->edge++;
    if (val != s->pval) {
        int bits = (s->edge * BAUD_RATE*2 + SAMPLING_RATE) / (SAMPLING_RATE * 2);

	    decode_bit_slicer(tp, s, 0);      // NRZI
	    while (--bits > 0) {
	        decode_bit_slicer(tp, s, 1);
	    }

	    s->edge = 0;
	    s->pval = val;
    }
}

#define PLL_STEP ((int)(((1ULL << 32) + SAMPLING_N/2) / SAMPLING_N))

// DireWolf PLL — per slicer
static void decode2_slicer(tnc_t *tp, slicer_t *s, int val)
{
    int32_t prev_pll = s->pll_counter >> 31; // compiler bug workaround

    s->pll_counter += PLL_STEP;

    if ((s->pll_counter >> 31) < prev_pll) { // overflow

        decode_bit_slicer(tp, s, val == s->nrzi);    // decode NRZI
        s->nrzi = val;
    }

    if (val != s->pval) {

        // Adaptive PLL inertia (Dire Wolf-style):
        //   not-yet-locked (FLAG state) → >>1 = ×0.5 — faster pull-in on
        //                                 the preamble flag train
        //   locked (DATA state)         → >>2 = ×0.75 — lower jitter inside
        //                                 the frame so destuff/CRC stay aligned
        int sh = (s->state == DATA) ? 2 : 1;
        s->pll_counter -= s->pll_counter >> sh;
        s->pval = val;
    }
}

void demodulator(tnc_t *tp, int adc)
{
    int val;

    //printf("%d,", adc);

    //dac_output_voltage(DAC_CHANNEL_1, adc >> 4);

#define AVERAGE_MUL 256 
#define AVERAGE_N 64
#define AVERAGE_SHIFT 6
#define CDT_AVG_N 128
#define CDT_MUL 256
#define CDT_SHIFT 6

    // update average value
    tp->avg += (adc * AVERAGE_MUL - tp->avg) >> AVERAGE_SHIFT;
    val = adc - (tp->avg + AVERAGE_MUL/2) / AVERAGE_MUL;

    // carrier detect
    tp->cdt_lvl += (val * val * CDT_MUL - tp->cdt_lvl) >> CDT_SHIFT;

#if 0
    static int count = 0;
    if ((++count & ((1 << 16) - 1)) == 0) {
        printf("(%u) decode: adc: %d, cdt_lvl: %d, avg: %d, port = %d\n", tnc_time(), adc, tp->cdt_lvl, tp->avg, tp->port);
    }
#endif

#define CDT_THR_LOW 1024
#define CDT_THR_HIGH (CDT_THR_LOW * 2) // low +6dB

    if (!tp->cdt && tp->cdt_lvl > CDT_THR_HIGH) { // CDT on (signal energy only — demod gate)

        tp->cdt = true;

    } else if (tp->cdt && tp->cdt_lvl < CDT_THR_LOW) { // CDT off

        tp->cdt = false;

    }

    // Per-slicer DCD watchdog: any slicer stuck in DATA without producing a
    // further byte (e.g. fell into DATA on a noise-induced 0x7e) gets reset.
    // Recompute the global LED state after sweeping.
    if (tp->dcd) {
        uint32_t now = tnc_time();
        for (int j = 0; j < NUM_SLICERS; j++) {
            slicer_t *s = &tp->slicer[j];
            if (s->state == DATA && s->data_cnt > 0 &&
                (now - s->dcd_last_byte_time) >= DCD_WATCHDOG_TICKS) {
                s->state = FLAG;
            }
        }
        dcd_recompute(tp);
    }

    if (!tp->cdt) return;

#if 0
	sum += adc;
	if (++count >= AVERAGE_N) {
	    average = sum / AVERAGE_N;
	    //ESP_LOGI(TAG, "average adc value = %d",  average);
	    sum = 0;
	    count = 0;
	}
#else
	//tp->average = (tp->average * (AVERAGE_N - 1) + adc + AVERAGE_N/2) / AVERAGE_N;
	/*
	if (count >= 13200) {
		printf("average = %d\n", average);
		count = 0;
	}
	*/
#endif

#define LPF_N SAMPLING_N


	//val = bell202_decode((int)adc - average);
	//val = bell202_decode((int)adc - 2048);
	//val = bell202_decode(tp, adc); // delayed decode
#ifdef BELL202_SYNC
	val = bell202_decode2(tp, adc); // sync decode
#else
	val = bell202_decode(tp, adc); // delayed decode
#endif
	//lpf = (lpf * (LPF_N-1) + val) / LPF_N;

	//printf("demodulator(%d) = %d\n", adc, val);
#if 0
    static uint32_t count = 0;
    if (count < 132) {
        printf("%d, %d\n", adc, val);
    }
    if (++count >= 13200 * 10) {
        count = 0;
        printf("----------------\n");
    }
#endif

#define LPF_THRESHOLD (1 << 12)

    // Fan out the shared LPF output across all parallel slicers. Each slicer
    // applies its own threshold offset and runs an independent PLL + NRZI +
    // HDLC state machine. With NUM_SLICERS=1 (current default) this behaves
    // identically to the original single-decoder path.
    for (int j = 0; j < NUM_SLICERS; j++) {
        slicer_t *s = &tp->slicer[j];

        // Per-slicer decision: shift the LPF output by the slicer's bias
        // offset, then apply the original ±LPF_THRESHOLD deadband (which
        // acts as zero-crossing hysteresis against noise). The offset
        // effectively moves the decision center: a positive offset biases
        // the slicer to read mark (data bit 0) more readily — useful when
        // the radio's de-emphasis attenuates the mark tone relative to space.
        int v = val + s->offset;
        if      (v < -LPF_THRESHOLD) s->bit = 1;
        else if (v >= LPF_THRESHOLD) s->bit = 0;
        // else: hold previous s->bit (deadband)

#ifdef DECODE_PLL
        decode2_slicer(tp, s, s->bit);   // Direwolf PLL
#else
        decode_slicer(tp, s, s->bit);    // bit length
#endif
    }
}
