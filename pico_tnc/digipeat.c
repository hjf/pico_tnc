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
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"

#include "tnc.h"
#include "ax25.h"
#include "send.h"
#include "digipeat.h"

#define SSID_LOC 6
#define H_BIT 0x80
#define AX25_MIN_LEN (7 + 7 + 1 + 1)    // dst, src, control, PID
#define MAX_DIGIPEATER 8

#ifndef DIGI_PATH
#define DIGI_PATH ""
#endif

// Hold-off (ms) before a fill-in digi retransmits. A wide-area digi whose
// reach overlaps ours will usually transmit first; if we hear its copy
// during the hold-off, we suppress ours via the dedup check below.
// Setting this to 0 makes digipeat behave exactly like before.
//
// TODO(digipeat): add jitter — pick fire_at = now + DIGI_HOLDOFF_MS +
//   rand(0, DIGI_HOLDOFF_JITTER_MS) — so synchronous fill-ins don't all
//   key up at the same instant.
// TODO(digipeat): external GPIO carrier-detect input — gate TX on
//   (!tp->dcd) && (!gpio_get(EXT_CD_PIN)) for radios whose squelch
//   tracks faster than the HDLC-derived dcd.
// HDLC-based carrier-sense (tp->dcd) already gates TX in send.c. tp->cdt
// is now signal-energy only and used internally as a demod gate.
#ifndef DIGI_HOLDOFF_MS
#define DIGI_HOLDOFF_MS 0
#endif

#define MAX_DIGI_ALIASES 8

// Each alias stores the AX.25 name (space-padded, NOT shifted) and the
// maximum incoming SSID we will serve (1..15). We accept any packet whose
// next non-repeated digi is NAME-N with 1 <= N <= max_ssid, decrement N,
// and set the H bit when N reaches 0.
typedef struct {
    char name[6];
    uint8_t max_ssid;
} digi_alias_t;

static digi_alias_t digi_aliases[MAX_DIGI_ALIASES];
static int n_digi_aliases = 0;
static bool digi_init_done = false;

static int strip_trailing_spaces(const char *name)
{
    int n = 6;
    while (n > 0 && name[n - 1] == ' ') n--;
    return n;
}

static void parse_one_alias(const char *start, const char *end)
{
    while (start < end && isspace((unsigned char)*start)) start++;
    while (end > start && isspace((unsigned char)*(end - 1))) end--;
    if (start >= end || n_digi_aliases >= MAX_DIGI_ALIASES) return;

    const char *dash = NULL;
    for (const char *p = start; p < end; p++) {
        if (*p == '-') { dash = p; break; }
    }

    const char *name_end = dash ? dash : end;
    int name_len = (int)(name_end - start);
    if (name_len <= 0 || name_len > 6) return;

    int ssid = 0;
    if (dash) {
        if (dash + 1 >= end) return;
        for (const char *p = dash + 1; p < end; p++) {
            if (!isdigit((unsigned char)*p)) return;
            ssid = ssid * 10 + (*p - '0');
        }
        if (ssid < 1 || ssid > 15) return;
    } else {
        // bare name (e.g. "WIDE") — default to max ssid 7
        ssid = 7;
    }

    digi_alias_t *a = &digi_aliases[n_digi_aliases++];
    memset(a->name, ' ', 6);
    for (int i = 0; i < name_len; i++) {
        a->name[i] = toupper((unsigned char)start[i]);
    }
    a->max_ssid = (uint8_t)ssid;
}

static void digipeat_init(void)
{
    const char *path = DIGI_PATH;
    const char *start = path;
    for (const char *p = path; ; p++) {
        if (*p == ',' || *p == '\0') {
            parse_one_alias(start, p);
            if (*p == '\0') break;
            start = p + 1;
        }
    }

    digi_init_done = true;

    printf("Digipeat: %d aliases", n_digi_aliases);
    for (int i = 0; i < n_digi_aliases; i++) {
        int n = strip_trailing_spaces(digi_aliases[i].name);
        printf(" %.*s-%u", n, digi_aliases[i].name, digi_aliases[i].max_ssid);
    }
    printf("\n");
}

// Compare a 6-char alias name against an AX.25 address (shifted left 1).
static bool alias_name_match(const char *name, const uint8_t *addr)
{
    for (int i = 0; i < 6; i++) {
        if (name[i] != (char)(addr[i] >> 1)) return false;
    }
    return true;
}

// Returns index into digi_aliases on match, else -1.
static int find_alias_match(const uint8_t *addr)
{
    uint8_t ssid = (addr[SSID_LOC] >> 1) & 0x0f;
    if (ssid == 0) return -1;
    for (int i = 0; i < n_digi_aliases; i++) {
        if (alias_name_match(digi_aliases[i].name, addr)
            && ssid <= digi_aliases[i].max_ssid) {
            return i;
        }
    }
    return -1;
}

// ---- Dedup table -------------------------------------------------------
//
// Identifies packets by a 32-bit FNV-1a hash over (dst addr + src addr +
// control/PID/info). The digipath is intentionally excluded: a wide-area
// digi's repeat differs from ours only in the path (decremented SSID,
// set H-bit), and we want it to match.

#define DIGI_DEDUP_SIZE 16
#define DIGI_DEDUP_TTL_MS 30000

typedef struct {
    uint32_t hash;             // 0 means "slot empty"
    absolute_time_t last;
} digi_dedup_entry_t;

static digi_dedup_entry_t digi_dedup[DIGI_DEDUP_SIZE];

static int find_control_offset(const uint8_t *packet, int len)
{
    int i = AX25_ADDR_LEN - 1;  // SSID byte of dst
    while (i < len) {
        if (packet[i] & 1) return i + 1;  // control byte follows
        i += AX25_ADDR_LEN;
    }
    return -1;
}

static uint32_t fnv1a(const uint8_t *data, int len)
{
    uint32_t h = 0x811c9dc5u;
    for (int i = 0; i < len; i++) {
        h ^= data[i];
        h *= 0x01000193u;
    }
    return h;
}

// FCS is NOT included in the hash — caller may pass packet with or without it.
static uint32_t digi_packet_hash(const uint8_t *packet, int len)
{
    int ctrl = find_control_offset(packet, len);
    if (ctrl < 0 || ctrl >= len) return 0;

    uint32_t h = 0x811c9dc5u;
    // dst addr (7) + src addr (7)
    for (int i = 0; i < AX25_ADDR_LEN * 2 && i < len; i++) {
        h ^= packet[i];
        h *= 0x01000193u;
    }
    // control + PID + info (skip digipath)
    for (int i = ctrl; i < len; i++) {
        h ^= packet[i];
        h *= 0x01000193u;
    }
    if (h == 0) h = 1;  // reserve 0 for "empty"
    return h;
}

// Return the last-seen time for `hash`, or nil_time if not present or expired.
static absolute_time_t digi_dedup_lookup(uint32_t hash)
{
    if (!hash) return nil_time;
    absolute_time_t now = get_absolute_time();
    for (int i = 0; i < DIGI_DEDUP_SIZE; i++) {
        if (digi_dedup[i].hash != hash) continue;
        int64_t age = absolute_time_diff_us(digi_dedup[i].last, now);
        if (age > (int64_t)DIGI_DEDUP_TTL_MS * 1000) {
            digi_dedup[i].hash = 0;  // expire
            return nil_time;
        }
        return digi_dedup[i].last;
    }
    return nil_time;
}

static void digi_dedup_record_hash(uint32_t hash)
{
    if (!hash) return;
    absolute_time_t now = get_absolute_time();

    // Update if already present.
    for (int i = 0; i < DIGI_DEDUP_SIZE; i++) {
        if (digi_dedup[i].hash == hash) {
            digi_dedup[i].last = now;
            return;
        }
    }
    // Find empty slot, or evict oldest.
    int evict = 0;
    for (int i = 0; i < DIGI_DEDUP_SIZE; i++) {
        if (digi_dedup[i].hash == 0) { evict = i; break; }
        if (absolute_time_diff_us(digi_dedup[i].last, digi_dedup[evict].last) > 0) {
            evict = i;
        }
    }
    digi_dedup[evict].hash = hash;
    digi_dedup[evict].last = now;
}

// Public: called from decode.c output_packet() for every valid frame.
// `len` is the logical packet length (NO trailing FCS).
void digipeat_record_rx(const uint8_t *packet, int len)
{
    if (len < AX25_MIN_LEN) return;
    digi_dedup_record_hash(digi_packet_hash(packet, len));
}

// ---- Pending-TX slots (hold-off) ---------------------------------------
//
// PORT_N=1 so file-scope storage is fine. If PORT_N grows these should
// move into tnc_t (or become an array indexed by tp->port).

#define DIGI_PENDING_SLOTS 2

typedef struct {
    bool active;
    tnc_t *tp;
    uint32_t hash;
    absolute_time_t scheduled_at;
    absolute_time_t fire_at;
    uint16_t len;                // bytes valid in frame[], no FCS
    uint8_t frame[DATA_LEN];
} digi_pending_t;

static digi_pending_t digi_pending[DIGI_PENDING_SLOTS];

// Schedule a frame for hold-off retransmit. With DIGI_HOLDOFF_MS == 0
// this sends immediately. Returns true on success.
static bool digi_schedule_tx(tnc_t *tp, const uint8_t *frame, int len)
{
    uint32_t hash = digi_packet_hash(frame, len);

    if (DIGI_HOLDOFF_MS == 0) {
        if (!send_packet(tp, (uint8_t *)frame, len)) return false;
        digi_dedup_record_hash(hash);
        return true;
    }

    for (int i = 0; i < DIGI_PENDING_SLOTS; i++) {
        digi_pending_t *s = &digi_pending[i];
        if (s->active) continue;
        if (len > (int)sizeof(s->frame)) return false;

        s->active = true;
        s->tp = tp;
        s->hash = hash;
        s->scheduled_at = get_absolute_time();
        s->fire_at = make_timeout_time_ms(DIGI_HOLDOFF_MS);
        s->len = (uint16_t)len;
        memcpy(s->frame, frame, len);
        return true;
    }

    printf("Digipeat: pending slots full — drop\n");
    return false;
}

// Public: called from the main loop. Fires any pending TX whose hold-off
// has elapsed, suppressing if a duplicate of the same packet has been
// received from someone else since we scheduled.
void digipeat_poll(void)
{
    absolute_time_t now = get_absolute_time();

    for (int i = 0; i < DIGI_PENDING_SLOTS; i++) {
        digi_pending_t *s = &digi_pending[i];
        if (!s->active) continue;
        if (absolute_time_diff_us(now, s->fire_at) > 0) continue;  // not yet

        absolute_time_t last = digi_dedup_lookup(s->hash);
        bool suppress = !is_nil_time(last)
            && absolute_time_diff_us(s->scheduled_at, last) > 0;

        if (suppress) {
            printf("Digipeat: suppressed (another digi was first)\n");
        } else {
            if (send_packet(s->tp, s->frame, s->len)) {
                digi_dedup_record_hash(s->hash);
            } else {
                printf("Digipeat: send_queue full — drop\n");
            }
        }

        s->active = false;
    }
}

// ---- digipeat() --------------------------------------------------------

void digipeat(tnc_t *tp, uint8_t *packet, int len)
{

    if (!digi_init_done) digipeat_init();

    if (len < AX25_MIN_LEN) return;
    if (!ax25_ui(packet, len)) return;

    // Never digipeat our own packets.
    if (ax25_callcmp(&param.mycall, &packet[AX25_ADDR_LEN])) return;

    int offset = AX25_ADDR_LEN;                       // src addr
    if (packet[offset + SSID_LOC] & 1) return;        // no digis in path

    offset += AX25_ADDR_LEN;                          // first digi
    int digis = 1;

    while (offset + AX25_ADDR_LEN <= len) {

        if (!(packet[offset + SSID_LOC] & H_BIT)) {   // first non-repeated digi

            // 1) Exact MYCALL / MYALIAS match — set H bit, re-transmit.
            if (ax25_callcmp(&param.mycall, &packet[offset])
                || ax25_callcmp(&param.myalias, &packet[offset])) {

                packet[offset + SSID_LOC] |= H_BIT;
                digi_schedule_tx(tp, packet, len - 2);
                packet[offset + SSID_LOC] &= ~H_BIT;
                printf("Digipeat: relayed (direct match)\n");
                return;
            }

            // 2) WIDEn-N alias — decrement SSID, set H bit when it hits 0.
            int idx = find_alias_match(&packet[offset]);
            if (idx >= 0) {
                uint8_t orig = packet[offset + SSID_LOC];
                uint8_t old_ssid = (orig >> 1) & 0x0f;
                uint8_t new_ssid = old_ssid - 1;

                // Keep extension bit (0) and reserved bits (5,6); rewrite SSID.
                packet[offset + SSID_LOC] = (orig & 0x61) | (new_ssid << 1);
                if (new_ssid == 0) packet[offset + SSID_LOC] |= H_BIT;

                digi_schedule_tx(tp, packet, len - 2);
                packet[offset + SSID_LOC] = orig;

                int n = strip_trailing_spaces(digi_aliases[idx].name);
                printf("Digipeat: relayed via %.*s-%u -> %.*s-%u%s\n",
                       n, digi_aliases[idx].name, old_ssid,
                       n, digi_aliases[idx].name, new_ssid,
                       new_ssid == 0 ? "*" : "");
                return;
            }

            return;  // first non-repeated digi did not match — stop
        }

        if (++digis >= MAX_DIGIPEATER) return;
        offset += AX25_ADDR_LEN;
    }
}
