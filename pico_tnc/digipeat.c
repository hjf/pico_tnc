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

// Callsign the digipeater identifies as in the path.  Independent of the
// TNC's terminal-typed MYCALL (KISS clients never set that, and a multi-port
// build could have different per-port calls in the future).  If unset we
// refuse to digipeat — otherwise we'd insert a blank callsign into the path.
#ifndef DIGI_MYCALL
#define DIGI_MYCALL ""
#endif

// Optional alias the digi also matches on.  When a packet's next unused digi
// is this address, we substitute DIGI_MYCALL into the path (per §4.3.1).
#ifndef DIGI_MYALIAS
#define DIGI_MYALIAS ""
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
// 0 = spec-compliant YOLO: transmit as soon as the channel clears, count on
// FM capture to settle collisions with other digis (APRS Protocol Reference
// §3 / WB2OSZ §3.2.1).  Positive = smart fill-in: wait this long, re-check
// dedup, and suppress our copy if a colocated wide-area digi already covered
// the frame.  See the plan in plan-for-how-to-toasty-barto.md for context.
#ifndef DIGI_HOLDOFF_MS
#define DIGI_HOLDOFF_MS 1500
#endif

// Hard ceiling on time a frame may sit in pending state waiting for the
// channel to clear.  Beyond this we drop rather than block a slot forever.
// Under the dedup TTL so it can't fight with §3.3 suppression.
#ifndef DIGI_PENDING_MAX_MS
#define DIGI_PENDING_MAX_MS 5000
#endif

#ifndef DIGI_LOCAL_ORIGIN_TTL_MS
#define DIGI_LOCAL_ORIGIN_TTL_MS (30 * 60 * 1000)
#endif

#define MAX_DIGI_ALIASES 8
#define DIGI_LOCAL_ORIGIN_SIZE 32

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

// Resolved at init from DIGI_MYCALL / DIGI_MYALIAS compile defs.  digi_active
// is false if DIGI_MYCALL was unset or unparseable — in that case digipeat()
// is a no-op (we'd otherwise stamp a blank callsign into the path).
static callsign_t digi_mycall;
static callsign_t digi_myalias;
static bool digi_active;
static bool digi_has_myalias;

typedef struct {
    callsign_t call;
    absolute_time_t last;
} digi_local_origin_entry_t;

static digi_local_origin_entry_t digi_local_origins[DIGI_LOCAL_ORIGIN_SIZE];

static bool local_origin_entry_fresh(const digi_local_origin_entry_t *e, absolute_time_t now)
{
    if (!e->call.call[0] || is_nil_time(e->last)) return false;
    int64_t age = absolute_time_diff_us(e->last, now);
    return age >= 0 && age <= (int64_t)DIGI_LOCAL_ORIGIN_TTL_MS * 1000;
}

bool digipeat_addr_is_local_origin(const uint8_t *addr)
{
    absolute_time_t now = get_absolute_time();

    for (int i = 0; i < DIGI_LOCAL_ORIGIN_SIZE; i++) {
        digi_local_origin_entry_t *e = &digi_local_origins[i];
        if (!local_origin_entry_fresh(e, now)) {
            e->call.call[0] = '\0';
            e->last = nil_time;
            continue;
        }

        if (ax25_callcmp(&e->call, (uint8_t *)addr)) return true;
    }

    return false;
}

void digipeat_record_local_origin(const uint8_t *packet, int len)
{
    if (len < AX25_ADDR_LEN * 2) return;

    callsign_t src = {0};
    for (int i = 0; i < 6; i++) {
        src.call[i] = (char)(packet[AX25_ADDR_LEN + i] >> 1);
    }
    src.ssid = (packet[AX25_ADDR_LEN + SSID_LOC] >> 1) & 0x0f;

    absolute_time_t now = get_absolute_time();
    int evict = -1;

    for (int i = 0; i < DIGI_LOCAL_ORIGIN_SIZE; i++) {
        digi_local_origin_entry_t *e = &digi_local_origins[i];

        if (!local_origin_entry_fresh(e, now)) {
            if (evict < 0) evict = i;
            continue;
        }

        if (ax25_callcmp(&e->call, (uint8_t *)(packet + AX25_ADDR_LEN))) {
            e->last = now;
            return;
        }
    }

    if (evict < 0) {
        evict = 0;
        for (int i = 1; i < DIGI_LOCAL_ORIGIN_SIZE; i++) {
            if (absolute_time_diff_us(digi_local_origins[i].last,
                                      digi_local_origins[evict].last) > 0) {
                evict = i;
            }
        }
    }

    digi_local_origins[evict].call = src;
    digi_local_origins[evict].last = now;
}

// Format a callsign_t as "CALL" or "CALL-SSID" for log lines.  Returns the
// number of characters written (not counting the NUL).  `out` must hold at
// least 10 bytes (6 call + '-' + 2 ssid + NUL).
static int format_callsign(char *out, int size, const callsign_t *c)
{
    if (size <= 0) return 0;
    int n = 6;
    while (n > 0 && c->call[n - 1] == ' ') n--;
    int written = 0;
    for (int i = 0; i < n && written < size - 1; i++) {
        out[written++] = c->call[i];
    }
    if (c->ssid && written < size - 1) {
        written += snprintf(out + written, size - written, "-%u", c->ssid);
    }
    if (written < size) out[written] = '\0';
    else out[size - 1] = '\0';
    return written;
}

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

// Parse "CALL[-SSID]" into a callsign_t.  Returns true on success.
// Accepts up to 6 alphanumeric chars (case-insensitive), optional "-N" SSID
// in 0..15.  Leading whitespace skipped; trailing whitespace allowed.
static bool digi_parse_call(const char *in, callsign_t *out)
{
    if (!in) return false;
    while (*in == ' ' || *in == '\t') in++;
    memset(out->call, ' ', 6);
    out->ssid = 0;

    int pos = 0;
    while (*in && *in != '-' && *in != ' ' && *in != '\t' && pos < 6) {
        char c = *in++;
        if (c >= 'a' && c <= 'z') c -= 32;
        if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) return false;
        out->call[pos++] = c;
    }
    if (pos == 0) return false;

    if (*in == '-') {
        in++;
        if (*in < '0' || *in > '9') return false;
        int ssid = 0;
        while (*in >= '0' && *in <= '9') ssid = ssid * 10 + (*in++ - '0');
        if (ssid > 15) return false;
        out->ssid = (uint8_t)ssid;
    }
    return true;
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

    // Resolve our own callsign + optional alias from build-time config.
    digi_active = digi_parse_call(DIGI_MYCALL, &digi_mycall);
    digi_has_myalias = (DIGI_MYALIAS[0] != '\0')
                       && digi_parse_call(DIGI_MYALIAS, &digi_myalias);

    digi_init_done = true;

    if (digi_active) {
        char tmp[10];
        format_callsign(tmp, sizeof(tmp), &digi_mycall);
        printf("Digipeat: Using %s as MYCALL", tmp);
        if (digi_has_myalias) {
            format_callsign(tmp, sizeof(tmp), &digi_myalias);
            printf(" MYALIAS %s", tmp);
        }
    } else {
        printf("Digipeat: DISABLED — DIGI_MYCALL is unset or invalid (%s)", DIGI_MYCALL);
    }
    printf(", %d aliases", n_digi_aliases);
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

// True if the address (shifted) is one of the well-known APRS-IS path tags:
// TCPIP, TCPXX, NOGATE, RFONLY.  Used to suppress re-digipeating packets
// that originated on the internet or are explicitly marked don't-RF.
static bool is_path_tag(const uint8_t *addr)
{
    static const char *const tags[] = { "TCPIP ", "TCPXX ", "NOGATE", "RFONLY" };
    for (int t = 0; t < 4; t++) {
        if (alias_name_match(tags[t], addr)) return true;
    }
    return false;
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
// 8 s window: long enough to catch flood-back from a colocated wide-area digi
// (arrives 1-3 s after our copy), short enough that APRS message retries
// (typically >=30 s with backoff) get through and the remote can re-ACK.
#define DIGI_DEDUP_TTL_MS 8000

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
    // dst addr (call only, 6 bytes) + src addr (7 bytes incl. SSID).
    // Per WB2OSZ §4.2(a), destination SSID is excluded from the dedup key.
    for (int i = 0; i < AX25_ADDR_LEN * 2 && i < len; i++) {
        if (i == SSID_LOC) continue;  // destination SSID byte
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

// Queue a frame in the pending pool.  Even with DIGI_HOLDOFF_MS=0 we route
// through pending so digipeat_poll() can do the dedup re-check and the
// channel-clear gate just before keying up — otherwise the frame would
// block inside send_packet's CSMA state and we'd never re-evaluate dedup
// while waiting for DCD to drop.
static bool digi_schedule_tx(tnc_t *tp, const uint8_t *frame, int len)
{
    uint32_t hash = digi_packet_hash(frame, len);

    for (int i = 0; i < DIGI_PENDING_SLOTS; i++) {
        digi_pending_t *s = &digi_pending[i];
        if (s->active) continue;
        if (len > (int)sizeof(s->frame)) return false;

        s->active = true;
        s->tp = tp;
        s->hash = hash;
        s->scheduled_at = get_absolute_time();
        s->fire_at = make_timeout_time_ms(DIGI_HOLDOFF_MS);  // 0 ms is fine
        s->len = (uint16_t)len;
        memcpy(s->frame, frame, len);
        return true;
    }

    printf("Digipeat: pending slots full — drop\n");
    return false;
}

// Called from the main loop.  Gating order:
//   1. hold-off timer            (0 ms = immediate; positive = smart fill-in)
//   2. dedup re-check            (suppress if another digi already covered)
//   3. stale-packet timeout      (drop rather than block a slot forever)
//   4. channel-clear gate        (wait for DCD low; closes the long-packet race)
//   5. send_packet               (TXes promptly since DCD is currently low)
void digipeat_poll(void)
{
    absolute_time_t now = get_absolute_time();

    for (int i = 0; i < DIGI_PENDING_SLOTS; i++) {
        digi_pending_t *s = &digi_pending[i];
        if (!s->active) continue;

        // 1. Hold-off timer.
        if (absolute_time_diff_us(now, s->fire_at) > 0) continue;

        // 2. Dedup re-check: did anyone retransmit this frame after we scheduled?
        absolute_time_t last = digi_dedup_lookup(s->hash);
        bool suppress = !is_nil_time(last)
            && absolute_time_diff_us(s->scheduled_at, last) > 0;
        if (suppress) {
            printf("Digipeat: suppressed (another digi was first)\n");
            s->active = false;
            continue;
        }

        // 3. Stale-packet timeout.  Channel may be permanently busy, or the
        //    frame may simply be too old to be worth digipeating.
        if (absolute_time_diff_us(s->scheduled_at, now)
                > (int64_t)DIGI_PENDING_MAX_MS * 1000) {
            printf("Digipeat: pending timeout — drop\n");
            s->active = false;
            continue;
        }

        // 4. Channel-clear gate.  Hold the frame in our pending slot until
        //    DCD drops; this is the fix for the long-packet race where
        //    send_packet's own CSMA wait would keep us blocked past the
        //    point where the dedup table picks up the wide-area's copy.
        if (s->tp->dcd) continue;

        // 5. TX.  send_packet_now bypasses p-persistence so we key up the
        //    instant DCD drops — colocated digis then collide on the air
        //    and FM capture decides (APRS Protocol Reference §3 / WB2OSZ
        //    §3.2.1).  Smart fill-in already filtered the should-we-TX
        //    question above; no value in adding random CSMA delay here.
        if (send_packet_now(s->tp, s->frame, s->len)) {
            digi_dedup_record_hash(s->hash);
        } else {
            printf("Digipeat: send_queue full — drop\n");
        }
        s->active = false;
    }
}

// ---- digipeat() --------------------------------------------------------

void digipeat(tnc_t *tp, uint8_t *packet, int len)
{

    if (!digi_init_done) digipeat_init();
    if (!digi_active) return;                         // DIGI_MYCALL unset

    if (len < AX25_MIN_LEN) return;
    if (!ax25_ui(packet, len)) return;

    // Never digipeat packets sourced by or addressed to ourselves — that
    // would loop our own retransmissions (which now carry digi_mycall in
    // the path) right back through digipeat().
    if (ax25_callcmp(&digi_mycall, &packet[AX25_ADDR_LEN])) return;
    if (ax25_callcmp(&digi_mycall, packet)) return;

    // If destination is a station recently seen originating locally
    // (KISS/AGWPE), suppress RF digipeating to avoid duplex starvation.
    if (digipeat_addr_is_local_origin(packet)) return;

    // Suppress re-digipeating of APRS-IS-originated traffic.  A colocated
    // wide-area iGate's gate-IS->RF wraps the original frame in a third-
    // party '}' envelope; relaying it duplicates the wide-area's RF copy
    // so listeners hear two of every IS-routed message.  Same logic also
    // catches packets whose path carries TCPIP/TCPXX/NOGATE/RFONLY.
    int control = find_control_offset(packet, len);
    if (control < 0) return;
    if (control + 2 < len - 2 && packet[control + 2] == '}') {
        printf("Digipeat: skip third-party (} prefix)\n");
        return;
    }
    for (int off = AX25_ADDR_LEN * 2; off + AX25_ADDR_LEN <= control; off += AX25_ADDR_LEN) {
        if (is_path_tag(packet + off)) {
            char name[7];
            for (int i = 0; i < 6; i++) name[i] = (char)(packet[off + i] >> 1);
            name[6] = '\0';
            // trim trailing space (TCPIP / TCPXX are 5 chars)
            for (int i = 5; i >= 0 && name[i] == ' '; i--) name[i] = '\0';
            printf("Digipeat: skip path tag %s\n", name);
            return;
        }
    }

    int offset = AX25_ADDR_LEN;                       // src addr
    if (packet[offset + SSID_LOC] & 1) return;        // no digis in path

    offset += AX25_ADDR_LEN;                          // first digi
    int digis = 1;                                    // 1-based slot index

    while (digis <= MAX_DIGIPEATER && offset + AX25_ADDR_LEN <= len) {

        if (packet[offset + SSID_LOC] & H_BIT) {
            // already-used digi — step past
            digis++;
            offset += AX25_ADDR_LEN;
            continue;
        }

        // First non-repeated digi.  Decide what kind of match this is and
        // build the outgoing frame in a stack buffer (length may grow by 7).
        int logical_len = len - 2;                    // drop FCS
        if (logical_len <= offset) return;
        uint8_t out[DATA_LEN];
        if (logical_len + AX25_ADDR_LEN > (int)sizeof(out)) return;
        int out_len;

        char mycall_str[10];
        format_callsign(mycall_str, sizeof(mycall_str), &digi_mycall);

        if (ax25_callcmp(&digi_mycall, &packet[offset])) {
            // MYCALL exact match.  Just mark used.
            memcpy(out, packet, logical_len);
            out[offset + SSID_LOC] |= H_BIT;
            out_len = logical_len;
            printf("Digipeat: relayed (direct match for %s)\n", mycall_str);

        } else if (digi_has_myalias
                   && ax25_callcmp(&digi_myalias, &packet[offset])) {
            // §4.3.1: MYALIAS match — substitute MYCALL, mark used,
            // preserve the end-of-address-field bit.
            memcpy(out, packet, logical_len);
            uint8_t orig = out[offset + SSID_LOC];
            ax25_mkax25addr(out + offset, &digi_mycall);
            out[offset + SSID_LOC] |= H_BIT | (orig & 0x01);
            out_len = logical_len;
            char myalias_str[10];
            format_callsign(myalias_str, sizeof(myalias_str), &digi_myalias);
            printf("Digipeat: relayed via %s -> %s*\n", myalias_str, mycall_str);

        } else {
            int idx = find_alias_match(&packet[offset]);
            if (idx < 0) return;                      // not for us

            uint8_t orig = packet[offset + SSID_LOC];
            uint8_t old_n = (orig >> 1) & 0x0f;
            int name_n = strip_trailing_spaces(digi_aliases[idx].name);

            if (old_n >= 2 && digis < MAX_DIGIPEATER) {
                // §4.3.2(a): insert MYCALL (marked used) before WIDEn-N,
                // decrement N on the alias.  Length grows by 7.
                memcpy(out, packet, offset);
                ax25_mkax25addr(out + offset, &digi_mycall);
                // Inserted MYCALL is not the end-of-address-field (the alias
                // follows it), so clear bit 0 and set the H bit.
                out[offset + SSID_LOC] = (out[offset + SSID_LOC] & ~0x01) | H_BIT;
                memcpy(out + offset + AX25_ADDR_LEN, packet + offset,
                       logical_len - offset);
                // Decrement N on the alias (now shifted right by 7).
                // Keep its original reserved/extension bits, leave H=0.
                uint8_t *alias_ssid = &out[offset + AX25_ADDR_LEN + SSID_LOC];
                *alias_ssid = (orig & 0x61) | ((old_n - 1) << 1);
                out_len = logical_len + AX25_ADDR_LEN;
                printf("Digipeat: relayed via %.*s-%u -> %s*,%.*s-%u\n",
                       name_n, digi_aliases[idx].name, old_n,
                       mycall_str,
                       name_n, digi_aliases[idx].name, old_n - 1);

            } else if (old_n >= 2) {
                // §4.3.2(a) fallback: path is at the 8-digi limit, can't
                // insert MYCALL.  Just decrement N in place.
                memcpy(out, packet, logical_len);
                out[offset + SSID_LOC] = (orig & 0x61) | ((old_n - 1) << 1);
                out_len = logical_len;
                printf("Digipeat: relayed via %.*s-%u -> %.*s-%u (8-digi limit, no %s)\n",
                       name_n, digi_aliases[idx].name, old_n,
                       name_n, digi_aliases[idx].name, old_n - 1,
                       mycall_str);

            } else {
                // §4.3.2(b): N=1.  Replace WIDEn-1 with MYCALL marked used,
                // preserving the original end-of-address-field bit.  Length
                // unchanged.
                memcpy(out, packet, logical_len);
                ax25_mkax25addr(out + offset, &digi_mycall);
                out[offset + SSID_LOC] |= H_BIT | (orig & 0x01);
                out_len = logical_len;
                printf("Digipeat: relayed via %.*s-1 -> %s*\n",
                       name_n, digi_aliases[idx].name, mycall_str);
            }
        }

        digi_schedule_tx(tp, out, out_len);
        return;
    }
}
