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
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "class/cdc/cdc_device.h"
#include "pico/sync.h"

#include "usb_output.h"
#include "usb_input.h"
#include "ax25.h"
#include "tnc.h"
#include "tty.h"
#include "flash.h"
#include "receive.h"
#include "beacon.h"
#include "digipeat.h"

typedef struct CMD {
    uint8_t *name;
    int len;
    bool (*func)(tty_t *ttyp, uint8_t *buf, int len);
} cmd_t;

static char const help_str[] =
    "\r\n"
    "Commands are Case Insensitive\r\n"
    "Use Backspace Key (BS) for Correction\r\n"
    "Use the DISP command to desplay all options\r\n"
    "Connect GPS for APRS Operation, (GP4/GP5/9600bps)\r\n"
    "Connect to Terminal for Command Interpreter, (USB serial or GP0/GP1/115200bps)\r\n"
    "\r\n"
    "Commands (with example):\r\n"
    "MYCALL (mycall jn1dff-2)\r\n"
    "UNPROTO (unproto jn1dff-14 v jn1dff-1) - 3 digis max\r\n"
    "BTEXT (btext Bob)-APRS beacon comment, 100 chars max\r\n"
    "BEACON (beacon every n, beacon now, beacon off), 0<n<=60\r\n"
    "MONitor (mon all,mon me, or mon off)\r\n"
    "DIGIpeat (digi on or digi off)\r\n"
    "MYALIAS (myalias RELAY)\r\n"
#if !PICO_TNC_PARENT_INTEGRATION
    "DIGIPATH (digipath WIDE1-1,WIDE2-1)\r\n"
    "DIGIHOLD (digihold 1500) milliseconds, 0..5000\r\n"
    "TOCALL (tocall APRS)\r\n"
    "BPATH (bpath WIDE1-1,WIDE2-1; bpath % clears)\r\n"
    "BPOSITION (bposition -27.2963412,-58.6176194)\r\n"
    "BSYMBOL (bsymbol /#) table and symbol code\r\n"
    "BPREVIEW shows the complete APRS packet without transmitting\r\n"
#endif
    "PERM (PERM)\r\n"
    "ECHO (echo on or echo off)\r\n"
    "GPS (gps $GPGGA or gps $GPGLL or gps $GPRMC)\r\n"
    "TRace (tr xmit or tr rcv) - For debugging only\r\n"
    "TXDELAY (txdelay n 0<n<201 n is number of delay flags to send)\r\n"
    "CALIBRATE (Calibrate Mode - Testing Only)\r\n"
    "CONverse (con)\r\n"
    "\r\n";



enum STATE_CALLSIGN {
    CALL = 0,
    HYPHEN,
    SSID1,
    SSID2,
    SPACE,
    END,
};



enum TRACE {
    TR_OFF = 0,
    TR_XMIT,
    TR_RCV,
};

static const uint8_t *gps_str[] = {
    "$GPGGA",
    "$GPGLL",
    "$GPRMC",
};

// indicate converse mode
bool converse_mode = false;
// indicate calibrate mode
bool calibrate_mode = false;
uint8_t calibrate_idx = 0;

static uint8_t *read_call(uint8_t *buf, callsign_t *c)
{
    callsign_t cs;
    int i, j;
    int state = CALL;
    bool error = false;

    cs.call[i] = '\0';
    for (i = 1; i < 6; i++) cs.call[i] = ' ';
    cs.ssid = 0;

    // callsign
    j = 0;
    for (i = 0; buf[i] && state != END; i++) {
        int ch = buf[i];

        switch (state) {

            case CALL:
                if (isalnum(ch)) {
                    cs.call[j++] = toupper(ch);
                    if (j >= 6) state = HYPHEN;
                    break;
                } else if (ch == '-') {
                    state = SSID1;
                    break;
                } else if (ch != ' ') {
                    error = true;
                }
                state = END;
                break;

            case HYPHEN:
                if (ch == '-') {
                    state = SSID1;
                    break;
                }
                if (ch != ' ') {
                    error = true;
                }
                state = END;
                break;

            case SSID1:
                if (isdigit(ch)) {
                    cs.ssid = ch - '0';
                    state = SSID2;
                    break;
                }
                error = true;
                state = END;
                break;

            case SSID2:
                if (isdigit(ch)) {
                    cs.ssid *= 10;
                    cs.ssid += ch - '0';
                    state = SPACE;
                    break;
                }
                /* FALLTHROUGH */

            case SPACE:
                if (ch != ' ') error = true;
                state = END;
        }
    }

    if (cs.ssid > 15) error = true;

    if (error) return NULL;

    memcpy(c, &cs, sizeof(cs));

    return &buf[i];
}

static int callsign2ascii(uint8_t *buf, callsign_t *c)
{
    int i;

    if (!c->call[0]) {
        memcpy(buf, "NOCALL", 7);
        
        return 6;
    }

    for (i = 0; i < 6; i++) {
        int ch = c->call[i];

        if (ch == ' ') break;

        buf[i] = ch;
    }

    if (c->ssid > 0) {
        buf[i++] = '-';

        if (c->ssid > 9) {
            buf[i++] = '1';
            buf[i++] = c->ssid - 10 + '0';
        } else {
            buf[i++] = c->ssid + '0';
        }
    }

    buf[i] = '\0';

    return i;
}

static bool cmd_mycall(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {
        if (read_call(buf, &param.mycall) == NULL) return false;
        digipeat_config_changed();
        return true;

        //usb_write(buf, len);
        //usb_write("\r\n", 2);

    } else {
        uint8_t temp[10];

        tty_write_str(ttyp, "MYCALL ");
        tty_write(ttyp, temp, callsign2ascii(temp, &param.mycall));
        tty_write_str(ttyp, "\r\n");
    }

    return true;
}

static bool cmd_unproto(tty_t *ttyp, uint8_t *buf, int len)
{
    int i;
    uint8_t *p;


    if (buf && buf[0]) {
        callsign_t parsed[UNPROTO_N] = {0};

        p = read_call(buf, &parsed[0]);
        if (p == NULL) return false;

        while (*p == ' ') p++;
        if (!*p) {
            memcpy(param.unproto, parsed, sizeof(parsed));
            return true;
        }

        // Accept "VIA" or "V" as the keyword before the digipeater list
        if (toupper(*p) != 'V') return false;
        p++;
        // Skip the rest of "VIA" if spelled out
        if (toupper(*p) == 'I') { p++; if (toupper(*p) == 'A') p++; }
        if (*p != ' ' && *p != '\0') return false;
        while (*p == ' ') p++;

        // Parse comma-separated or "V"-separated digipeater list
        for (i = 1; *p; i++) {
            uint8_t call[10];
            int call_len = 0;

            if (i >= UNPROTO_N) return false;
            while (p[call_len] && p[call_len] != ',' && p[call_len] != ' ') {
                if (call_len >= (int)sizeof(call) - 1) return false;
                call[call_len] = p[call_len];
                call_len++;
            }
            if (!call_len) return false;
            call[call_len] = '\0';
            uint8_t *end = read_call(call, &parsed[i]);
            if (end == NULL || *end) return false;
            p += call_len;

            while (*p == ' ') p++;
            if (!*p) break;
            if (*p == ',') {
                p++;
                while (*p == ' ') p++;
                if (!*p) return false;
                continue;
            }
            // Handle "V" or "VIA" between digipeaters (original format)
            if (toupper(*p) == 'V') {
                p++;
                if (toupper(*p) == 'I') { p++; if (toupper(*p) == 'A') p++; }
                if (*p != ' ') return false;
                while (*p == ' ') p++;
                if (!*p) return false;
                continue;
            }
            return false;
        }

        memcpy(param.unproto, parsed, sizeof(parsed));

    } else {

        tty_write_str(ttyp, "UNPROTO ");

        for (i = 0; i < 4; i++) {
            uint8_t temp[10];

            if (!param.unproto[i].call[0]) break;

            if (i > 0) tty_write_str(ttyp, " V ");
            tty_write(ttyp, temp, callsign2ascii(temp, &param.unproto[i]));
        }

        tty_write_str(ttyp, "\r\n");
    }

    return true;
}

#if !PICO_TNC_PARENT_INTEGRATION
static bool cmd_tocall(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {
        return read_call(buf, &param.unproto[0]) != NULL;
    }

    uint8_t value[10];
    tty_write_str(ttyp, "TOCALL ");
    tty_write(ttyp, value, callsign2ascii(value, &param.unproto[0]));
    tty_write_str(ttyp, "\r\n");
    return true;
}

static bool cmd_bpath(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {
        if (buf[0] == '%' && len == 1) {
            for (int i = 1; i < UNPROTO_N; i++) param.unproto[i].call[0] = '\0';
            return true;
        }

        uint8_t command[CMD_BUF_LEN + 1];
        int offset = callsign2ascii(command, &param.unproto[0]);
        if (offset + 3 + len >= (int)sizeof(command)) return false;
        memcpy(command + offset, " V ", 3);
        memcpy(command + offset + 3, buf, len);
        command[offset + 3 + len] = '\0';
        return cmd_unproto(ttyp, command, offset + 3 + len);
    }

    tty_write_str(ttyp, "BPATH ");
    for (int i = 1; i < UNPROTO_N; i++) {
        if (!param.unproto[i].call[0]) break;
        uint8_t value[10];
        if (i > 1) tty_write_char(ttyp, ',');
        tty_write(ttyp, value, callsign2ascii(value, &param.unproto[i]));
    }
    tty_write_str(ttyp, "\r\n");
    return true;
}

static bool parse_decimal_e7(const char **input, int32_t *value)
{
    const char *p = *input;
    while (*p == ' ') p++;
    int sign = 1;
    if (*p == '-' || *p == '+') {
        if (*p++ == '-') sign = -1;
    }
    if (!isdigit((unsigned char)*p)) return false;

    int64_t whole = 0;
    while (isdigit((unsigned char)*p)) {
        whole = whole * 10 + (*p++ - '0');
        if (whole > 180) return false;
    }

    int64_t fraction = 0;
    int digits = 0;
    if (*p == '.') {
        p++;
        while (isdigit((unsigned char)*p)) {
            if (digits >= 7) return false;
            fraction = fraction * 10 + (*p++ - '0');
            digits++;
        }
    }
    while (digits++ < 7) fraction *= 10;

    int64_t scaled = whole * 10000000 + fraction;
    if (scaled > INT32_MAX) return false;
    *value = (int32_t)(sign * scaled);
    *input = p;
    return true;
}

static bool cmd_bposition(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {
        if (buf[0] == '%' && len == 1) {
            param.beacon_position_set = 0;
            return true;
        }

        const char *p = (char *)buf;
        int32_t lat, lon;
        if (!parse_decimal_e7(&p, &lat) || *p++ != ',' || !parse_decimal_e7(&p, &lon)) {
            return false;
        }
        while (*p == ' ') p++;
        if (*p || lat < -900000000 || lat > 900000000
                || lon < -1800000000 || lon > 1800000000) {
            return false;
        }
        param.beacon_lat_e7 = lat;
        param.beacon_lon_e7 = lon;
        param.beacon_position_set = 1;
    } else {
        tty_write_str(ttyp, "BPOSITION ");
        if (param.beacon_position_set) {
            char value[32];
            int64_t lat = param.beacon_lat_e7;
            int64_t lon = param.beacon_lon_e7;
            int size = snprintf(value, sizeof(value), "%s%lld.%07lld,%s%lld.%07lld",
                                lat < 0 ? "-" : "", llabs(lat) / 10000000,
                                llabs(lat) % 10000000,
                                lon < 0 ? "-" : "", llabs(lon) / 10000000,
                                llabs(lon) % 10000000);
            tty_write(ttyp, (uint8_t *)value, size);
        }
        tty_write_str(ttyp, "\r\n");
    }
    return true;
}

static bool cmd_bsymbol(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {
        if (len != 2 || (buf[0] != '/' && buf[0] != '\\')
                || buf[1] < '!' || buf[1] > '~') {
            return false;
        }
        param.beacon_symbol_table = buf[0];
        param.beacon_symbol_code = buf[1];
    } else {
        tty_write_str(ttyp, "BSYMBOL ");
        tty_write_char(ttyp, param.beacon_symbol_table);
        tty_write_char(ttyp, param.beacon_symbol_code);
        tty_write_str(ttyp, "\r\n");
    }
    return true;
}

static bool cmd_bpreview(tty_t *ttyp, uint8_t *buf, int len)
{
    uint8_t packet[BEACON_PREVIEW_LEN];
    int size = beacon_format_preview(packet, sizeof(packet));
    tty_write_str(ttyp, "BPREVIEW ");
    if (size > 0 && size < (int)sizeof(packet)) tty_write(ttyp, packet, size);
    tty_write_str(ttyp, "\r\n");
    return true;
}
#endif

static bool cmd_btext(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {

        uint8_t *p = buf;
        int i;

        if (buf[0] == '%' && len == 1) {
            param.btext[0] = '\0';
            return true;
        }

        for (i = 0; i < len && i < BTEXT_LEN; i++) {
            param.btext[i] = buf[i];
        }
        param.btext[i] = '\0';

    } else {

        tty_write_str(ttyp, "BTEXT ");
        tty_write_str(ttyp, param.btext);
        tty_write_str(ttyp, "\r\n");
    }

    return true;
}

static bool cmd_beacon(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {

        static uint8_t const every[] = "EVERY";
        uint8_t const *s = every;
        int i = 0;

        if (!strncasecmp(buf, "OFF", 3)) {
            param.beacon = 0;
            return true;
        }

        if (!strncasecmp(buf, "NOW", 3)) {
            beacon_now();
            beacon_reset();
            return true;
        }

        while (toupper(buf[i]) == *s) {
            i++;
            s++;
        }

        if (!buf[i] || buf[i] != ' ') return false;

        int r, t;
        r = sscanf(&buf[i], "%d", &t);

        if (r != 1 || (t < 0 || t > 60)) return false;

        param.beacon = t;
        beacon_reset();     // beacon timer reset

    } else {

        tty_write_str(ttyp, "BEACON ");

        if (param.beacon > 0) {
            uint8_t temp[4];

            tty_write_str(ttyp, "On EVERY ");
            tty_write(ttyp, temp, sprintf(temp, "%u", param.beacon));
        } else {
            tty_write_str(ttyp, "Off");
        }
        tty_write_str(ttyp, "\r\n");
    }

    return true;
}

static bool cmd_monitor(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {

        if (!strncasecmp(buf, "ALL", 3) || !strncasecmp(buf, "ON", 2)) {
            param.mon = MON_ALL;
        } else if (!strncasecmp(buf, "ME", 2)) {
            param.mon = MON_ME;
        } else if (!strncasecmp(buf, "OFF", 3)) {
            param.mon = MON_OFF;
        } else if (buf[0] == '0') {
            param.mon = MON_OFF;  // TNC2-style "MON 0" = monitor off
        } else if (buf[0] >= '1' && buf[0] <= '9') {
            param.mon = MON_ALL;  // TNC2-style "MON 1" (or any non-zero) = monitor all
        } else {
            return false;
        }

    } else {

        tty_write_str(ttyp, "MONitor ");
        if (param.mon == MON_ALL) {
            tty_write_str(ttyp, "ALL");
        } else if (param.mon == MON_ME) {
            tty_write_str(ttyp, "ME");
        } else {
            tty_write_str(ttyp, "OFF");
        }
        tty_write_str(ttyp, "\r\n");

    }

    return true;
}

static bool cmd_digipeat(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {

        if (!strncasecmp(buf, "ON", 2)) {
            param.digi = true;
        } else if (!strncasecmp(buf, "OFF", 3)) {
            param.digi = false;
        } else {
            return false;
        }

    } else {

        tty_write_str(ttyp, "DIGIpeater ");
        if (param.digi) {
            tty_write_str(ttyp, "ON");
        } else {
            tty_write_str(ttyp, "OFF");
        }
        tty_write_str(ttyp, "\r\n");
    }

    return true;
}

static bool cmd_myalias(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {
        if (read_call(buf, &param.myalias) == NULL) return false;
        digipeat_config_changed();
        return true;

        //usb_write(buf, len);
        //usb_write("\r\n", 2);

    } else {
        uint8_t call[10];

        tty_write_str(ttyp, "MYALIAS ");
        if (param.myalias.call[0]) tty_write(ttyp, call, callsign2ascii(call, &param.myalias));
        tty_write_str(ttyp, "\r\n");

    }

    return true;
}

#if !PICO_TNC_PARENT_INTEGRATION
static bool cmd_digipath(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {
        if (len > DIGI_PATH_LEN || !digipeat_path_valid((char *)buf)) return false;
        memcpy(param.digi_path, buf, len);
        param.digi_path[len] = '\0';
        digipeat_config_changed();
    } else {
        tty_write_str(ttyp, "DIGIPATH ");
        tty_write_str(ttyp, param.digi_path);
        tty_write_str(ttyp, "\r\n");
    }
    return true;
}

static bool cmd_digihold(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {
        char *end;
        long value = strtol((char *)buf, &end, 10);
        while (*end == ' ') end++;
        if (*end || value < 0 || value > 5000) return false;
        param.digi_holdoff_ms = (uint16_t)value;
        digipeat_config_changed();
    } else {
        uint8_t value[8];
        int size = snprintf((char *)value, sizeof(value), "%u", param.digi_holdoff_ms);
        tty_write_str(ttyp, "DIGIHOLD ");
        tty_write(ttyp, value, size);
        tty_write_str(ttyp, "\r\n");
    }
    return true;
}
#endif

static bool cmd_perm(tty_t *ttyp, uint8_t *buf, int len)
{
    //tty_write("PERM\r\n", 6);

    receive_off(); // stop ADC free running

    bool ret = flash_write(&param, sizeof(param));

    receive_on();

    return ret;
}

static bool cmd_echo(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {

        if (!strncasecmp(buf, "ON", 2)) {
            param.echo = 1;
        } else if (!strncasecmp(buf, "OFF", 3)) {
            param.echo = 0;
        } else {
            return false;
        }

     } else {

        tty_write_str(ttyp, "ECHO ");
        if (param.echo) {
            tty_write_str(ttyp, "ON"); 
        } else {
            tty_write_str(ttyp, "OFF");
        }
        tty_write_str(ttyp, "\r\n");
    }

    return true;
}

static bool cmd_gps(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {

        for (int i = 0; i < 3; i++) {
            uint8_t const *str = gps_str[i];
        
            if (!strncasecmp(buf, str, strlen(str))) {
                param.gps = i;
                return true;
            }
        }
        return false;

    } else {

        tty_write_str(ttyp, "GPS ");
        tty_write_str(ttyp, gps_str[param.gps]);
        tty_write_str(ttyp, "\r\n");
    }

    return true;
}

static bool cmd_trace(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {

        if (!strncasecmp(buf, "OFF", 3)) {
            param.trace = TR_OFF;
        } else if (!strncasecmp(buf, "XMIT", 4)) {
            param.trace = TR_XMIT;
        } else if (!strncasecmp(buf, "RCV", 3)) {
            param.trace = TR_RCV;
        } else {
            return false;
        }
    
    } else {

        tty_write_str(ttyp, "TRace ");
        if (param.trace == TR_XMIT) {
            tty_write_str(ttyp, "XMIT");
        } else if (param.trace == TR_RCV) {
            tty_write_str(ttyp, "RCV");
        } else {
            tty_write_str(ttyp, "OFF");
        }
        tty_write_str(ttyp, "\r\n");
    }
    
    return true;
}

static bool cmd_txdelay(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {
        
        int t = atoi(buf);

        if (t <= 0 || t > 200) return false;

        param.txdelay = t;

        // set txdelay
        tnc[0].kiss_txdelay = param.txdelay * 2 / 3;

    } else {
        uint8_t temp[8];

        tty_write_str(ttyp, "TXDELAY ");
        tty_write(ttyp, temp, snprintf(temp, 8, "%u", param.txdelay));
        tty_write_str(ttyp, "\r\n");
    }

    return true;
}

static bool cmd_calibrate(tty_t *ttyp, uint8_t *buf, int len)
{
    //tty_write_str(ttyp, "CALIBRATE\r\n");
    tnc_t *tp = &tnc[0];
    if (tp->send_state != SP_IDLE) {
        tty_write_str(ttyp, "Transmitter busy\r\n");
        return false;
    }

    tp->send_state = SP_CALIBRATE;
    tp->do_nrzi = false;
    calibrate_mode = true;
    calibrate_idx = 0;
    tp->cal_data = 0x00;
    tp->ttyp = ttyp;
    tp->cal_time = tnc_time();
    tty_write_str(ttyp, "Calibrate Mode. SP to toggle; ctl C to Exit\r\n");
    return true;
}

void calibrate(void)
{
    tnc_t *tp = &tnc[0];
    if (tp->send_state != SP_CALIBRATE_OFF) return;

    tp->send_state = SP_IDLE;
    tp->do_nrzi = true;
    calibrate_mode = false;
    tty_write_str(tp->ttyp, "Exit Calibrate Mode\r\ncmd: ");
}

static bool cmd_converse(tty_t *ttyp, uint8_t *buf, int len)
{
    //tty_write("CONVERSE\r\n", 10);
    converse_mode = true;
    tty_write_str(ttyp, "***  Converse Mode, ctl C to Exit\r\n");
    return true;
}

static bool cmd_kiss(tty_t *ttyp, uint8_t *buf, int len)
{
    if (buf && buf[0]) {

        if (!strncasecmp(buf, "ON", 2)) {
            ttyp->kiss_mode = 1;
        } else if (!strncasecmp(buf, "OFF", 3)) {
            ttyp->kiss_mode = 0;
        } else {
            return false;
        }

     } else {

        tty_write_str(ttyp, "KISS ");
        if (ttyp->kiss_mode) {
            tty_write_str(ttyp, "ON"); 
        } else {
            tty_write_str(ttyp, "OFF");
        }
        tty_write_str(ttyp, "\r\n");
    }

    return true;
}

static bool cmd_help(tty_t *ttyp, uint8_t *buf, int len)
{
    //printf("tud_cdc_write_available() = %d\n", tud_cdc_write_available());

    tty_write_str(ttyp, help_str);

    //printf("tud_cdc_write_available() = %d\n", tud_cdc_write_available());

    return true;
}

static bool cmd_disp(tty_t *ttyp, uint8_t *buf, int len)
{
    uint8_t temp[10]; // 6 + '-' + 2 + '\0'

    tty_write_str(ttyp, "\r\n");

    // echo
    cmd_echo(ttyp, NULL, 0);

    // txdelay
    cmd_txdelay(ttyp, NULL, 0);

    // gps
    cmd_gps(ttyp, NULL, 0);

    // trace
    cmd_trace(ttyp, NULL, 0);

    // monitor
    cmd_monitor(ttyp, NULL, 0);

    // digipeat
    cmd_digipeat(ttyp, NULL, 0);

#if !PICO_TNC_PARENT_INTEGRATION
    cmd_digipath(ttyp, NULL, 0);
    cmd_digihold(ttyp, NULL, 0);
#endif

    // beacon
    cmd_beacon(ttyp, NULL, 0);

    // unproto
    cmd_unproto(ttyp, NULL, 0);

#if !PICO_TNC_PARENT_INTEGRATION
    cmd_tocall(ttyp, NULL, 0);
    cmd_bpath(ttyp, NULL, 0);
    cmd_bposition(ttyp, NULL, 0);
    cmd_bsymbol(ttyp, NULL, 0);
    cmd_bpreview(ttyp, NULL, 0);
#endif

    // mycall
    cmd_mycall(ttyp, NULL, 0);

    // myalias
    cmd_myalias(ttyp, NULL, 0);

    // btext
    cmd_btext(ttyp, NULL, 0);

    //usb_write("\r\n", 2);
    
    return true;
}

static const cmd_t cmd_list[] = {
    { "HELP", 4, cmd_help, },
    { "?", 1, cmd_help, },
    { "DISP", 4, cmd_disp, },
    { "MYCALL", 6, cmd_mycall, },
    { "UNPROTO", 7, cmd_unproto, },
    { "BTEXT", 6, cmd_btext, },
    { "BEACON", 7, cmd_beacon, },
    { "MONITOR", 8, cmd_monitor, },
    { "DIGIPEAT", 9, cmd_digipeat, },
    { "MYALIAS", 8, cmd_myalias, },
#if !PICO_TNC_PARENT_INTEGRATION
    { "DIGIPATH", 8, cmd_digipath, },
    { "DIGIHOLD", 8, cmd_digihold, },
    { "TOCALL", 6, cmd_tocall, },
    { "BPATH", 5, cmd_bpath, },
    { "BPOSITION", 9, cmd_bposition, },
    { "BSYMBOL", 7, cmd_bsymbol, },
    { "BPREVIEW", 8, cmd_bpreview, },
#endif
    { "PERM", 4, cmd_perm, },
    { "ECHO", 4, cmd_echo, },
    { "GPS", 3, cmd_gps, },
    { "TRACE", 5, cmd_trace, },
    { "TXDELAY", 7, cmd_txdelay, },
    { "CALIBRATE", 9, cmd_calibrate, },
    { "CONVERSE", 8, cmd_converse, },
    { "K", 1, cmd_converse, },
    { "KISS", 4, cmd_kiss, },

    // end mark
    { NULL, 0, NULL, },
};


void cmd(tty_t *ttyp, uint8_t *buf, int len)
{
#if 0
    tud_cdc_write(buf, len);
    tud_cdc_write("\r\n", 2);
    tud_cdc_write_flush();
#endif

    uint8_t *top;
    int i;

    for (i = 0; i < len; i++) {
        if (buf[i] != ' ') break;
    }
    top = &buf[i];
    int n = len - i;

    if (n <= 0) return;

    uint8_t *param = strchr(top, ' ');
    int param_len = 0;

    if (param) {
        n = param - top;
        param_len = len - (param - buf);

        for (i = 0; i < param_len; i++) {
            if (param[i] != ' ') break;
        }
        param += i;
        param_len -= i;
    }

    cmd_t const *cp = &cmd_list[0], *mp;
    int matched = 0;

    while (cp->name) {

#if 0
        tud_cdc_write(cp->name, cp->len);
        tud_cdc_write("\r\n", 2);
        tud_cdc_write_flush();
#endif
     
        // For single-char commands (like K), require exact match to avoid
        // ambiguity with longer commands (e.g. K vs KISS).
        bool exact = (n == cp->len);
        bool prefix = (cp->len > n) && (n > 1);
        if ((exact || prefix) && !strncasecmp(top, cp->name, n)) {
            ++matched;
            mp = cp;
        }
        cp++;
    }

    if (matched == 1) {

        if (mp->func(ttyp, param, param_len)) {
            if (!(converse_mode | calibrate_mode)) tty_write_str(ttyp, "\r\nOK\r\n");
            return;
        }
    }

    tty_write_str(ttyp, "\r\n?\r\n");
}
