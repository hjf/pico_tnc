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
#include "pico/stdlib.h"

#include "tnc.h"
#include "beacon.h"
#include "unproto.h"

static uint32_t beacon_time = 0;

static void format_callsign(char *out, int size, const callsign_t *callsign)
{
  int call_len = 0;
  while (call_len < (int)sizeof(callsign->call)
         && callsign->call[call_len] && callsign->call[call_len] != ' ') {
    call_len++;
  }

  if (!call_len) {
    snprintf(out, size, "NOCALL");
  } else if (callsign->ssid) {
    snprintf(out, size, "%.*s-%u", call_len, callsign->call, callsign->ssid);
  } else {
    snprintf(out, size, "%.*s", call_len, callsign->call);
  }
}

int beacon_format(uint8_t *out, int size)
{
  if (!param.beacon_position_set) {
    return snprintf((char *)out, size, "%s", param.btext);
  }

  int64_t lat = param.beacon_lat_e7;
  int64_t lon = param.beacon_lon_e7;
  char north_south = lat < 0 ? 'S' : 'N';
  char east_west = lon < 0 ? 'W' : 'E';
  if (lat < 0) lat = -lat;
  if (lon < 0) lon = -lon;

  int lat_degrees = lat / 10000000;
  int lon_degrees = lon / 10000000;
  int lat_hundredths = ((lat % 10000000) * 6000 + 5000000) / 10000000;
  int lon_hundredths = ((lon % 10000000) * 6000 + 5000000) / 10000000;
  if (lat_hundredths == 6000) {
    lat_degrees++;
    lat_hundredths = 0;
  }
  if (lon_hundredths == 6000) {
    lon_degrees++;
    lon_hundredths = 0;
  }

  return snprintf((char *)out, size, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c%s",
          lat_degrees, lat_hundredths / 100, lat_hundredths % 100,
          north_south, param.beacon_symbol_table,
          lon_degrees, lon_hundredths / 100, lon_hundredths % 100,
          east_west, param.beacon_symbol_code, param.btext);
}

int beacon_format_preview(uint8_t *out, int size)
{
  char preview[BEACON_PREVIEW_LEN];
  uint8_t information[BTEXT_LEN + 32];
  char source[10];
  char destination[10];
  int information_len = beacon_format(information, sizeof(information));
  if (information_len < 0 || information_len >= (int)sizeof(information)) return -1;

  format_callsign(source, sizeof(source), &param.mycall);
  format_callsign(destination, sizeof(destination), &param.unproto[0]);
  int len = snprintf(preview, sizeof(preview), "%s>%s", source, destination);

  for (int i = 1; i < UNPROTO_N && param.unproto[i].call[0]; i++) {
    char repeater[10];
    format_callsign(repeater, sizeof(repeater), &param.unproto[i]);
    len += snprintf(preview + len, sizeof(preview) - len, ",%s", repeater);
  }

  len += snprintf(preview + len, sizeof(preview) - len, ":%s", information);
  if (len < 0 || len >= (int)sizeof(preview)) return -1;
  return snprintf((char *)out, size, "%s", preview);
}

void beacon_now(void)
{
  uint8_t packet[BTEXT_LEN + 32];
  int len = beacon_format(packet, sizeof(packet));
  if (len > 0 && len < (int)sizeof(packet)) {
    send_unproto(&tnc[BEACON_PORT], packet, len);
  }
}

void beacon_reset(void)
{
    beacon_time = tnc_time();
}

void beacon(void)
{
    if (!param.beacon) return;

    if (tnc_time() - beacon_time < param.beacon * 60 * 100) return; // convert minutes to 10 ms

    beacon_now();
    beacon_time = tnc_time();
}
