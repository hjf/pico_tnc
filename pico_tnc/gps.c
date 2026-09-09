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
#include <string.h>
#include "pico/stdlib.h"

#include "tnc.h"
#if PICO_TNC_PARENT_INTEGRATION
#include "usb_multi.h"
#endif

#define GPS_LEN 127

static uint8_t gps_buf[GPS_LEN + 1];
static int gps_idx = 0;
static bool position_valid = false;
static bool motion_valid = false;
static uint16_t course_degrees = 0;
static uint16_t speed_knots = 0;

#define DOLLAR '$'
#define LF '\n'

static int hex_value(char ch)
{
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  return -1;
}

static bool checksum_valid(char *sentence)
{
  if (sentence[0] != '$') return false;

  char *asterisk = strchr(sentence, '*');
  if (!asterisk || hex_value(asterisk[1]) < 0 || hex_value(asterisk[2]) < 0) return false;

  uint8_t checksum = 0;
  for (char *p = sentence + 1; p < asterisk; p++) checksum ^= (uint8_t)*p;
  if (checksum != (uint8_t)((hex_value(asterisk[1]) << 4) | hex_value(asterisk[2]))) return false;

  *asterisk = '\0';
  return true;
}

static bool parse_coordinate(const char *value, char hemisphere, bool longitude,
               int32_t *coordinate)
{
  const char *dot = strchr(value, '.');
  int degree_digits = longitude ? 3 : 2;
  if (!dot || dot - value != degree_digits + 2) return false;

  int degrees = 0;
  for (int i = 0; i < degree_digits; i++) {
    if (value[i] < '0' || value[i] > '9') return false;
    degrees = degrees * 10 + value[i] - '0';
  }

  if (value[degree_digits] < '0' || value[degree_digits] > '5'
      || value[degree_digits + 1] < '0' || value[degree_digits + 1] > '9') {
    return false;
  }

  int64_t minutes_e7 = (value[degree_digits] - '0') * 100000000LL
             + (value[degree_digits + 1] - '0') * 10000000LL;
  int64_t scale = 1000000;
  const char *p = dot + 1;
  if (!*p) return false;
  while (*p && scale > 0) {
    if (*p < '0' || *p > '9') return false;
    minutes_e7 += (*p++ - '0') * scale;
    scale /= 10;
  }
  while (*p) {
    if (*p++ < '0' || p[-1] > '9') return false;
  }

  int64_t result = degrees * 10000000LL + (minutes_e7 + 30) / 60;
  int maximum = longitude ? 180 : 90;
  if (result > maximum * 10000000LL) return false;

  if ((!longitude && hemisphere == 'S') || (longitude && hemisphere == 'W')) {
    result = -result;
  } else if ((!longitude && hemisphere != 'N') || (longitude && hemisphere != 'E')) {
    return false;
  }

  *coordinate = (int32_t)result;
  return true;
}

static bool parse_decimal_tenths(const char *value, uint16_t maximum, uint16_t *result)
{
  if (!value[0]) return false;

  uint32_t whole = 0;
  const char *p = value;
  while (*p >= '0' && *p <= '9') {
    whole = whole * 10 + (*p++ - '0');
    if (whole > maximum) return false;
  }

  uint32_t tenths = 0;
  if (*p == '.') {
    p++;
    if (*p >= '0' && *p <= '9') tenths = *p++ - '0';
    while (*p >= '0' && *p <= '9') p++;
  }
  if (*p || whole > maximum || (whole == maximum && tenths)) return false;

  *result = (uint16_t)(whole * 10 + tenths);
  return true;
}

static void gps_update_position(char *sentence)
{
  if (param.beacon_position_set != BEACON_POSITION_GPS || !checksum_valid(sentence)) return;

  char *fields[16];
  int count = 0;
  for (char *field = sentence; field && count < (int)(sizeof(fields) / sizeof(fields[0]));) {
    fields[count++] = field;
    char *comma = strchr(field, ',');
    if (comma) *comma++ = '\0';
    field = comma;
  }

  const char *latitude = NULL;
  const char *longitude = NULL;
  char north_south = 0;
  char east_west = 0;
  if (strlen(fields[0]) != 6) return;

  const char *type = fields[0] + 3;
  if (!strcmp(type, "GGA")) {
    if (count < 7) return;
    if (fields[6][0] == '0' || !fields[6][0]) {
      position_valid = false;
      motion_valid = false;
      return;
    }
    latitude = fields[2];
    north_south = fields[3][0];
    longitude = fields[4];
    east_west = fields[5][0];
  } else if (!strcmp(type, "RMC")) {
    if (count < 9) return;
    if (fields[2][0] != 'A') {
      position_valid = false;
      motion_valid = false;
      return;
    }
    latitude = fields[3];
    north_south = fields[4][0];
    longitude = fields[5];
    east_west = fields[6][0];

    uint16_t speed_tenths;
    uint16_t course_tenths;
    if (parse_decimal_tenths(fields[7], 999, &speed_tenths)
        && parse_decimal_tenths(fields[8], 360, &course_tenths)) {
      speed_knots = (speed_tenths + 5) / 10;
      course_degrees = (course_tenths + 5) / 10;
      if (course_degrees == 0) course_degrees = 360;
      if (course_degrees > 360) course_degrees = 360;
      motion_valid = true;
    } else {
      motion_valid = false;
    }
  } else if (!strcmp(type, "GLL")) {
    if (count < 7) return;
    if (fields[6][0] != 'A') {
      position_valid = false;
      motion_valid = false;
      return;
    }
    latitude = fields[1];
    north_south = fields[2][0];
    longitude = fields[3];
    east_west = fields[4][0];
  } else {
    return;
  }

  int32_t lat;
  int32_t lon;
  if (!parse_coordinate(latitude, north_south, false, &lat)
      || !parse_coordinate(longitude, east_west, true, &lon)) {
    return;
  }

  param.beacon_lat_e7 = lat;
  param.beacon_lon_e7 = lon;
  position_valid = true;
}

bool gps_position_valid(void)
{
  return position_valid;
}

bool gps_motion_valid(uint16_t *course, uint16_t *speed)
{
  if (!motion_valid) return false;
  *course = course_degrees;
  *speed = speed_knots;
  return true;
}

void gps_position_reset(void)
{
  position_valid = false;
  motion_valid = false;
}

void gps_input(int ch)
{
#if PICO_TNC_PARENT_INTEGRATION
    // Forward every raw NMEA byte to the GPS passthrough CDC port
    usb_multi_write_char(USB_CDC_GPS, (uint8_t)ch);
#endif

    if (ch == DOLLAR) gps_idx = 0;

    if (gps_idx < GPS_LEN) gps_buf[gps_idx++] = ch;

    if (ch == LF) {
    gps_buf[gps_idx] = '\0';
    gps_update_position((char *)gps_buf);
        gps_idx = 0;
    }
}
