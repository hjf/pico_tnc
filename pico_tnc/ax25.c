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

/*
    calculate CCITT-16 CRC using DMA CRC hardware
*/
#include <stdio.h>
#include "pico/stdlib.h"

#include "ax25.h"

//#ifdef RASPBERRYPI_PICO
#ifdef PICO_DEFAULT_UART

#include "hardware/dma.h"

#define OUT_INV (1 << 11)
#define OUT_REV (1 << 10)

int ax25_fcs(uint32_t crc, const uint8_t *data, int size)
{
    uint8_t dummy;
    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_write_increment(&c, false);
    channel_config_set_read_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_FORCE);

    dma_channel_configure(
        dma_chan,
        &c,
        &dummy,
        data,
        size,
        false
    );

    // enable sniffer
    dma_sniffer_enable(dma_chan, 0x3, true);
    dma_hw->sniff_ctrl |= OUT_INV | OUT_REV;
    dma_hw->sniff_data = crc;
    dma_hw->sniff_data >>= 16;

    // start DMA
    dma_channel_start(dma_chan);

    // wait for finish
    dma_channel_wait_for_finish_blocking(dma_chan);
    uint32_t result = dma_hw->sniff_data >> 16;
    dma_sniffer_disable();
    dma_channel_unclaim(dma_chan);

    return result;
}

#else

#warning using C standard routine

#define CRC16_POLY 0x10811 /* G(x) = 1 + x^5 + x^12 + x^16 */

int ax25_fcs(uint32_t crc, const uint8_t packet[], int length)
{
    int i, j;

    if (length <= 0) return -1; // packet too short

    // calculate CRC x^16 + x^12 + x^5 + 1
    crc = 0xffff; /* initial value */
    for (i = 0; i < length; i++) {
	crc ^= packet[i];
	for (j = 0; j < 8; j++) {
	    if (crc & 1) crc ^= CRC16_POLY;
	    crc >>= 1;
	}
    }
    crc ^= 0xffff; // invert

    return crc;
}

#endif

bool ax25_callcmp(callsign_t *c, uint8_t *addr)
{
    for (int i = 0; i < 6; i++) {
        if (c->call[i] != (addr[i] >> 1)) return false;
    }

    if (c->ssid == ((addr[6] >> 1) & 0x0f)) return true;

    return false;
}

void ax25_mkax25addr(uint8_t *addr, callsign_t *c)
{
    uint8_t *s = addr;

    for (int i = 0; i < 6; i++) {
        *s++ = c->call[i] << 1;
    }

    // SSID
    *s = (c->ssid << 1) | 0x60;
}

bool ax25_callsign_valid(const callsign_t *call)
{
    if (!call || call->ssid > 15) return false;
    bool padding = false;
    for (int i = 0; i < 6; ++i) {
        unsigned char c = call->call[i];
        if (c == ' ' && i > 0) padding = true;
        else if (padding || !((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) return false;
    }
    return true;
}

int ax25_control_offset(const uint8_t *packet, int len)
{
    if (!packet || len < 15 || len > AX25_MAX_FRAME_LEN) return -1;
    bool unused = false;
    for (int off = 0; off + 7 < len && off < 70; off += 7) {
        callsign_t c;
        for (int j = 0; j < 6; ++j) {
            if (packet[off+j] & 1) return -1;
            c.call[j] = packet[off+j] >> 1;
        }
        c.ssid = (packet[off+6] >> 1) & 15;
        if (!ax25_callsign_valid(&c)) return -1;
        // Reserved bits are ignored on receive for compatibility with older TNCs.
        if (off >= 14) {
            if (!(packet[off+6] & 0x80)) unused = true;
            else if (unused) return -1; // repeated addresses must form a prefix
        }
        if (packet[off+6] & 1) return off >= 7 ? off+7 : -1;
    }
    return -1;
}

bool ax25_frame_valid(const uint8_t *packet, int len)
{
    int c = ax25_control_offset(packet, len);
    if (c < 0) return false;
    uint8_t ctrl = packet[c] & ~0x10; // P/F bit
    if (!(ctrl & 1) || ctrl == 3) // I or UI: PID and bounded information
        return len >= c+2 && len-c-2 <= AX25_MAX_INFO_LEN;
    if ((ctrl & 3) == 1) // modulo-8 supervisory frames
        return len == c+1 && (ctrl & 0x0f) != 0x0d;
    // Known unnumbered controls; only XID/TEST carry information.
    if (ctrl == 0xaf || ctrl == 0xe3) return len-c-1 <= AX25_MAX_INFO_LEN;
    return len == c+1 && (ctrl == 0x2f || ctrl == 0x6f || ctrl == 0x43 ||
                         ctrl == 0x63 || ctrl == 0x0f);
}

bool ax25_ui(uint8_t *packet, int len)
{
    int c = ax25_control_offset(packet, len);
    return c >= 0 && len >= c+3 && len-c-2 <= AX25_MAX_INFO_LEN &&
           packet[c] == 0x03 && packet[c+1] == 0xf0;
}
