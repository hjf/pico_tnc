/**
 * Copyright (c) 2021 JN1DFF
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * send.c - send packet
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/structs/uart.h"
#include "pico/util/queue.h"

#include "pio_dac.pio.h"

#include "tnc.h"
#include "send.h"
#include "ax25.h"

//#include "wave_table.h"
#include "wave_table.h"

static const int pwm_pins[] = {
    10, // port 0
    8,  // port 1
    6,  // port 2
};

static const int ptt_pins[] = {
    11, // port 0
    9,  // port 1
    7,  // port 2
};

#define LED_PIN PICO_DEFAULT_LED_PIN

#define ISR_PIN 15

#define CAL_TIMEOUT (60 * 100)  // 60 sec

static void __isr dma_handler(void)
{
    uint32_t int_status = dma_hw->ints0;
    uint32_t own_mask = 0;

    for (int i = 0; i < PORT_N; i++) {
        tnc_t *tp = &tnc[i];
        own_mask |= tp->data_chan_mask;

        if (int_status & tp->data_chan_mask) {

            // generate modem signal
            uint32_t *addr;
            if (queue_try_remove(&tp->dac_queue, &addr)) {
            
                dma_channel_set_read_addr(tp->ctrl_chan, addr, true);

            } else {
                gpio_put(tp->ptt_pin, 0); // PTT off
                tp->busy = false;
            }
        }
    } // for

    // Only clear interrupt bits that belong to this handler.
    // Do NOT clear bits owned by the CYW43 WiFi driver which also
    // shares DMA_IRQ_0 on Pico W.
    dma_hw->ints0 = int_status & own_mask;
}

static void send_start(tnc_t *tp)
{
    if (!tp->busy) {
        gpio_put(tp->ptt_pin, 1); // PTT on
        tp->busy = true;
        //printf("restart dma, ctrl = %08x, port = %d\n", dma_hw->ch[tp->data_chan].ctrl_trig, tp->port);
        dma_channel_set_read_addr(tp->data_chan, NULL, true); // trigger NULL interrupt
    }
}

static bool send_packet_flags(tnc_t *tp, uint8_t *data, int len, uint8_t flags)
{
    int length = len + 2; // fcs 2 byte
    uint8_t byte;

    // Queue layout per packet: [flags][len_lo][len_hi][data...][fcs_lo][fcs_hi]
    if (send_queue_free(tp) < length + 3) return false; // queue has no room

    byte = flags;
    queue_try_add(&tp->send_queue, &byte);

    // packet length
    byte = length;
    queue_try_add(&tp->send_queue, &byte);
    byte = length >> 8;
    queue_try_add(&tp->send_queue, &byte);

    // send packet to queue
    for (int i = 0; i < len; i++) {
        queue_try_add(&tp->send_queue, &data[i]);
    }

    int fcs = ax25_fcs(0, data, len);

    // fcs
    byte = fcs;
    queue_try_add(&tp->send_queue, &byte);
    byte = fcs >> 8;
    queue_try_add(&tp->send_queue, &byte);

    return true;
}

bool send_packet(tnc_t *tp, uint8_t *data, int len)
{
    return send_packet_flags(tp, data, len, 0);
}

bool send_packet_now(tnc_t *tp, uint8_t *data, int len)
{
    return send_packet_flags(tp, data, len, SP_FLAG_SKIP_CSMA);
}

int send_byte(tnc_t *tp, uint8_t data, bool bit_stuff)
{
    int idx = 0;

    // generate modem signal
    if (!queue_is_full(&tp->dac_queue)) {
        //uint8_t data = rand();
            
        int byte = data | 0x100; // sentinel

        int bit = byte & 1;
        while (byte > 1) { // check sentinel

            if (tp->do_nrzi) {
                if (!bit) tp->level ^= 1; // NRZI, invert if original bit == 0
            } else {
                tp->level = bit;
            }

            // make Bell202 CPAFSK audio samples
            tp->dma_blocks[tp->next][idx++] = phase_tab[tp->level][tp->phase]; // 1: mark, 0: space

            if (!tp->level) { // need adjust phase if space (2200Hz)
                if (--tp->phase < 0) tp->phase = PHASE_CYCLE - 1;
            }

            // bit stuffing
            if (bit_stuff) {

                if (bit) {

                    if (++tp->cnt_one >= BIT_STUFF_BITS) {
                        // insert "0" bit
                        bit = 0;
                        continue; // while
                    }

                } else {

                    tp->cnt_one = 0;
                }

            }

            byte >>= 1;
            bit = byte & 1;
        }
        
        // insert DMA end mark
        tp->dma_blocks[tp->next][idx] = NULL;

        // send fsk data to dac queue
        uint32_t const **block = &tp->dma_blocks[tp->next][0];
        if (queue_try_add(&tp->dac_queue, &block)) {
#if 0
            if (!tp->busy) {
                tp->busy = true;
                dma_channel_set_read_addr(tp->data_chan, NULL, true);
                //printf("restart dma, ctrl = %08x, port = %d\n", dma_hw->ch[tp->data_chan].ctrl_trig, tp->port);
            }
#endif
            if (++tp->next >= DAC_BLOCK_LEN) tp->next = 0;
                
        
            //printf("main: queue add success\n");
        } else {
            printf("main: queue add fail\n");
        }

    } else {

        send_start(tp);

    }

    return idx;
}


void send_init(void)
{
    // set system clock, PWM uses system clock
    //set_sys_clock_khz(SYS_CLK_KHZ, true);

    // pio_dac initialize
    PIO pio = pio0;
    // load pio_dac program
    uint offset = pio_add_program(pio, &pio_dac_program);

    // initialize tnc[]
    for (int i = 0; i < PORT_N; i++) {
        tnc_t *tp = &tnc[i];

        // port No.
        tp->port = i;

        // queue
        queue_init(&tp->dac_queue, sizeof(uint32_t *), DAC_QUEUE_LEN);

        // PTT pins
        tp->ptt_pin = ptt_pins[i];
        gpio_init(tp->ptt_pin);
        gpio_set_dir(tp->ptt_pin, true); // output
        gpio_put(tp->ptt_pin, 0);

#if 0
        // PWM pins
        tp->pwm_pin = pwm_pins[i];
        // PWM configuration
        gpio_set_function(tp->pwm_pin, GPIO_FUNC_PWM);
        // PWM slice
        tp->pwm_slice = pwm_gpio_to_slice_num(tp->pwm_pin);

        // PWM configuration
        pwm_config pc = pwm_get_default_config();
        pwm_config_set_clkdiv_int(&pc, 1); // 1.0
        pwm_config_set_wrap(&pc, PWM_CYCLE - 1);
        pwm_init(tp->pwm_slice, &pc, true);   // start PWM
#endif

        // PIO
        // find free state machine
        uint sm = pio_claim_unused_sm(pio, true);
        pio_dac_program_init(pio, sm, offset, pwm_pins[i], PIO_DAC_FS);

        // DMA
        tp->ctrl_chan = dma_claim_unused_channel(true);
        tp->data_chan = dma_claim_unused_channel(true);
        tp->data_chan_mask = 1 << tp->data_chan;

        //printf("port %d: pwm_pin = %d, ptt_pin = %d, ctrl_chan = %d, data_chan = %d\n", tp->port, tp->pwm_pin, tp->ptt_pin, tp->ctrl_chan, tp->data_chan);
        
        // DMA control channel
        dma_channel_config dc = dma_channel_get_default_config(tp->ctrl_chan);
        channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
        channel_config_set_read_increment(&dc, true);
        channel_config_set_write_increment(&dc, false);

        dma_channel_configure(
            tp->ctrl_chan,
            &dc,
            &dma_hw->ch[tp->data_chan].al3_read_addr_trig, // Initial write address
            NULL,                        // Initial read address
            1,                                         // Halt after each control block
            false                                      // Don't start yet
        );

        // DMA data channel
        dc = dma_channel_get_default_config(tp->data_chan);
        channel_config_set_transfer_data_size(&dc, DMA_SIZE_32); // pio_dac data size, 32bit
        channel_config_set_dreq(&dc, pio_get_dreq(pio, sm, true));  // pio sm, TX
        channel_config_set_chain_to(&dc, tp->ctrl_chan);
        channel_config_set_irq_quiet(&dc, true);
        // set high priority bit
        //dc.ctrl |= DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS;

        dma_channel_configure(
            tp->data_chan,
            &dc,
            &pio0_hw->txf[sm],  // The initial write address
            NULL,           // Initial read address and transfer count are unimportant;
            BIT_CYCLE,      // audio data of 1/1200 s
            false           // Don't start yet.
        );

        // configure IRQ
        dma_channel_set_irq0_enabled(tp->data_chan, true);
    }

    // configure IRQ as shared: CYW43 also uses DMA_IRQ_0 on Pico W
    irq_add_shared_handler(DMA_IRQ_0, dma_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);

    // ISR time measurement
    gpio_init(ISR_PIN);
    gpio_set_dir(ISR_PIN, true);

#if !PICO_CYW43_SUPPORTED
    // GPIO23 is CYW43 WL_REG_ON on Pico W — do not touch it
#define SMPS_PIN 23
    // SMPS set PWM mode
    gpio_init(SMPS_PIN);
    gpio_set_dir(SMPS_PIN, true);
    gpio_put(SMPS_PIN, 1);
#endif
}


int send_queue_free(tnc_t *tp)
{
    return SEND_QUEUE_LEN - queue_get_level(&tp->send_queue);
}

void send(void)
{
    uint8_t data;

    tnc_t *tp = &tnc[0];
    while (tp < &tnc[PORT_N]) {

        switch (tp->send_state) {
            case SP_IDLE:
                //printf("(%d) send: SP_IDEL\n", tnc_time());
                if (!queue_is_empty(&tp->send_queue)) {
                    tp->send_state = SP_READ_FLAGS;
                    continue;
                }
                break;

            case SP_READ_FLAGS:
                // First byte of each queued packet is per-packet flags.
                if (!queue_try_remove(&tp->send_queue, &data)) {
                    tp->send_state = SP_IDLE;
                    break;
                }
                tp->send_flags = data;
                tp->send_state = SP_WAIT_CLR_CH;
                continue;

            case SP_WAIT_CLR_CH:
                //printf("(%d) send: SP_WAIT_CLR_CH\n", tnc_time());
                if (tp->kiss_fullduplex || !tp->dcd) {
                    // YOLO digi: skip p-persistence so all colocated digis
                    // commit to keying the moment DCD clears — FM capture
                    // then decides cleanly. (APRS Protocol Reference §3 /
                    // WB2OSZ §3.2.1.)
                    tp->send_state = (tp->send_flags & SP_FLAG_SKIP_CSMA)
                                         ? SP_PTT_ON
                                         : SP_P_PERSISTENCE;
                    continue;
                }
                break;

            case SP_P_PERSISTENCE:
                data = rand();
                //printf("(%d) send: SP_P_PERSISTENCE, rnd = %d\n", tnc_time(), data);
                if (data <= tp->kiss_p) {
                    tp->send_state = SP_PTT_ON;
                    continue;
                }
                tp->send_time = tnc_time();
                tp->send_state = SP_WAIT_SLOTTIME;
                break;

            case SP_WAIT_SLOTTIME:
                //printf("(%d) send: SP_WAIT_SLOTTIME\n", tnc_time());
                if (tnc_time() - tp->send_time >= tp->kiss_slottime) {
                    tp->send_state = SP_WAIT_CLR_CH;
                    continue;
                }
                break;

            case SP_PTT_ON:
                //printf("(%d) send: SP_PTT_ON\n", tnc_time());
                //gpio_put(tp->ptt_pin, 1);
                tp->send_len = (tp->kiss_txdelay * 3) / 2 + 1; // TXDELAY * 10 [ms] into number of flags
                tp->send_state = SP_SEND_FLAGS;
                /* FALLTHROUGH */

            case SP_SEND_FLAGS:
                // Pre-data preamble path: emit kiss_txdelay * 10 ms of HDLC
                // flag bytes after PTT-on, then drop into SP_DATA_START.
                // The packet's flag byte was already consumed in SP_READ_FLAGS.
                //printf("(%d) send: SP_SEND_FLAGS\n", tnc_time());
                while (tp->send_len > 0 && send_byte(tp, AX25_FLAG, false)) { // false: bit stuffing off
                    --tp->send_len;
                }
                if (tp->send_len > 0) break;
                tp->cnt_one = 0;                // bit stuffing counter clear
                tp->send_state = SP_DATA_START;
                /* FALLTHROUGH */

            case SP_DATA_START:
                if (!queue_try_remove(&tp->send_queue, &data)) {
                    tp->send_state = SP_IDLE;
                    break;
                }
                // read packet length low byte
                tp->send_len = data;
                if (!queue_try_remove(&tp->send_queue, &data)) {
                    printf("send: send_queue underrun, len\n");
                    tp->send_state = SP_IDLE;
                    break;
                }
                // read packet length high byte
                tp->send_len += data << 8;
                //printf("(%d) send: SP_DATA_START, len = %d\n", tnc_time(), tp->send_len);
                if (!queue_try_remove(&tp->send_queue, &data)) {
                    printf("send: send_queue underrun, data(1)\n");
                    tp->send_state = SP_IDLE;
                    break;
                }
                tp->send_data = data;
                --tp->send_len;
                tp->send_state = SP_DATA;
                /* FALLTHROUGH */

            case SP_DATA:
                //printf("(%d) send: SP_DATA\n", tnc_time());
                if (!send_byte(tp, tp->send_data, true)) break;
                if (tp->send_len <= 0) {
                    tp->send_len = 1;
                    tp->send_state = SP_SEND_TRAILING_FLAG;
                    send_start(tp);
                    continue;
                }
                if (!queue_try_remove(&tp->send_queue, &data)) {
                    printf("send: send_queue underrun, data(2)\n");
                    tp->send_state = SP_IDLE;
                    break;
                }
                --tp->send_len;
                tp->send_data = data;
                continue;

            case SP_SEND_TRAILING_FLAG:
                // Post-data path: emit one inter-packet HDLC flag.  If
                // another packet is queued, chain into it (reuse the
                // already-on PTT) by reading its per-packet flag byte
                // before SP_DATA_START reads the length.
                while (tp->send_len > 0 && send_byte(tp, AX25_FLAG, false)) {
                    --tp->send_len;
                }
                if (tp->send_len > 0) break;
                tp->cnt_one = 0;
                if (!queue_is_empty(&tp->send_queue)) {
                    uint8_t flag_byte;
                    if (queue_try_remove(&tp->send_queue, &flag_byte)) {
                        tp->send_flags = flag_byte;
                        tp->send_state = SP_DATA_START;
                        continue;
                    }
                }
                tp->send_state = SP_IDLE;
                break;

            case SP_ERROR:
                //printf("(%d) send: SP_ERROR\n", tnc_time());
                while (queue_try_remove(&tp->send_queue, &data)) {
                }
                tp->send_state = SP_IDLE;
                break;

            case SP_CALIBRATE:
                if (tnc_time() - tp->cal_time >= CAL_TIMEOUT) {
                    tp->send_state = SP_CALIBRATE_OFF;
                } else {
                    send_byte(tp, tp->cal_data, false);
                }
                break;
        }

        tp++;
    } // while
}
