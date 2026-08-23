/**
 * Copyright (c) 2021 JN1DFF
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once
#include "pico/util/queue.h"

#include "filter.h"
#include "ax25.h"
//#include "cmd.h"

// number of ports
#define PORT_N 1    // number of ports, 1..3

#define BAUD_RATE 1200
#define SAMPLING_N 33
//#define DELAY_N 3
//#define SAMPLING_RATE ((1000000*DELAY_N+DELAY_US/2)/DELAY_US)
#define SAMPLING_RATE (BAUD_RATE * SAMPLING_N)
#define DELAY_US 446 // 446us
#define DELAYED_N ((SAMPLING_RATE * DELAY_US + 500000) / 1000000)

#define ADC_SAMPLING_RATE (SAMPLING_RATE * PORT_N)

#define DATA_LEN 1024                   // packet receive buffer size

#define FIR_LPF_N 81
#define FIR_BPF_N 75

#define ADC_BIT 12
//#define ADC_BIT 12      // adc bits 8 or 12

//#define BELL202_SYNC 1  // sync decode
#define DECODE_PLL 1    // use PLL

// Number of parallel bit-slicers per port. Each runs its own PLL + NRZI +
// HDLC state machine over the shared LPF output, biased by a per-slicer
// threshold offset. Frame-level dedupe drops duplicate decodes.
#define NUM_SLICERS 9

// Recent-frame ring for cross-slicer dedupe: when multiple slicers decode the
// same frame, only the first to FCS_OK wins. Ring stores (fcs ^ len) keys
// with a 1.5 s expiry at 10 ms ticks.
#define DEDUP_RING 8
#define DEDUP_TIMEOUT_TICKS 150

// Fix-bits single-bit-invert salvage on FCS-fail frames. Different slicers
// often produce the same garbled bytes for an impossible-to-decode frame —
// the fix-bits attempts ring suppresses redundant inner-loop runs across
// slicers in a 1.5 s window.
#define ENABLE_FIXBITS  1
#define FIXBITS_RING    4

#define CONTROL_N 10
#define DAC_QUEUE_LEN 64
#define DAC_BLOCK_LEN (DAC_QUEUE_LEN + 1)

#define SEND_QUEUE_LEN (1024 * 16)

#define AX25_FLAG 0x7e

#define BUSY_PIN 22

#define BEACON_PORT 0

#define KISS_PACKET_LEN 1024                // kiss packet length
#if PICO_TNC_PARENT_INTEGRATION
#define TTY_N 4                             // number of serial
#else
#define TTY_N 3                             // number of serial
#endif
#define CMD_BUF_LEN 255


enum STATE {
	FLAG,
	DATA
};

typedef struct {
  int low_i;
  int low_q;
  int high_i;
  int high_q;
} values_t;

typedef struct TTY tty_t;

// Per-slicer state: each slicer slices the shared LPF output through its own
// threshold offset, runs its own DireWolf-style PLL clock recovery, NRZI
// decode, HDLC framer (flag detect, bit destuff), and frame buffer. With
// NUM_SLICERS>1 the slicers vote at frame level via dedupe in output_packet.
typedef struct SLICER {
    int16_t  offset;          // per-slicer threshold offset (set at init)
    uint8_t  bit;             // last sliced bit (0/1)
    uint8_t  pval;             // previous bit for PLL edge detect
    uint8_t  edge;             // sample counter for non-PLL clock recovery
    int32_t  pll_counter;      // DireWolf PLL phase accumulator
    uint8_t  nrzi;             // last raw bit for NRZI XOR
    uint8_t  state;            // FLAG / DATA
    uint8_t  flag;             // 8-bit shift register for 0x7e detect
    uint8_t  ui_seen;          // 0x03 control byte decoded past address field — LED gate
    uint8_t  data_byte;
    uint8_t  data_bit_cnt;
    uint16_t data_cnt;
    uint32_t dcd_last_byte_time; // per-slicer DCD watchdog
    uint8_t  data[DATA_LEN];
} slicer_t;

typedef struct DEDUP_ENTRY {
    uint16_t fcs;        // trailing 16-bit FCS
    uint16_t len;        // total frame length including FCS
    uint8_t  prefix[4];  // first 4 frame bytes (start of dest call)
    uint32_t ts;         // tick when seen
} dedup_entry_t;

typedef struct TNC {
    uint8_t port;

    // receive

    // parallel bit slicers — each owns its own PLL + NRZI + HDLC state
    slicer_t slicer[NUM_SLICERS];

    // recent-frame dedupe ring (cross-slicer)
    dedup_entry_t dedup_ring[DEDUP_RING];
    uint8_t       dedup_head;

#if ENABLE_FIXBITS
    // recent fix-bits-attempt ring: suppress redundant inner-loop runs when
    // multiple slicers produce the same FCS-fail bytes for one physical frame
    dedup_entry_t fixbits_ring[FIXBITS_RING];
    uint8_t       fixbits_head;
#endif

    // output_packet
    int pkt_cnt;

    // bell202_decode
    int delayed[DELAYED_N];
    int delay_idx;
    int cdt;
    int cdt_lvl;
    int avg;
    uint8_t cdt_pin;

    // data carrier detect (logical): OR of per-slicer in-frame flags.
    // Trips on the first byte past a perceived preamble — used by CSMA to
    // hold off TX. This is intentionally loose: a noise-induced 0x7e plus
    // one decoded byte is enough.
    int dcd;

    // LED state: a *tighter* version of DCD that drives cdt_pin. Only
    // asserted once a slicer has accumulated LED_MIN_BYTES of in-frame
    // data, so noise-induced false preambles (which the DCD watchdog
    // clears within ~200 ms) don't blink the LED.
    int led_on;

    // bell202_decode2
    int sum_low_i;
    int sum_low_q;
    int sum_high_i;
    int sum_high_q;
    int low_idx;
    int high_idx;
    values_t values[SAMPLING_N];
    int values_idx;
    filter_t lpf;
    filter_t bpf;

    // send

    // kiss parameter
    uint8_t kiss_txdelay;
    uint8_t kiss_p;
    uint8_t kiss_slottime;
    uint8_t kiss_fullduplex;

    // dac queue
    queue_t dac_queue;

    // DAC, PTT pin
    uint8_t ptt_pin;
    uint8_t pwm_pin;
    uint8_t pwm_slice;

    // DMA channels
    uint8_t ctrl_chan;
    uint8_t data_chan;
    uint32_t data_chan_mask;
    uint8_t busy;

    // Bell202 wave generator
    int next;
    int phase;
    int level;
    int cnt_one;

    // wave buffer for DMA
    uint32_t const *dma_blocks[DAC_BLOCK_LEN][CONTROL_N + 1];

    // send data queue
    queue_t send_queue;
    int send_time;
    int send_len;
    int send_state;
    int send_data;
    uint8_t send_flags;  // per-packet flags (read in SP_READ_FLAGS)

    // field for test packet
    int test_state;
    uint8_t const *ptp;
    uint8_t const *packet;
    uint16_t packet_len;
    int wait_time;

    // calibrate
    uint8_t cal_data;
    bool do_nrzi;
    uint32_t cal_time;
    tty_t *ttyp;

} tnc_t;

extern tnc_t tnc[];
extern uint32_t __tnc_time;

void tnc_init(void);

// Bridge for non-pico_tnc modules (e.g. iGate IS->RF gating) to queue an
// AX.25 frame for transmission. `data` is the raw AX.25 frame WITHOUT
// trailing FCS; send_packet computes and appends it. Also records the
// frame in the digipeat dedup table to suppress loopback re-digipeating.
// Returns false on invalid port or queue full.
bool tnc_inject_tx(int port, const uint8_t *data, int len);

inline uint32_t tnc_time(void)
{
    return __tnc_time;
}

// TNC command
enum MONITOR {
    MON_ALL = 0,
    MON_ME,
    MON_OFF,
};

// GPS
enum GPS_SENTENCE {
    GPGGA = 0,
    GPGLL,
    GPRMC,
};

#define UNPROTO_N 4
#define BTEXT_LEN 100


// TNC parameter
typedef struct TNC_PARAM {
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
} param_t;

extern param_t param;

// tty

enum TTY_MODE {
    TTY_TERMINAL = 0,
    TTY_GPS,
};

enum TTY_SERIAL {
    TTY_USB = 0,   // TNC2 port 0 on USB CDC 1
#if PICO_TNC_PARENT_INTEGRATION
    TTY_USB2,      // TNC2 port 1 on USB CDC 2
#endif
    TTY_UART0,
    TTY_UART1,
};

typedef struct TTY {
    uint8_t kiss_buf[KISS_PACKET_LEN];
    uint8_t cmd_buf[CMD_BUF_LEN + 1];
    int kiss_idx;
    int cmd_idx;

    uint8_t num;        // index of tty[]

    uint8_t tty_mode;   // terminal or GPS
    uint8_t tty_serial; // USB, UART0, UART1

    uint8_t kiss_mode;  // kiss mode
    uint8_t kiss_state; // kiss state
    uint32_t kiss_timeout; // kiss timer

    tnc_t *tp;          // input/output port No.
} tty_t;

extern tty_t tty[];

// send process state
enum SEND_STATE {
    SP_IDLE = 0,
    SP_READ_FLAGS,
    SP_WAIT_CLR_CH,
    SP_P_PERSISTENCE,
    SP_WAIT_SLOTTIME,
    SP_PTT_ON,
    SP_SEND_FLAGS,            // preamble (pre-data HDLC flags)
    SP_SEND_TRAILING_FLAG,    // post-data flag; may chain to next packet
    SP_DATA_START,
    SP_DATA,
    SP_ERROR,
    SP_CALIBRATE,
    SP_CALIBRATE_OFF,
};
