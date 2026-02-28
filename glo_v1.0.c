
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "tusb.h"
#include "i2s.pio.h"

#define PIN_DOUT 8
#define PIN_DIN  9
#define PIN_BCK  10
#define PIN_LRCK 11
#define PIN_MCLK 20
#define PIN_FS1  0
#define PIN_FS2  1
#define PIN_FS3  2
#define PIN_LED1 4
#define PIN_LED2 3
#define PIN_TYPE   26
#define PIN_INTENS 27

#define PIN_UART_TX 5       
#define PIN_UART_RX 6 
#define DISPLAY_UART uart1
#define DISPLAY_BAUD 115200

#define FS    48000
#define BLOCK 128

static inline int32_t clamp24(int32_t v) {
    if (v >  8388607) return  8388607;
    if (v < -8388608) return -8388608;
    return v;
}




static uint32_t buf[3][BLOCK];
static int dma_rx_ch, dma_tx_ch;

static void dma_rx_go(uint32_t *b) {
    dma_channel_set_write_addr(dma_rx_ch, b, false);
    dma_channel_set_trans_count(dma_rx_ch, BLOCK, true);
}
static void dma_tx_go(uint32_t *b) {
    dma_channel_set_read_addr(dma_tx_ch, b, false);
    dma_channel_set_trans_count(dma_tx_ch, BLOCK, true);
}
static void wait_rx(void) { while (dma_channel_is_busy(dma_rx_ch)) tight_loop_contents(); }
static void wait_tx(void) { while (dma_channel_is_busy(dma_tx_ch)) tight_loop_contents(); }


#define FX_BUF_LEN  48000
static int16_t fx_buf[FX_BUF_LEN];


#define MODE_LOFI  0
#define MODE_REV   1
#define MODE_CLIP  2
#define MODE_STU   3
#define MODE_ALL   4
#define MODE_COUNT 5
static int cur_mode = MODE_LOFI;

#define XFADE_LEN  960
static int xfade_cnt = 0;
static int xfade_from = MODE_LOFI;

#define DEFAULT_BPM     120
#define MIN_BPM         40
#define MAX_BPM         300
#define TAP_TIMEOUT_MS  2000
#define DEBOUNCE_MS     50

static uint32_t bpm = DEFAULT_BPM;
static uint32_t beat_period;

static uint32_t tap_last_ms = 0;
static uint32_t tap_intervals[4];
static int      tap_count = 0;

static void update_beat_period(void) {
    beat_period = (FS * 60) / bpm;
    if (beat_period < 1) beat_period = 1;
}


static bool     fs1_last = false;
static uint32_t fs1_debounce = 0;


static bool     fs2_held = false;
static bool     fs3_latched = false; 
static bool     fs3_last_raw = false;
static uint32_t fs3_debounce = 0;

static bool     fx_active = false;
static bool     force_max = false;
static int      intensity = 0;
static int      effective_intensity = 0; 
static uint32_t kt_raw, ki_raw;

static uint32_t led1_phase_samples = 0;
static uint32_t transport_phase_samples = 0;
#define LED1_PULSE_MS 18


static void read_controls(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    adc_select_input(0);
    ki_raw = (ki_raw * 7 + adc_read()) >> 3;

    adc_select_input(1);
    kt_raw = (kt_raw * 7 + adc_read()) >> 3;

    intensity = (int)(kt_raw >> 5);
    if (intensity > 128) intensity = 128;

    int new_mode;
    if      (ki_raw < 819)  new_mode = MODE_LOFI;  
    else if (ki_raw < 1638) new_mode = MODE_REV;  
    else if (ki_raw < 2457) new_mode = MODE_CLIP; 
    else if (ki_raw < 3276) new_mode = MODE_STU;
    else                    new_mode = MODE_ALL;

    if (new_mode != cur_mode) {
        xfade_from = cur_mode;
        xfade_cnt = XFADE_LEN;
        cur_mode = new_mode;
        memset(fx_buf, 0, sizeof(fx_buf));
    }

   
    bool fs1_now = gpio_get(PIN_FS1);
    if (fs1_now && !fs1_last && (now - fs1_debounce > DEBOUNCE_MS)) {
        if (tap_last_ms > 0 && (now - tap_last_ms) < TAP_TIMEOUT_MS) {
            uint32_t interval = now - tap_last_ms;
            if (tap_count < 4) {
                tap_intervals[tap_count++] = interval;
            } else {
                for (int j = 0; j < 3; j++) tap_intervals[j] = tap_intervals[j+1];
                tap_intervals[3] = interval;
            }
            uint32_t sum = 0;
            for (int j = 0; j < tap_count; j++) sum += tap_intervals[j];
            uint32_t avg_ms = sum / (uint32_t)tap_count;
            if (avg_ms > 0) {
                uint32_t new_bpm = 60000 / avg_ms;
                if (new_bpm < MIN_BPM) new_bpm = MIN_BPM;
                if (new_bpm > MAX_BPM) new_bpm = MAX_BPM;
                bpm = new_bpm;
                update_beat_period();
                led1_phase_samples = 0;
                transport_phase_samples = 0;
            }
        } else {
            tap_count = 0;
        }
        tap_last_ms = now;
        fs1_debounce = now;
    }
    fs1_last = fs1_now;

   
    fs2_held = gpio_get(PIN_FS2);

   
    bool fs3_raw = gpio_get(PIN_FS3);
    if (fs3_raw && !fs3_last_raw && (now - fs3_debounce > DEBOUNCE_MS)) {
        fs3_latched = !fs3_latched;
        fs3_debounce = now;
    }
    fs3_last_raw = fs3_raw;

   
    static bool fs3_took_over = false; 

   
    static bool fs3_was_latched = false;
    bool fs3_just_on = (fs3_latched && !fs3_was_latched);
    bool fs3_just_off = (!fs3_latched && fs3_was_latched);
    fs3_was_latched = fs3_latched;

   
    if (fs3_just_on && fs2_held) {
        fs3_took_over = true;
    }

   
    if (!fs2_held) {
        fs3_took_over = false;
    }

   
    fx_active = fs3_latched || fs2_held;

   
    if (fs3_latched && fs2_held && !fs3_took_over) {
       
        force_max = true;
    } else {
        force_max = false;
    }

   
    effective_intensity = force_max ? 128 : intensity;

   
    gpio_put(PIN_LED2, fx_active);

   
    if (fx_active) {
        transport_phase_samples += BLOCK;
        if (transport_phase_samples >= beat_period)
            transport_phase_samples %= beat_period;

        led1_phase_samples += BLOCK;
        if (led1_phase_samples >= beat_period)
            led1_phase_samples %= beat_period;

        uint32_t pulse = (uint32_t)(FS * LED1_PULSE_MS / 1000);
        if (pulse < 60) pulse = 60;
        if (pulse > (beat_period / 2)) pulse = (beat_period / 2);
        gpio_put(PIN_LED1, led1_phase_samples < pulse);
    } else {
        gpio_put(PIN_LED1, false);
        led1_phase_samples = 0;
        transport_phase_samples = 0;
    }
}





#define FW_VERSION "GLO_FX_v15"

static uint8_t usb_rx[16];
static int     usb_rx_idx = 0;

static void check_usb_upgrade(void) {
    if (!tud_cdc_connected() || !tud_cdc_available()) return;

    while (tud_cdc_available() && usb_rx_idx < 8) {
        usb_rx[usb_rx_idx++] = (uint8_t)tud_cdc_read_char();
    }

    if (usb_rx_idx >= 8) {
        if (memcmp(usb_rx, "GLOBOOT!", 8) == 0) {
           
            tud_cdc_write_str("BOOT_OK\n");
            tud_cdc_write_flush();
            sleep_ms(100);
            reset_usb_boot(0, 0);
           
        }
        else if (memcmp(usb_rx, "GLOINFO!", 8) == 0) {
           
            tud_cdc_write_str(FW_VERSION);
            tud_cdc_write_str("\n");
            tud_cdc_write_flush();
        }
        usb_rx_idx = 0;
    }
}






static uint8_t  peak_level = 0;
static uint32_t uart_skip = 0;     

static void send_display_packet(void) {
   
    uart_skip++;
    if (uart_skip < 4) return;
    uart_skip = 0;

    uint8_t pkt[12];
    pkt[0] = 0xAA;
    pkt[1] = 0x55;
    pkt[2] = (uint8_t)(bpm >> 8);
    pkt[3] = (uint8_t)(bpm & 0xFF);
    pkt[4] = (uint8_t)cur_mode;
    pkt[5] = (uint8_t)intensity;
    pkt[6] = (fx_active ? 1 : 0)
           | (force_max ? 2 : 0)
           | (fs2_held ? 4 : 0)
           | (fs3_latched ? 8 : 0);
    pkt[7] = peak_level;

   
    uint32_t bp = (beat_period > 0)
        ? (uint32_t)(((uint64_t)transport_phase_samples * 255) / beat_period)
        : 0;
    if (bp > 255) bp = 255;
    pkt[8] = (uint8_t)bp;
    pkt[9] = (uint8_t)effective_intensity;

   
    uint8_t ck = 0;
    for (int i = 2; i <= 9; i++) ck ^= pkt[i];
    pkt[10] = ck;
    pkt[11] = 0xFF;

   
    for (int i = 0; i < 12; i++) {
        if (uart_is_writable(DISPLAY_UART)) {
            uart_putc_raw(DISPLAY_UART, pkt[i]);
        }
    }
}


static void update_peak(int32_t *s, int n) {
    int32_t mx = 0;
    for (int i = 0; i < n; i++) {
        int32_t v = s[i] < 0 ? -s[i] : s[i];
        if (v > mx) mx = v;
    }
   
    uint8_t pk = (uint8_t)(mx >> 15);
    if (pk > 255) pk = 255;
   
    if (pk > peak_level) {
        peak_level = pk;
    } else {
        if (peak_level > 2) peak_level -= 2;
        else peak_level = 0;
    }
}




#define CHO_LEN       (1<<13)
#define CHO_MASK      (CHO_LEN-1)
#define CHO_CENTER    600
#define CHO_MOD_MAX   42
#define CHO_WET_MAX   45
#define CHO_FB_MAX    28
#define CHO_LFO_LEN   (1<<16)
#define CHO_LFO_HALF  (CHO_LFO_LEN>>1)

static int32_t cho_buf[CHO_LEN];
static int     cho_wp;
static int     cho_lph;
static int32_t cho_fbs;


#define ONSET_FAST_ALPHA   32
#define ONSET_SLOW_ALPHA   2
#define ONSET_RATIO        3
#define ONSET_MIN_LEVEL    50000
#define ONSET_COOLDOWN     24000

static int32_t onset_fast = 0;
static int32_t onset_slow = 0;
static int     onset_cool = 0;


#define TAIL_DS       4
#define TAIL_BUF_LEN  14400         
#define TAIL_MAX_LEN  57600         
#define TAIL_MIN_LEN  4800
#define TAIL_WET_MAX  110

static int16_t tail_buf[TAIL_BUF_LEN];
static int     tail_wp = 0;
static int     tail_ds_cnt = 0;
static int32_t tail_ds_acc = 0;

static bool     tail_active = false;
static uint32_t tail_rpf;
static int32_t  tail_spd;
static int32_t  tail_spd_step;
static int32_t  tail_amp;
static int32_t  tail_amp_step;
static int32_t  tail_lp = 0;       
static int      tail_fade_in = 0;
#define TAIL_FADE_LEN 480

static uint32_t tail_rng = 0xDEADBEEF;
static int tail_random(int max) {
    tail_rng = tail_rng * 1664525u + 1013904223u;
    return (int)((tail_rng >> 16) % (uint32_t)max);
}

static const int32_t spd_up[8] = {
    69433, 73567, 77936, 82570, 87480, 92682, 98193, 104032
};
static const int32_t spd_down[8] = {
    61858, 58386, 55109, 52016, 49096, 46341, 43740, 41285
};

static void tail_trigger(int intens) {
    int tail_len = TAIL_MIN_LEN
                 + (int)(((int64_t)(TAIL_MAX_LEN - TAIL_MIN_LEN) * intens) >> 7);

    int semitones = 3 + tail_random(6);
    int dir = (tail_random(2) == 0) ? 1 : -1;

    int32_t target_spd = (dir > 0)
        ? spd_up[semitones - 1]
        : spd_down[semitones - 1];

    tail_spd = 65536;
    tail_spd_step = (target_spd - 65536) / tail_len;

    tail_amp = 65536;
    tail_amp_step = -(65536 / tail_len);

    tail_rpf = (uint32_t)tail_wp << 16;
    tail_fade_in = 0;
    tail_active = true;
}

static inline int32_t tail_read_ring(uint32_t rpf_val) {
    int ri = ((int)(rpf_val >> 16)) % TAIL_BUF_LEN;
    if (ri < 0) ri += TAIL_BUF_LEN;
    int rn = (ri + 1) % TAIL_BUF_LEN;
    int32_t frac = (int32_t)(rpf_val & 0xFFFF);
    int32_t a = (int32_t)tail_buf[ri] << 8;
    int32_t b = (int32_t)tail_buf[rn] << 8;
    return a + (int32_t)(((int64_t)(b - a) * frac) >> 16);
}

static void do_lofi(int32_t *s, int n, int intens) {
    int cmod = (CHO_MOD_MAX * intens) >> 7;
    int cwet = (CHO_WET_MAX * intens) >> 7;
    int cfb  = (CHO_FB_MAX  * intens) >> 7;
    int twet = (TAIL_WET_MAX * intens) >> 7;

    for (int i = 0; i < n; i++) {
        int32_t dry = s[i];

       
        int32_t abs_in = dry < 0 ? -dry : dry;
        onset_fast += ((abs_in - onset_fast) * ONSET_FAST_ALPHA) >> 8;
        onset_slow += ((abs_in - onset_slow) * ONSET_SLOW_ALPHA) >> 8;
        if (onset_cool > 0) onset_cool--;

        if (!tail_active && onset_cool == 0
            && onset_fast > ONSET_MIN_LEVEL
            && onset_fast > onset_slow * ONSET_RATIO) {
            tail_trigger(intens);
            onset_cool = ONSET_COOLDOWN;
        }

       
        int32_t write_in = clamp24(dry + (int32_t)(((int64_t)cho_fbs * cfb) >> 7));
        cho_buf[cho_wp & CHO_MASK] = write_in;
        cho_wp++;

       
        tail_ds_acc += dry;
        tail_ds_cnt++;
        if (tail_ds_cnt >= TAIL_DS) {
            int32_t avg = tail_ds_acc / TAIL_DS;
            int32_t v16 = avg >> 8;
            if (v16 >  32767) v16 =  32767;
            if (v16 < -32768) v16 = -32768;
            tail_buf[tail_wp] = (int16_t)v16;
            tail_wp = (tail_wp + 1) % TAIL_BUF_LEN;
            tail_ds_cnt = 0;
            tail_ds_acc = 0;
        }

       
        cho_lph = (cho_lph + 1) & (CHO_LFO_LEN - 1);
        int clfo = (cho_lph < CHO_LFO_HALF)
            ? (((cho_lph * cmod * 2) >> 15) - cmod)
            : (cmod - (((cho_lph - CHO_LFO_HALF) * cmod * 2) >> 15));

        int cho_delay = CHO_CENTER + clfo;
        if (cho_delay < 2) cho_delay = 2;
        if (cho_delay > CHO_LEN - 2) cho_delay = CHO_LEN - 2;
        int ri = (cho_wp - cho_delay) & CHO_MASK;
        int rn = (ri + 1) & CHO_MASK;
        int32_t ws = (cho_buf[ri] + cho_buf[rn]) >> 1;
        cho_fbs = clamp24(ws);
        int32_t cho_wet = (int32_t)(((int64_t)cho_fbs * cwet) >> 7);

       
        int32_t tail_out = 0;
        if (tail_active) {
            int32_t raw = tail_read_ring(tail_rpf);

            tail_lp += ((raw - tail_lp) * 50) >> 8;
            raw = tail_lp;

            tail_fade_in++;
            if (tail_fade_in < TAIL_FADE_LEN) {
                raw = (int32_t)(((int64_t)raw * tail_fade_in) / TAIL_FADE_LEN);
            }

            tail_out = (int32_t)(((int64_t)raw * tail_amp) >> 16);

            tail_rpf += (uint32_t)(tail_spd >> 2);

            int read_pos = ((int)(tail_rpf >> 16)) % TAIL_BUF_LEN;
            if (read_pos < 0) read_pos += TAIL_BUF_LEN;
            int dist = (tail_wp - read_pos + TAIL_BUF_LEN) % TAIL_BUF_LEN;
            if (dist < 100 || dist > TAIL_BUF_LEN - 100) {
                tail_active = false;
            }

            tail_spd += tail_spd_step;
            tail_amp += tail_amp_step;
            if (tail_amp <= 0) {
                tail_active = false;
                tail_amp = 0;
            }
        }

        int32_t tail_scaled = (int32_t)(((int64_t)tail_out * twet) >> 7);
        s[i] = clamp24(dry + cho_wet + tail_scaled);
    }
}




#define REV_MIN_FRAG  2400
#define REV_MAX_FRAG  48000
#define REV_WET_MAX   110
#define REV_FADE_IN   4096
#define REV_FADE_OUT  1024

static int     rev_state = 0;
static int     rev_pos = 0;
static int     rev_frag_len = 0;
static int     rev_skip = 0;
static int32_t rev_acc = 0;
static int     rev_play_pos = -1;
static int     rev_sub = 0;
static int32_t rev_wait_samples = 0;

#define MICRO_AP_LEN  523
#define MICRO_AP_G    45
static int32_t micro_ap[MICRO_AP_LEN];
static int     micro_ap_wp = 0;

static inline int32_t micro_ap_tick(int32_t in) {
    int32_t br = micro_ap[micro_ap_wp];
    micro_ap[micro_ap_wp] = clamp24(in + (int32_t)(((int64_t)br * MICRO_AP_G) >> 7));
    if (++micro_ap_wp >= MICRO_AP_LEN) micro_ap_wp = 0;
    return clamp24(br - (int32_t)(((int64_t)in * MICRO_AP_G) >> 7));
}

static inline int rev_fade128(int played_out, int remain_out) {
    int32_t f = 128;
    if (played_out < REV_FADE_IN) {
        int64_t t = played_out;
        int64_t T = REV_FADE_IN;
        int32_t fi = (int32_t)((t * t * 128) / (T * T));
        if (fi < f) f = fi;
    }
    if (remain_out < REV_FADE_OUT) {
        int32_t fo = (int32_t)((int64_t)remain_out * 128 / REV_FADE_OUT);
        if (fo < f) f = fo;
    }
    if (f < 0) f = 0;
    if (f > 128) f = 128;
    return (int)f;
}


static inline int32_t rev_compute_wait_to_next_beat(void) {
    uint32_t phase = transport_phase_samples % beat_period;
    if (phase == 0) return 0;
    return (int32_t)(beat_period - phase);
}

static void do_rev(int32_t *s, int n, int intens) {
    int frag = REV_MIN_FRAG + (int)(((int64_t)(REV_MAX_FRAG - REV_MIN_FRAG) * intens) >> 7);
    if (frag < REV_MIN_FRAG) frag = REV_MIN_FRAG;
    if (frag > REV_MAX_FRAG) frag = REV_MAX_FRAG;
    int rwet = (REV_WET_MAX * intens) >> 7;

    for (int i = 0; i < n; i++) {
        int32_t dry = s[i];

        if (rev_state == 0) {
            rev_acc += dry;
            rev_skip++;
            if (rev_skip >= 2) {
                int32_t avg = rev_acc >> 1;
                int32_t v16 = avg >> 8;
                if (v16 >  32767) v16 =  32767;
                if (v16 < -32768) v16 = -32768;
                fx_buf[rev_pos] = (int16_t)v16;
                rev_pos++;
                rev_skip = 0;
                rev_acc = 0;
            }
            if (rev_pos >= frag) {
                rev_frag_len = rev_pos;
                rev_wait_samples = rev_compute_wait_to_next_beat();
                rev_state = 1;
            }
            s[i] = dry;
        }
        else if (rev_state == 1) {
            if (rev_wait_samples > 0) rev_wait_samples--;
            if (rev_wait_samples <= 0) {
                rev_play_pos = rev_frag_len - 1;
                rev_sub = 0;
                rev_state = 2;
            }
            s[i] = dry;
        }
        else {
            if (rev_play_pos < 0) {
                rev_state = 0;
                rev_pos = 0;
                rev_skip = 0;
                rev_acc = 0;
                s[i] = dry;
                continue;
            }
            int32_t a = (int32_t)fx_buf[rev_play_pos] << 8;
            int32_t b = a;
            if (rev_play_pos > 0) b = (int32_t)fx_buf[rev_play_pos - 1] << 8;
            int32_t rev_sample = (rev_sub == 0) ? a : ((a + b) >> 1);

            int played_out = ((rev_frag_len - 1 - rev_play_pos) << 1) + rev_sub;
            int remain_out = (rev_play_pos << 1) + (1 - rev_sub);
            int f128 = rev_fade128(played_out, remain_out);

            rev_sample = micro_ap_tick(clamp24(rev_sample));
            int32_t aw = (int32_t)(((int64_t)rwet * f128) >> 7);
            int32_t rev_scaled = (int32_t)(((int64_t)rev_sample * aw) >> 7);
            s[i] = clamp24(dry + rev_scaled);

            rev_sub ^= 1;
            if (rev_sub == 0) rev_play_pos--;
        }
    }
}




#define GLITCH_BUF_LEN  4096
#define GLITCH_BUF_MASK 4095

#define GEV_NONE   0
#define GEV_HOLD   1
#define GEV_LOOP   2
#define GEV_JUMP   3
#define GEV_FREEZE 4

#define GLITCH_XFADE 96

static int     g_wp;
static int     g_rp;
static int     g_event;
static int     g_last_event;
static int     g_event_timer;
static int     g_event_total;
static int     g_beat_counter;
static int     g_next_event_at;
static int32_t g_hold_val;
static int     g_loop_start;
static int     g_loop_len;
static int     g_loop_pos;
static bool    g_writing;
static int32_t g_lp1 = 0;
static int32_t g_lp2 = 0;
static int32_t g_lp3 = 0;
static uint32_t g_rng = 0xCAFEBABE;

static int g_rand(int max) {
    g_rng = g_rng * 1664525u + 1013904223u;
    return (max > 0) ? (int)((g_rng >> 16) % (uint32_t)max) : 0;
}

static const int beat_divs[] = {
    32, 43, 48, 64, 85, 96, 128, 171
};
#define NUM_DIVS 8

static int glitch_next_interval(void) {
    int div = beat_divs[g_rand(NUM_DIVS)];
    int interval = (int)(((int64_t)beat_period * div) >> 8);
    if (interval < 200) interval = 200;
    return interval;
}

static void glitch_trigger_event(int intens) {
    int32_t S = ((int32_t)intens * intens) >> 7;
    if (S > 128) S = 128;

    int choices[4] = { GEV_HOLD, GEV_LOOP, GEV_JUMP, GEV_FREEZE };
    int pool[3]; int pc = 0;
    for (int j = 0; j < 4; j++) {
        if (choices[j] != g_last_event) pool[pc++] = choices[j];
    }
    g_event = pool[g_rand(pc)];
    g_last_event = g_event;

    int hold_len   = 200 + (int)(((int64_t)800 * S) >> 7);
    int loop_len   = 8 + (int)(((int64_t)600 * S) >> 7);
    int jump_back  = 1 + (int)(((int64_t)2000 * S) >> 7);
    int freeze_len = 200 + (int)(((int64_t)8000 * S) >> 7);

    int prob = 70 + (int)(((int64_t)55 * S) >> 7);
    if (g_rand(128) > prob) {
        g_event = GEV_NONE;
        g_event_timer = 0; g_event_total = 0;
        return;
    }

    int min_len = GLITCH_XFADE * 3;

    switch (g_event) {
    case GEV_HOLD:
        g_loop_len = 4 + g_rand(12);
        g_loop_start = (g_wp - g_loop_len) & GLITCH_BUF_MASK;
        g_loop_pos = 0;
        g_event_timer = hold_len;
        break;
    case GEV_LOOP:
        g_loop_len = loop_len;
        if (g_loop_len < 4) g_loop_len = 4;
        if (g_loop_len > 2048) g_loop_len = 2048;
        g_loop_start = (g_wp - g_loop_len) & GLITCH_BUF_MASK;
        g_loop_pos = 0;
        g_event_timer = g_loop_len * (2 + g_rand(4));
        break;
    case GEV_JUMP:
        g_rp = (g_wp - 1 - g_rand(jump_back)) & GLITCH_BUF_MASK;
        g_event_timer = 80 + g_rand(400);
        break;
    case GEV_FREEZE:
        g_rp = (g_wp - 1) & GLITCH_BUF_MASK;
        g_event_timer = freeze_len;
        break;
    default:
        g_event_timer = 0; break;
    }
    if (g_event_timer < min_len) g_event_timer = min_len;
    if (g_event_timer > 3500) g_event_timer = 3500;
    g_event_total = g_event_timer;
    g_writing = false;
}

static void do_clip(int32_t *s, int n, int intens) {
    int32_t S = ((int32_t)intens * intens) >> 7;
    if (S > 128) S = 128;

    int32_t thresh = 8388607 - (int32_t)(((int64_t)2516582 * S) >> 7);
    if (thresh < 5872025) thresh = 5872025;

    bool pure_wet = (intens >= 122);

    for (int i = 0; i < n; i++) {
        int32_t dry = s[i];

        if (g_writing) {
            int32_t v16 = dry >> 8;
            if (v16 > 32767) v16 = 32767;
            if (v16 < -32768) v16 = -32768;
            fx_buf[g_wp & GLITCH_BUF_MASK] = (int16_t)v16;
            g_wp = (g_wp + 1) & GLITCH_BUF_MASK;
        }

        g_beat_counter++;
        if (g_beat_counter >= g_next_event_at) {
            g_beat_counter = 0;
            g_next_event_at = glitch_next_interval();
            glitch_trigger_event(intens);
        }

        int32_t glitch;

        if (g_event_timer > 0 && g_event != GEV_NONE) {
            int played = g_event_total - g_event_timer;
            switch (g_event) {
            case GEV_HOLD:
            case GEV_LOOP:
                glitch = (int32_t)fx_buf[(g_loop_start + g_loop_pos) & GLITCH_BUF_MASK] << 8;
                g_loop_pos++;
                if (g_loop_pos >= g_loop_len) g_loop_pos = 0;
                break;
            case GEV_JUMP:
                glitch = (int32_t)fx_buf[g_rp & GLITCH_BUF_MASK] << 8;
                g_rp = (g_rp + 1) & GLITCH_BUF_MASK;
                break;
            case GEV_FREEZE:
                glitch = (int32_t)fx_buf[g_rp & GLITCH_BUF_MASK] << 8;
                break;
            default: glitch = dry; break;
            }

            if (glitch > thresh) glitch = thresh + ((glitch - thresh) >> 3);
            else if (glitch < -thresh) glitch = -thresh - ((-thresh - glitch) >> 3);

            if (played < GLITCH_XFADE) {
                glitch = (int32_t)(((int64_t)dry * (GLITCH_XFADE - played)
                        + (int64_t)glitch * played) / GLITCH_XFADE);
            }
            if (g_event_timer <= GLITCH_XFADE) {
                glitch = (int32_t)(((int64_t)glitch * g_event_timer
                        + (int64_t)dry * (GLITCH_XFADE - g_event_timer)) / GLITCH_XFADE);
            }

            g_event_timer--;
            if (g_event_timer <= 0) {
                g_writing = true;
                g_event = GEV_NONE;
            }
        } else {
            glitch = dry;
            if (!g_writing) g_writing = true;
        }

        g_lp1 += ((glitch - g_lp1) * 32) >> 8;
        g_lp2 += ((g_lp1 - g_lp2) * 40) >> 8;
        g_lp3 += ((g_lp2 - g_lp3) * 48) >> 8;
        glitch = g_lp3;

        if (pure_wet) {
            s[i] = clamp24(glitch);
        } else {
            int dry_mix = 128 - intens;
            if (dry_mix < 0) dry_mix = 0;
            s[i] = clamp24(glitch + (int32_t)(((int64_t)dry * dry_mix) >> 7));
        }
    }
}




#define STU_FADE  64
static uint32_t stu_counter = 0;
static int      stu_beat = 0;

static void do_stu(int32_t *s, int n, int intens) {
    uint32_t sixteenth = beat_period >> 2;
    if (sixteenth < 128) sixteenth = 128;
    int depth = intens;
    for (int i = 0; i < n; i++) {
        stu_counter++;
        if (stu_counter >= sixteenth) { stu_counter = 0; stu_beat = (stu_beat + 1) & 3; }
        if (stu_beat & 1) {
            int gain = 128 - depth;
            if (stu_counter < STU_FADE) {
                int fade = 128 - (int)(((int64_t)(128 - gain) * stu_counter) / STU_FADE);
                s[i] = (int32_t)(((int64_t)s[i] * fade) >> 7);
            } else if (stu_counter >= sixteenth - STU_FADE) {
                int remain = sixteenth - stu_counter;
                int fade = 128 - (int)(((int64_t)(128 - gain) * remain) / STU_FADE);
                s[i] = (int32_t)(((int64_t)s[i] * fade) >> 7);
            } else {
                s[i] = (int32_t)(((int64_t)s[i] * gain) >> 7);
            }
        }
    }
}




#define VINYL_BASE   8
#define VINYL_RANGE  4800
#define VINYL_DENS   3
static uint32_t vinyl_rng   = 0xACE1ACE1u;
static uint32_t vinyl_rng2  = 0xBEEF1234u;
static int32_t  vinyl_state = 0;
static int      vinyl_tick  = 0;

static void add_vinyl_noise(uint32_t *b, int n, int intens) {
    int32_t *s = (int32_t *)b;
    int32_t vamp = VINYL_BASE + (int32_t)(((int64_t)VINYL_RANGE * intens) >> 7);
    for (int i = 0; i < n; i++) {
        int32_t noise = 0;
        vinyl_tick++;
        if (vinyl_tick >= VINYL_DENS) {
            vinyl_tick = 0;
            vinyl_rng = vinyl_rng * 1664525u + 1013904223u;
            int32_t raw = (int32_t)((int16_t)(vinyl_rng >> 16));
            int32_t absraw = raw < 0 ? -raw : raw;
            if (absraw < 20000) raw = 0;
            else raw = (raw > 0) ? (absraw - 20000) : -(absraw - 20000);
            vinyl_rng2 = vinyl_rng2 * 1664525u + 1013904223u;
            vinyl_state = (int32_t)(((int64_t)180 * raw + (int64_t)76 * vinyl_state) >> 8);
            noise = vinyl_state;
        }
        noise = (int32_t)(((int64_t)noise * vamp) >> 15);
        int32_t pcm = (s[i] >> 8) + noise;
        s[i] = clamp24(pcm) << 8;
    }
}




static void do_all(int32_t *s, int n, int intens) {
    do_lofi(s, n, intens);
    do_rev(s, n, intens);
    do_clip(s, n, intens);
}

static void process_effect(int32_t *s, int n, int mode, int intens) {
    switch (mode) {
    case MODE_LOFI: do_lofi(s, n, intens); break;
    case MODE_REV:  do_rev(s, n, intens);  break;
    case MODE_CLIP: do_clip(s, n, intens); break;
    case MODE_STU:  do_stu(s, n, intens);  break;
    case MODE_ALL:  do_all(s, n, intens);  break;
    default: break;
    }
}

static void process_block(uint32_t *b, int n) {
    int32_t *s = (int32_t *)b;
    for (int i = 0; i < n; i++) s[i] = (s[i] >> 8) >> 1;

    if (!fx_active) {
        for (int i = 0; i < n; i++) s[i] = clamp24(s[i]) << 8;
        return;
    }

    if (xfade_cnt > 0) {
        int32_t dry_copy[BLOCK];
        memcpy(dry_copy, s, n * sizeof(int32_t));
        process_effect(s, n, cur_mode, effective_intensity);
        process_effect(dry_copy, n, xfade_from, effective_intensity);
        for (int i = 0; i < n; i++) {
            int fn = XFADE_LEN - xfade_cnt, fo = xfade_cnt;
            s[i] = (int32_t)(((int64_t)s[i] * fn + (int64_t)dry_copy[i] * fo) / XFADE_LEN);
            if (xfade_cnt > 0) xfade_cnt--;
        }
    } else {
        process_effect(s, n, cur_mode, effective_intensity);
    }

    for (int i = 0; i < n; i++) s[i] = clamp24(s[i]) << 8;
}




int main(void) {
    sleep_ms(2000);

    gpio_init(PIN_LED1); gpio_set_dir(PIN_LED1, GPIO_OUT);
    gpio_init(PIN_LED2); gpio_set_dir(PIN_LED2, GPIO_OUT);
    gpio_init(PIN_FS1);  gpio_set_dir(PIN_FS1,  GPIO_IN);
    gpio_init(PIN_FS2);  gpio_set_dir(PIN_FS2,  GPIO_IN);
    gpio_init(PIN_FS3);  gpio_set_dir(PIN_FS3,  GPIO_IN);
    adc_init();
    adc_gpio_init(PIN_TYPE);
    adc_gpio_init(PIN_INTENS);

   
    uart_init(DISPLAY_UART, DISPLAY_BAUD);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);
    uart_set_fifo_enabled(DISPLAY_UART, true);

    bpm = DEFAULT_BPM;
    update_beat_period();

    uint off_mclk = pio_add_program(pio0, &i2s_mclk_program);
    uint off_clk  = pio_add_program(pio0, &i2s_clk_program);
    uint off_rx   = pio_add_program(pio0, &i2s_rx_program);
    uint off_tx   = pio_add_program(pio1, &i2s_tx_program);
    i2s_mclk_program_init(pio0, 0, off_mclk, PIN_MCLK);
    pio_sm_set_enabled(pio0, 0, true);
    sleep_ms(100);
    i2s_clk_program_init(pio0, 1, off_clk, PIN_BCK, PIN_LRCK);
    pio_sm_set_enabled(pio0, 1, true);
    sleep_ms(300);
    i2s_rx_program_init(pio0, 2, off_rx, PIN_DOUT);
    i2s_tx_program_init(pio1, 0, off_tx, PIN_DIN, PIN_DOUT);
    pio_sm_set_enabled(pio1, 0, true);
    pio_sm_set_enabled(pio0, 2, true);

    dma_rx_ch = dma_claim_unused_channel(true);
    dma_tx_ch = dma_claim_unused_channel(true);

    dma_channel_config rc = dma_channel_get_default_config(dma_rx_ch);
    channel_config_set_transfer_data_size(&rc, DMA_SIZE_32);
    channel_config_set_read_increment(&rc, false);
    channel_config_set_write_increment(&rc, true);
    channel_config_set_dreq(&rc, pio_get_dreq(pio0, 2, false));
    dma_channel_configure(dma_rx_ch, &rc, buf[0], &pio0->rxf[2], BLOCK, false);

    dma_channel_config tc = dma_channel_get_default_config(dma_tx_ch);
    channel_config_set_transfer_data_size(&tc, DMA_SIZE_32);
    channel_config_set_read_increment(&tc, true);
    channel_config_set_write_increment(&tc, false);
    channel_config_set_dreq(&tc, pio_get_dreq(pio1, 0, true));
    dma_channel_configure(dma_tx_ch, &tc, &pio1->txf[0], buf[0], BLOCK, false);

    memset(cho_buf, 0, sizeof(cho_buf));
    memset(fx_buf, 0, sizeof(fx_buf));
    memset(micro_ap, 0, sizeof(micro_ap));
    memset(buf, 0, sizeof(buf));
    memset(tail_buf, 0, sizeof(tail_buf));

    cho_wp = 0; cho_lph = 0; cho_fbs = 0;
    onset_fast = 0; onset_slow = 0; onset_cool = 0;
    tail_active = false; tail_amp = 0; tail_lp = 0;
    tail_wp = 0; tail_ds_cnt = 0; tail_ds_acc = 0;
    tail_fade_in = 0;

    rev_state = 0; rev_pos = 0; rev_frag_len = 0;
    rev_skip = 0; rev_acc = 0;
    rev_play_pos = -1; rev_sub = 0; rev_wait_samples = 0;
    micro_ap_wp = 0;

    g_wp = 0; g_rp = 0; g_event = GEV_NONE; g_last_event = GEV_NONE;
    g_event_timer = 0; g_event_total = 0; g_beat_counter = 0;
    g_next_event_at = beat_period;
    g_lp1 = 0; g_lp2 = 0; g_lp3 = 0; g_writing = true;

    stu_counter = 0; stu_beat = 0;
    kt_raw = ki_raw = 2048;
    led1_phase_samples = 0;
    transport_phase_samples = 0;
    peak_level = 0;
    uart_skip = 0;

    dma_rx_go(buf[0]); wait_rx();
    dma_tx_go(buf[2]); dma_rx_go(buf[1]);
    read_controls();
    process_block(buf[0], BLOCK);
    wait_rx(); wait_tx();

    int itx = 0, irx = 2, iproc = 1;
    while (true) {
        if (fx_active && cur_mode != MODE_CLIP) {
            add_vinyl_noise(buf[itx], BLOCK, effective_intensity);
        }
        dma_tx_go(buf[itx]);
        dma_rx_go(buf[irx]);
        read_controls();
        process_block(buf[iproc], BLOCK);

       
        update_peak((int32_t *)buf[iproc], BLOCK);
        send_display_packet();
        tud_task();            
        check_usb_upgrade();   

        wait_tx(); wait_rx();
        int otx = itx; itx = iproc; iproc = irx; irx = otx;
    }
}