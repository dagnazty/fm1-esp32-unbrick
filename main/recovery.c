// fm1-esp32-unbrick - JieLi USB_KEY entry and ROM keep-alive pulses, on an ESP32-S3.
//
// A port of FM-1 Transporter src/recovery.c and pio/usb_key.pio (MIT, Leo Kuroshita / Hügelton Instruments).
// The RP2040 sent these with PIO at 1 MHz; here they are timed register writes with interrupts off on core 1,
// with the same timings (us):
//   key packet  0x16EF MSB first, polarity A: D+ the clock, D- the data. Each bit: data set with D+ low for 8,
//               D+ high for 12 (10 for the last bit); both pins driven low before and released after.
//   gap         4 probes 35 apart: D- driven high for 2 and read back (a target holding it low = ACK),
//               D+ high twice in a row = the ROM already pulled up.
//   pulses      D+ driven low for 4 every 1000 (the ROM trims its clock from them) until it releases D+
//               after calibration (>= 1 s) or 6 s have passed; the host must start then.
// Hardware findings behind these (fm-1-research-lab, 2026-09-30 / 10-01) are in the original's comments.

#include "recovery.h"

#include "compat.h"
#include "driver/gpio.h"
#include "esp_private/periph_ctrl.h"
#include "freertos/FreeRTOS.h"
#include "hal/usb_serial_jtag_ll.h"
#include "log.h"
#include "soc/gpio_reg.h"

#define USB_KEY_WORD 0x16EF
#define KEY_PACKET_US 340
#define GAP_PROBES 4
#define GAP_PROBE_SPACING_US 35
#define ACK_PROBE_LOWS 2
#define DP_HIGH_TIMEOUT_MS 500
#define PULSE_SAMPLE_US 100
#define PULSE_LOW_SAMPLES 3         // 300 us low = not just one of our 4 us pulses
#define HANDOFF_PULSE_MS 6000
#define CAL_DONE_MIN_MS 1000

#define BIT_DP (1u << PIN_DP)
#define BIT_DM (1u << PIN_DM)

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static inline void out_low(uint32_t m) { REG_WRITE(GPIO_OUT_W1TC_REG, m); }
static inline void out_high(uint32_t m) { REG_WRITE(GPIO_OUT_W1TS_REG, m); }
static inline void drive(uint32_t m) { REG_WRITE(GPIO_ENABLE_W1TS_REG, m); }
static inline void release(uint32_t m) { REG_WRITE(GPIO_ENABLE_W1TC_REG, m); }
static inline bool dp(void) { return REG_READ(GPIO_IN_REG) & BIT_DP; }
static inline bool dm(void) { return REG_READ(GPIO_IN_REG) & BIT_DM; }

void recovery_release_bus(void) {
    release(BIT_DP | BIT_DM);
    out_low(BIT_DP | BIT_DM);
    gpio_pulldown_dis(PIN_DP);
    gpio_pulldown_dis(PIN_DM);
}

static void bus_listen(void) {
    release(BIT_DP | BIT_DM);
    out_low(BIT_DP | BIT_DM);
    gpio_pulldown_en(PIN_DP);       // as the RP2040 pads: weak pull-downs, the target's pull-up wins
    gpio_pulldown_en(PIN_DM);
}

static bool probe_high(uint32_t bit) {
    out_high(bit);
    drive(bit);
    esp_rom_delay_us(2);
    bool v = REG_READ(GPIO_IN_REG) & bit;
    release(bit);
    out_low(bit);
    return v;
}

static void key_send_packet(void) {
    portENTER_CRITICAL(&mux);
    out_low(BIT_DP | BIT_DM);
    drive(BIT_DP | BIT_DM);
    esp_rom_delay_us(3);
    for (int i = 15; i >= 0; i--) {
        if ((USB_KEY_WORD >> i) & 1) out_high(BIT_DM);
        else out_low(BIT_DM);
        esp_rom_delay_us(8);
        out_high(BIT_DP);
        esp_rom_delay_us(i ? 12 : 10);
        out_low(BIT_DP);
    }
    release(BIT_DP | BIT_DM);
    out_low(BIT_DM);
    portEXIT_CRITICAL(&mux);
    esp_rom_delay_us(KEY_PACKET_US - 3 - 16 * 20);
}

static int gap_sense(bool *alive) {
    int lows = 0, dp_highs = 0;
    for (int i = 0; i < GAP_PROBES; i++) {
        esp_rom_delay_us(GAP_PROBE_SPACING_US);
        if (dp()) {
            if (++dp_highs >= 2) return 2;
        } else {
            dp_highs = 0;
        }
        if (probe_high(BIT_DM)) {
            *alive = true;
            lows = 0;
        } else if (*alive && ++lows >= ACK_PROBE_LOWS) {
            return 1;
        }
    }
    return 0;
}

static void pulse(void) {
    portENTER_CRITICAL(&mux);
    out_low(BIT_DP);
    drive(BIT_DP);
    esp_rom_delay_us(4);
    release(BIT_DP);
    portEXIT_CRITICAL(&mux);
}

// 1 ms pulses for up to ms; stop_on_release: return once the ROM drops D+ after calibration
static void pulses(uint32_t ms, bool stop_on_release) {
    gpio_set_drive_capability(PIN_DP, GPIO_DRIVE_CAP_1);
    uint64_t start = time_us_64(), next = start;
    uint32_t low_samples = 0;
    for (;;) {
        uint64_t now = time_us_64();
        if (now >= next) {
            pulse();
            next += 1000;
        }
        esp_rom_delay_us(PULSE_SAMPLE_US);
        now = time_us_64();
        uint64_t elapsed = now - start;
        if (stop_on_release && !dp()) {
            if (++low_samples == PULSE_LOW_SAMPLES) {
                if (elapsed >= CAL_DONE_MIN_MS * 1000ull) {
                    dlog("PULSE: D+ released after %.1f ms - calibration done, host takes over", elapsed / 1000.0);
                    break;
                }
                dlog("PULSE: early D+ blip at %.1f ms, ignored", elapsed / 1000.0);
            }
        } else {
            low_samples = 0;
        }
        if (elapsed >= ms * 1000ull) {
            dlog("PULSE: %lu ms of pulses, D+=%d - handing off", (unsigned long)ms, dp());
            break;
        }
    }
    gpio_set_drive_capability(PIN_DP, GPIO_DRIVE_CAP_0);
}

static bool wait_dp_high(uint32_t timeout_ms) {
    uint64_t deadline = time_us_64() + timeout_ms * 1000ull;
    int highs = 0, dm_highs = 0;
    while (time_us_64() < deadline) {
        if (dp()) {
            if (++highs >= 5) return true;
        } else {
            highs = 0;
        }
        if (dm()) {
            if (++dm_highs == 5) dlog("D- pulled up instead of D+ - are D+/D- swapped?");
        } else {
            dm_highs = 0;
        }
        esp_rom_delay_us(50);
    }
    return false;
}

void recovery_run(void) {
    uint32_t packets = 0;
    bool alive = false, was_alive = false;
    uint64_t last_status = time_us_64();

    dlog("KEY: sending 0x%04X, polarity A (D+ clock, D- data). Switch the FM-1 ON now.", USB_KEY_WORD);
    for (;;) {
        key_send_packet();
        packets++;
        int s = gap_sense(&alive);
        if (alive && !was_alive) {
            dlog("data line probe responds (not proof of a target)");
            was_alive = true;
        }
        if (s) {
            bus_listen();
            dlog("%s after %lu packets", s == 1 ? "ACK (data line held low)" : "D+ high in gap (ACK missed?)",
                 (unsigned long)packets);
            if (s == 2 || wait_dp_high(DP_HIGH_TIMEOUT_MS)) {
                dlog("PULSE: D+ high (ROM pull-up), sending 4 us pulses every 1 ms");
                pulses(HANDOFF_PULSE_MS, true);
                return;
            }
            dlog("no D+ pull-up within %d ms (D+=%d D-=%d) - false ACK, resuming key", DP_HIGH_TIMEOUT_MS, dp(), dm());
            alive = was_alive = false;
            continue;
        }
        uint64_t now = time_us_64();
        if (now - last_status > 2000000) {
            dlog("KEY: %lu packets", (unsigned long)packets);
            last_status = now;
        }
    }
}

void recovery_pulses_only(uint32_t ms) {
    dlog("PULSE: %lu ms of 1 ms pulses for an already attached target", (unsigned long)ms);
    pulses(ms, false);
}

bool recovery_wait_detached(uint32_t timeout_ms) {
    uint64_t deadline = time_us_64() + timeout_ms * 1000ull;
    int lows = 0;
    while (time_us_64() < deadline) {
        if (!dp()) {
            if (++lows >= 20) return true;      // 2 ms steadily low
        } else {
            lows = 0;
        }
        esp_rom_delay_us(100);
    }
    return false;
}

bool recovery_target_attached(void) {
    for (int i = 0; i < 50; i++) {
        if (!dp()) return false;
        esp_rom_delay_us(1000);
    }
    return true;
}

// The USB-Serial-JTAG block can keep the pads (and its 1.5 kOhm D+ pull-up) across resets that IDF's startup
// does not treat as power-on (seen: D+ high with nothing attached after an EN reset). Take them back explicitly.
static void usj_release_pads(void) {
    PERIPH_RCC_ATOMIC() {
        usb_serial_jtag_ll_enable_bus_clock(true);
    }
    usb_serial_jtag_pull_override_vals_t off = {.dp_pu = false, .dm_pu = false, .dp_pd = false, .dm_pd = false};
    usb_serial_jtag_ll_phy_enable_pull_override(&off);
    usb_serial_jtag_ll_phy_enable_pad(false);
}

void recovery_init(void) {
    usj_release_pads();
    gpio_config_t c = {
        .pin_bit_mask = BIT_DP | BIT_DM,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&c);
    gpio_set_drive_capability(PIN_DP, GPIO_DRIVE_CAP_0);    // weak, as the RP2040's 2 mA: a target can hold a line low
    gpio_set_drive_capability(PIN_DM, GPIO_DRIVE_CAP_0);
    bus_listen();
    esp_rom_delay_us(1000);
    dlog("bus idle: D+=%d D-=%d (both 0 with nothing attached)", dp(), dm());
}
