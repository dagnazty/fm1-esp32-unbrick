// fm1-esp32-unbrick - the few Pico SDK calls the ported Transporter sources use, on ESP-IDF.
#pragma once

#include <stdint.h>

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_DP 20   // GPIO20: USB D+ of the S3's own USB port
#define PIN_DM 19   // GPIO19: USB D-

static inline uint64_t time_us_64(void) { return (uint64_t)esp_timer_get_time(); }
static inline void sleep_us(uint32_t us) { esp_rom_delay_us(us); }
static inline void sleep_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1)); }
#define __dmb() __sync_synchronize()
