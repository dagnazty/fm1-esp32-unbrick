#include "log.h"

#include <stdarg.h>
#include <stdio.h>

#include "compat.h"
#include "freertos/FreeRTOS.h"

#define RING 16384      // power of two

static char ring[RING];
static uint32_t head, tail;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void dlog(const char *fmt, ...) {
    char line[160];
    int n = snprintf(line, sizeof(line), "%8.3f ", time_us_64() / 1e6);
    va_list ap;
    va_start(ap, fmt);
    n += vsnprintf(line + n, sizeof(line) - n - 1, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof(line) - 2) n = sizeof(line) - 2;
    line[n++] = '\n';
    portENTER_CRITICAL(&mux);
    for (int i = 0; i < n; i++) {
        ring[head++ & (RING - 1)] = line[i];
        if (head - tail > RING) tail = head - RING;     // the oldest lines go
    }
    portEXIT_CRITICAL(&mux);
}

uint32_t log_take(char *buf, uint32_t len) {
    uint32_t n = 0;
    portENTER_CRITICAL(&mux);
    while (tail != head && n < len) buf[n++] = ring[tail++ & (RING - 1)];
    portEXIT_CRITICAL(&mux);
    return n;
}
