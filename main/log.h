#pragma once

#include <stdint.h>

// The log stays on the board (UART0 is the data port): a ring of text lines, read with the `log` request.
void dlog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Copies the log written since the last call into buf (at most len bytes); returns the bytes copied.
uint32_t log_take(char *buf, uint32_t len);
