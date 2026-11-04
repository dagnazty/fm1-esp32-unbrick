// fm1-esp32-unbrick - the fm1t line protocol on UART0.
// Ported from FM-1 Transporter src/transporter_proto.c (MIT, Leo Kuroshita): the same requests, replies, CRCs
// and write gate. The RP2040's core 0 / core 1 split becomes a receive task and the worker task.

#include "proto.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "compat.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "host.h"
#include "jieli_uboot.h"
#include "log.h"

#define PORT UART_NUM_0
#define BAUD 115200             // the CH340 + macOS driver on the board this was made for drops bytes above it
#define PAYLOAD_TIMEOUT_MS 3000 // a wsec / ldr / memw payload that stops arriving is given up (bytes were lost)
#define REQ_LINE_MAX 64
#define STATUS_HOLD_MS 20000

RTC_NOINIT_ATTR uint32_t fm1_rekey_magic;      // main.c: "KEY1" = force the USB_KEY after this restart

// receive task -> worker: one request line at a time, plus the payload of wsec / ldr / memw
static char rx_line[REQ_LINE_MAX];
static unsigned rx_len;
static uint8_t sector[JL_SECTOR_SIZE];
static unsigned sector_fill, rx_expect;
static bool rx_binary;
static uint64_t rx_binary_us;   // when the payload began
static char req[REQ_LINE_MAX];
static volatile bool req_pending;

static uint8_t io[JL_READ_MAX];

static void tx_bytes(const void *data, uint32_t len) {
    uart_write_bytes(PORT, data, len);
}

static void tx_line(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void tx_line(const char *fmt, ...) {
    char line[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line) - 1, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(line) - 2) n = (int)sizeof(line) - 2;
    line[n++] = '\n';
    tx_bytes(line, (uint32_t)n);
}

static void tx_log(void) {
    static char buf[16384];
    uint32_t n = log_take(buf, sizeof(buf));
    tx_line("LOG %lu", (unsigned long)n);
    tx_bytes(buf, n);
    tx_line("END");
}

//--------------------------------------------------------------------+
// receive task
//--------------------------------------------------------------------+

static void rx_bytes(const uint8_t *buf, unsigned len) {
    for (unsigned i = 0; i < len; i++) {
        if (rx_binary) {
            unsigned n = len - i;
            if (n > rx_expect - sector_fill) n = rx_expect - sector_fill;
            memcpy(&sector[sector_fill], &buf[i], n);
            sector_fill += n;
            i += n - 1;
            if (sector_fill == rx_expect) {
                rx_binary = false;
                __dmb();
                req_pending = true;         // req already holds the wsec / ldr / memw line
            }
            continue;
        }
        char c = (char)buf[i];
        if (c == '\r') continue;
        if (c != '\n') {
            if (rx_len < REQ_LINE_MAX - 1) rx_line[rx_len++] = c;
            continue;
        }
        rx_line[rx_len] = 0;
        rx_len = 0;
        if (!strcmp(rx_line, "rekey")) {
            tx_line("OK rekey");
            uart_wait_tx_done(PORT, pdMS_TO_TICKS(100));
            fm1_rekey_magic = 0x4B455931;
            esp_restart();
        }
        if (!strcmp(rx_line, "ping")) {     // answered here, while the worker may still be keying
            tx_line("PONG");
            continue;
        }
        if (!strcmp(rx_line, "log")) {
            tx_log();
            continue;
        }
        if (req_pending) continue;          // one request at a time; drop extras
        memcpy(req, rx_line, sizeof(req));
        if (!strncmp(rx_line, "wsec ", 5)) {
            sector_fill = 0;
            rx_expect = JL_SECTOR_SIZE;
            rx_binary = true;               // the request fires once 4 KiB arrived
            rx_binary_us = time_us_64();
            continue;
        }
        if (!strncmp(rx_line, "ldr ", 4) || !strncmp(rx_line, "memw ", 5)) {
            // ldr <off> <len> <crc32> / memw <addr> <len> <crc32>, then <len> raw bytes (1..4096)
            char *a = strchr(rx_line + 4, ' ');
            unsigned long n = a ? strtoul(a + 1, NULL, 0) : 0;
            if (n >= 1 && n <= JL_SECTOR_SIZE) {
                sector_fill = 0;
                rx_expect = (unsigned)n;
                rx_binary = true;
                rx_binary_us = time_us_64();
                continue;
            }
            // bad length: the worker answers ERR
        }
        __dmb();
        req_pending = true;
    }
}

static void rx_task(void *arg) {
    (void)arg;
    uint8_t buf[512];
    for (;;) {
        int n = uart_read_bytes(PORT, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n > 0) rx_bytes(buf, (unsigned)n);
        if (rx_binary && time_us_64() - rx_binary_us > PAYLOAD_TIMEOUT_MS * 1000ull) {
            // Bytes were lost on the way in: nothing was written (the request never fired). The same reply as
            // a CRC mismatch, which the client may safely retry.
            dlog("PROTO: payload stopped at %u of %u bytes - dropped", sector_fill, rx_expect);
            rx_binary = false;
            tx_line("ERR crc short");
        }
    }
}

void proto_start(void) {
    uart_config_t cfg = {
        .baud_rate = BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PORT, 16384, 16384, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PORT, &cfg));
    xTaskCreatePinnedToCore(rx_task, "proto_rx", 6144, NULL, 4, NULL, 0);
}

//--------------------------------------------------------------------+
// worker
//--------------------------------------------------------------------+

// zlib-compatible CRC-32, so the Mac side can use zlib.crc32.
static uint32_t crc32_update(uint32_t crc, const uint8_t *p, uint32_t len) {
    crc = ~crc;
    while (len--) {
        crc ^= *p++;
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
    }
    return ~crc;
}

static void do_write_sector(uint32_t addr, uint32_t crc) {
    uint32_t got = crc32_update(0, sector, JL_SECTOR_SIZE);
    if (got != crc) {
        tx_line("ERR crc %08lX", (unsigned long)got);
        return;
    }
    if (addr < JL_WRITE_MIN || addr >= JL_WRITE_END || (addr % JL_SECTOR_SIZE)) {
        tx_line("ERR range");
        return;
    }
    dlog("PROTO: write sector %06lX", (unsigned long)addr);
    tx_line(jl_flash_write_sector(addr, sector) ? "OK" : "ERR write");
}

static void do_read(uint32_t addr, uint32_t len) {
    if (len == 0 || addr >= JL_FLASH_SIZE || len > JL_FLASH_SIZE - addr) {
        tx_line("ERR range");
        return;
    }
    if (!jl_ensure_loader()) {
        tx_line("ERR loader");
        return;
    }
    dlog("PROTO: read %06lX+%lu", (unsigned long)addr, (unsigned long)len);
    uint64_t t0 = time_us_64();
    tx_line("DATA %lu", (unsigned long)len);
    uint32_t crc = 0, chunk = jl_read_chunk();
    for (uint32_t off = 0; off < len; off += chunk) {
        uint16_t n = (uint16_t)((len - off) < chunk ? (len - off) : chunk);
        bool ok = false;
        for (int attempt = 0; attempt < 3 && !ok; attempt++) ok = jl_flash_read(addr + off, n, io);
        if (!ok) {
            // The Mac side counts bytes, so the stream cannot be resynced: zeros, and the failure in END.
            memset(io, 0, n);
            tx_bytes(io, n);
            for (off += n; off < len; off += chunk) {
                uint32_t m = (len - off) < chunk ? (len - off) : chunk;
                tx_bytes(io, m);
            }
            tx_line("END FAIL");
            return;
        }
        crc = crc32_update(crc, io, n);
        tx_bytes(io, n);
    }
    tx_line("END %08lX", (unsigned long)crc);
    dlog("PROTO: read done, %lu bytes in %.2f s, crc32 %08lX", (unsigned long)len, (time_us_64() - t0) / 1e6,
         (unsigned long)crc);
}

static void do_memw(uint32_t addr, uint32_t len, uint32_t crc) {
    if (len < 1 || len > JL_SECTOR_SIZE) {
        tx_line("ERR len");
        return;
    }
    uint32_t got = crc32_update(0, sector, len);
    if (got != crc) {
        tx_line("ERR crc %08lX", (unsigned long)got);
        return;
    }
    if (jl_loader_running()) {
        tx_line("ERR loader-running");
        return;
    }
    for (uint32_t off = 0; off < len; off += JL_IO_SIZE) {
        uint16_t n = (uint16_t)((len - off) < JL_IO_SIZE ? (len - off) : JL_IO_SIZE);
        if (!jl_rom_mem_write(addr + off, &sector[off], n)) {
            tx_line("ERR memw at %08lX", (unsigned long)(addr + off));
            return;
        }
    }
    tx_line("OK");
}

static void do_memr(uint32_t addr, uint32_t len) {
    if (len < 1 || len > JL_READ_MAX) {
        tx_line("ERR len");
        return;
    }
    if (jl_loader_running()) {
        tx_line("ERR loader-running");
        return;
    }
    for (uint32_t off = 0; off < len; off += JL_IO_SIZE) {
        uint16_t n = (uint16_t)((len - off) < JL_IO_SIZE ? (len - off) : JL_IO_SIZE);
        if (!jl_rom_mem_read(addr + off, n, &io[off])) {
            tx_line("ERR memr at %08lX", (unsigned long)(addr + off));
            return;
        }
    }
    tx_line("DATA %lu", (unsigned long)len);
    tx_bytes(io, len);
    tx_line("END %08lX", (unsigned long)crc32_update(0, io, len));
}

static void do_ldr(uint32_t off, uint32_t len, uint32_t crc) {
    uint32_t got = crc32_update(0, sector, len);
    if (got != crc) {
        tx_line("ERR crc %08lX", (unsigned long)got);
        return;
    }
    if (jl_loader_running()) {
        tx_line("ERR loader-running");
        return;
    }
    tx_line(jl_loader_load(off, sector, len) ? "OK" : "ERR loader-range");
}

static void do_jump(uint32_t addr, uint32_t arg) {
    if (jl_loader_running()) {
        tx_line("ERR loader-running");
        return;
    }
    dlog("PROTO: jump %08lX arg %04lX", (unsigned long)addr, (unsigned long)arg);
    uint64_t t0 = time_us_64();
    bool ok = jl_rom_jump(addr, (uint16_t)arg);
    uint32_t ms = (uint32_t)((time_us_64() - t0) / 1000);
    if (ok) tx_line("OK ms=%lu", (unsigned long)ms);
    else tx_line("ERR jump ms=%lu", (unsigned long)ms);
}

void proto_poll(void) {
    if (!req_pending) {
        jl_keepalive();
        return;
    }
    __dmb();
    if (!strcmp(req, "status") && !fm1_host_uboot_ready()) {
        uint64_t hs = fm1_host_started_us();
        if (!hs || time_us_64() - hs < STATUS_HOLD_MS * 1000ull) return;     // hold it until UBOOT is up
    }
    char line[REQ_LINE_MAX];
    memcpy(line, req, sizeof(line));
    req_pending = false;

    char *argv[4] = {0};
    int argc = 0;
    for (char *tok = strtok(line, " \t"); tok && argc < 4; tok = strtok(NULL, " \t")) argv[argc++] = tok;
    if (!argc) return;

    if (!strcmp(argv[0], "status")) {
        tx_line("OK uboot=%d v15=%d loader=%d loader_running=%d", fm1_host_uboot_ready(), fm1_host_v15_mounted(),
                jl_have_loader(), jl_loader_running());
    } else if (!strcmp(argv[0], "uboot")) {
        if (fm1_host_uboot_ready()) tx_line("OK uboot");
        else if (fm1_host_softkey()) tx_line("OK softkey");
        else tx_line("ERR no-v15");
    } else if (!strcmp(argv[0], "info")) {
        jl_info_t info;
        if (!fm1_host_uboot_ready()) {
            tx_line("ERR no-uboot");
        } else if (!jl_info(&info)) {
            tx_line("ERR info");
        } else {
            dlog("PROTO: chip key %04X, device type %u, flash id %06lX", info.chip_key, info.dev_type,
                 (unsigned long)info.flash_id);
            tx_line("OK key=%04X type=%u id=%06lX", info.chip_key, info.dev_type, (unsigned long)info.flash_id);
        }
    } else if (!strcmp(argv[0], "read") && argc == 3) {
        if (!fm1_host_uboot_ready()) tx_line("ERR no-uboot");
        else do_read(strtoul(argv[1], NULL, 0), strtoul(argv[2], NULL, 0));
    } else if (!strcmp(argv[0], "runapp")) {
        tx_line(jl_run_app() ? "OK" : "ERR no-loader");
    } else if (!strcmp(argv[0], "wsec") && argc == 3) {
        if (!fm1_host_uboot_ready()) tx_line("ERR no-uboot");
        else do_write_sector(strtoul(argv[1], NULL, 0), strtoul(argv[2], NULL, 16));
    } else if (!strcmp(argv[0], "ldr") && argc == 4) {
        do_ldr(strtoul(argv[1], NULL, 0), strtoul(argv[2], NULL, 0), strtoul(argv[3], NULL, 16));
    } else if (!strcmp(argv[0], "memw") && argc == 4) {
        if (!fm1_host_uboot_ready()) tx_line("ERR no-uboot");
        else do_memw(strtoul(argv[1], NULL, 0), strtoul(argv[2], NULL, 0), strtoul(argv[3], NULL, 16));
    } else if (!strcmp(argv[0], "memr") && argc == 3) {
        if (!fm1_host_uboot_ready()) tx_line("ERR no-uboot");
        else do_memr(strtoul(argv[1], NULL, 0), strtoul(argv[2], NULL, 0));
    } else if (!strcmp(argv[0], "jump") && argc == 3) {
        if (!fm1_host_uboot_ready()) tx_line("ERR no-uboot");
        else do_jump(strtoul(argv[1], NULL, 0), strtoul(argv[2], NULL, 0));
    } else {
        tx_line("ERR unknown");
    }
}
