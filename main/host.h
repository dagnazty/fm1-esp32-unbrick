#pragma once

#include <stdbool.h>
#include <stdint.h>

// The FM-1-facing USB host: the ESP32-S3's OTG controller on its internal PHY (GPIO19/20), through ESP-IDF's
// usb_host library. Replaces FM-1 Transporter src/pio_host.c (Pico-PIO-USB): enumerate the JieLi ROM UBOOT,
// find its MSC bulk pair, GET MAX LUN, then INQUIRY until it answers; then Bulk-Only commands for jieli_uboot.c.

void fm1_host_start(void);              // takes GPIO19/20: call right after recovery_release_bus()
void fm1_host_task(void);               // mount / unmount / probe work, from the worker task
bool fm1_host_uboot_ready(void);        // mounted and answered INQUIRY as the UBOOT
bool fm1_host_v15_mounted(void);        // stock V15 application mounted (no soft key here)
bool fm1_host_softkey(void);            // not supported on this board: false
uint64_t fm1_host_started_us(void);     // 0 before fm1_host_start

// One Bulk-Only Transport command to the mounted UBOOT. len == 0: no data stage.
bool fm1_host_bot(const uint8_t *cdb, uint8_t cdb_len, bool in, void *buf, uint16_t len, uint32_t *got);
