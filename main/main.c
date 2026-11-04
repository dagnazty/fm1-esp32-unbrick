// fm1-esp32-unbrick - FM-1 Transporter's recovery on an ESP32-S3 board with two USB-C ports.
//
//   Mac ── board's USB-UART port (UART0, 115200) ──► ESP32-S3 ── its own USB port (GPIO19 D-, GPIO20 D+) ──► FM-1
//
// core 0: the protocol's receive task and the usb_host library's tasks
// core 1: the worker: USB_KEY and pulses (recovery.c), then the host (host.c) and the requests (proto.c)
// GPIO19/20 have one owner at a time: recovery hands them to the OTG host in one call sequence, with the ROM
// pulses running until the moment the host takes them.

#include "compat.h"
#include "host.h"
#include "jieli_uboot.h"
#include "log.h"
#include "proto.h"
#include "recovery.h"

#define ATTACHED_PULSE_MS 6000
#define REKEY_MAGIC 0x4B455931      // "KEY1": force the USB_KEY

extern uint32_t fm1_rekey_magic;

static void worker(void *arg) {
    (void)arg;
    dlog("fm1-esp32-unbrick: D+=GPIO20 D-=GPIO19 (the S3's own USB port), data on UART0");
    dlog("loader %s", jl_loader_builtin() ? "built in (24064 bytes)" : "MISSING: size mismatch");
    recovery_init();

    bool force_key = fm1_rekey_magic == REKEY_MAGIC;
    fm1_rekey_magic = 0;
    if (force_key) {
        dlog("REKEY: waiting for D+ to drop (loader watchdog / power-off)");
        bool dropped = recovery_wait_detached(15000);
        dlog("REKEY: %s; USB_KEY forced now", dropped ? "D+ dropped" : "D+ still high after 15 s - power-cycle the FM-1");
        recovery_run();
    } else if (recovery_target_attached()) {
        dlog("D+ already pulled up (FM-1 app or a waiting UBOOT) - skipping USB_KEY");
        recovery_pulses_only(ATTACHED_PULSE_MS);
    } else {
        recovery_run();
    }

    uint64_t t_release = time_us_64();
    recovery_release_bus();
    fm1_host_start();
    dlog("HANDOFF: pulses stopped, OTG host started in %llu us", (unsigned long long)(time_us_64() - t_release));

    for (;;) {
        fm1_host_task();
        proto_poll();
        vTaskDelay(1);
    }
}

void app_main(void) {
    proto_start();
    xTaskCreatePinnedToCore(worker, "worker", 8192, NULL, 3, NULL, 1);
}
