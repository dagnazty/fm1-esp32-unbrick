#pragma once

#include <stdbool.h>
#include <stdint.h>

// JieLi USB_KEY entry and the ROM's 1 ms pulses on GPIO20 (D+) / GPIO19 (D-), bit-banged on core 1.
// Ported from FM-1 Transporter src/recovery.c (MIT, Leo Kuroshita): the same packet, gap probes, ACK rule,
// pulse width and hand-off conditions, with the RP2040's PIO replaced by timed register writes.

void recovery_init(void);
void recovery_run(void);                    // key until the ACK, then pulse until the host should take over
void recovery_pulses_only(uint32_t ms);     // a target that already holds D+ up
bool recovery_target_attached(void);
bool recovery_wait_detached(uint32_t timeout_ms);
void recovery_release_bus(void);            // both pins inputs, no pulls: ready for the USB PHY
