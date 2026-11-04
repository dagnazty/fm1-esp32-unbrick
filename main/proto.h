#pragma once

// The Mac-facing data channel: FM-1 Transporter's line protocol (src/transporter_proto.h, tools/fm1t.py,
// MvaveFM1Unbricker fm1_transporter_recover.py), on UART0 (the board's USB-UART port) instead of a USB CDC port.
//   ping, status, uboot, rekey, info, read, wsec, ldr, memw, memr, jump, runapp: as the Transporter
//   log -> LOG <n>\n <n bytes of the board's log> END\n   (this board's own, for debugging)
// One difference, deliberate: `status` is held back until the FM-1 is in UBOOT (or 20 s after the host
// started), because the recovery script takes the first status reply as final.

void proto_start(void);     // UART0 and its receive task (core 0)
void proto_poll(void);      // worker (core 1): one pending request, else the loader keep-alive
