#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""esp32_recover: MvaveFM1Unbricker's fm1_transporter_recover.py, unchanged, driving the FM-1 Transporter on an
ESP32-S3 board (fm1-esp32-unbrick).

The ESP32-S3 board speaks the Transporter's line protocol on its USB-UART port instead of a USB CDC port, so only
how the port is found and opened differs; every check, dump, comparison and write is the original script's:
  - the port: the board's USB-UART bridge (/dev/cu.usbserial-*, wchusbserial*, SLAB_USBtoUART, usbmodem*),
    or --port;
  - 115200 baud (the firmware's UART0);
  - DTR and RTS held off from the moment the port opens: on an ESP32 board they drive EN and GPIO0, and a
    reset in the middle of a session would drop the FM-1 out of UBOOT;
  - a link that loses bytes (a CH340 under WCH's macOS driver 1.0 does, at every rate): reads go as 4 KiB
    requests, each checked by the firmware's CRC-32 and retried on a mismatch (a read changes nothing; the
    script still reads the whole flash twice and compares); a sector write is retried only on `ERR crc`, the
    one reply that means the firmware refused the payload before erasing anything.
Use its commands as documented (info, dump, restore, recover), e.g.
  esp32_recover.py info
  esp32_recover.py dump backups
  esp32_recover.py recover FIRMWARE.fwsc --out backups
Not `wizard` or `flash-xiao` (they are for the XIAO). `esp32_recover.py log` prints the board's own log."""
import glob
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
UNBRICKER = Path(os.environ.get("FM1_UNBRICKER", HERE.parent / "third_party" / "MvaveFM1Unbricker"))   # tools/fetch_deps.sh
sys.path.insert(0, str(UNBRICKER))

import fm1_transporter_recover as ftr  # noqa: E402

BAUD = 115200
CHUNK = 4096
READ_TRIES = 8
WSEC_TRIES = 5
PATTERNS = ("/dev/cu.usbserial-*", "/dev/cu.wchusbserial*", "/dev/cu.SLAB_USBtoUART*", "/dev/cu.usbmodem*",
            "/dev/ttyUSB*", "/dev/ttyACM*")


def candidate_ports():
    out = []
    for pat in PATTERNS:
        out += sorted(glob.glob(pat))
    return out


def open_serial(path, timeout=0.3):
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = path, BAUD, timeout
    s.dtr = False       # set before open: pyserial applies them as the port opens
    s.rts = False
    s.open()
    return s


def open_port(path):
    try:
        s = open_serial(path)
    except Exception as e:
        ftr.log.debug("cannot open %s: %s", path, e)
        return None
    return ftr.Transporter(s, path)


def show_log(argv):
    port = argv[argv.index("--port") + 1] if "--port" in argv else None
    ports = [port] if port else candidate_ports()
    for p in ports:
        try:
            s = open_serial(p, 1.0)
        except Exception:
            continue
        s.reset_input_buffer()
        s.write(b"log\n")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            line = s.readline().decode("ascii", "replace").strip()
            if line.startswith("LOG "):
                n = int(line.split()[1])
                data = b""
                while len(data) < n:
                    chunk = s.read(n - len(data))
                    if not chunk:
                        break
                    data += chunk
                sys.stdout.write(data.decode("ascii", "replace"))
                print("[%s: %d bytes of log]" % (p, n))
                return 0
        s.close()
    print("no fm1-esp32-unbrick board answered `log` on: %s" % (", ".join(ports) or "no serial ports"))
    return 1


def resync(t):
    """After a broken transfer: let the rest of it arrive, then throw it away."""
    time.sleep(0.5)
    t.s.reset_input_buffer()


def read_chunk(t, addr, n):
    """One `read` of n bytes (the original Transporter.read_flash's parsing, without its progress bar)."""
    reply = t.request("read %#x %d" % (addr, n), 30.0)
    if reply != "DATA %d" % n:
        raise ftr.TransporterError("read 0x%X+0x%X: %s" % (addr, n, reply))
    data = bytearray()
    t.s.timeout = 3
    while len(data) < n:
        chunk = t.s.read(n - len(data))
        if not chunk:
            raise ftr.TransporterError("read 0x%X+0x%X stalled at %d bytes" % (addr, n, len(data)))
        data += chunk
    end = t._readline(3)
    want = "END %08X" % ftr.crc32(data)
    if end != want:
        raise ftr.TransporterError("read 0x%X+0x%X: transfer check failed (%s, local %s)"
                                   % (addr, n, end or "no END line", want))
    return bytes(data)


def read_flash(self, addr, length, label="Reading"):
    if addr < 0 or length <= 0 or addr + length > ftr.FLASH_SIZE:
        raise ftr.fu.SafetyError("read 0x%X+0x%X is outside the flash" % (addr, length))
    data = bytearray()
    prog = ftr.fu.Progress(label, length)
    retries = 0
    try:
        while len(data) < length:
            a, n = addr + len(data), min(CHUNK, length - len(data))
            for attempt in range(READ_TRIES):
                try:
                    data += read_chunk(self, a, n)
                    break
                except ftr.TransporterError as e:
                    retries += 1
                    ftr.log.debug("retry %d of 0x%X: %s", attempt + 1, a, e)
                    resync(self)
            else:
                raise ftr.TransporterError("read 0x%X failed %d times: the serial link is losing too many "
                                           "bytes" % (a, READ_TRIES))
            prog.update(n)
    finally:
        prog.close()
    if retries:
        ftr.log.info("  (%d chunk%s re-read after a transfer check failed)", retries, "s" * (retries != 1))
    return bytes(data)


def write_sector(self, addr, sec):
    for attempt in range(WSEC_TRIES):
        reply = _write_sector(self, addr, sec)
        if not reply.startswith("ERR crc"):
            return reply
        ftr.log.info("  sector 0x%05X: %s (the payload arrived damaged, nothing was written): resending",
                     addr, reply)
        resync(self)
    return reply


_read_flash, _write_sector = ftr.Transporter.read_flash, ftr.Transporter.write_sector
ftr.Transporter.read_flash = read_flash
ftr.Transporter.write_sector = write_sector
ftr.candidate_ports = candidate_ports
ftr.open_port = open_port

if __name__ == "__main__":
    argv = sys.argv[1:]
    if argv[:1] == ["log"]:
        sys.exit(show_log(argv))
    if argv[:1] and argv[0] in ("wizard", "flash-xiao", "setup"):
        sys.exit("esp32_recover: `%s` is for the XIAO RP2040 Transporter; use info, dump, restore or recover" % argv[0])
    sys.exit(ftr.main(argv))
