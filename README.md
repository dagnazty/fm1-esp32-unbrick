# fm1-esp32-unbrick

**Unbrick an M-VAVE FM-1 with an ESP32-S3 dev board.** No RP2040, no soldering, no Linux or Windows: an ESP32-S3 board with two USB-C ports, two cables and a Mac (or any computer with Python).

By [dag](https://dagnazty.dev/). It's a port of Leo Kuroshita's [FM-1 Transporter](https://github.com/kurogedelic/FM-1-transporter) from the RP2040 (XIAO) to the **ESP32-S3**, driven by Michael P Jones's recovery script from [MvaveFM1Unbricker](https://github.com/Quixotic7/MvaveFM1Unbricker).

## When you need it

Your FM-1 is soft-bricked: a firmware install was cut off (a hub, a cable, the battery), and now it shows **no USB device in any mode**. Only the charging LED lights, the OCT− rescue combination does nothing, and the usual installers can't see it. The flash is half old firmware, half new, and the app crashes before it starts USB.

The FM-1's chip (JieLi AC791N / WL82) has a mask-ROM USB boot mode that ignores the flash entirely. An ESP32-S3 can get it into that mode; then the flash can be read, checked and repaired.

It brought a real bricked FM-1 back on 2026-11-01: an install had stopped at 63%, leaving 46 of 143 sectors on the old firmware. The script wrote just those 46 sectors back and the FM-1 booted.

## How the ESP32-S3 does it

The ROM's boot mode (`WL82 UBOOT1.00`, USB `4C4A:8057`) can't be reached by a normal USB host. It needs a 16-bit **USB_KEY** (`0x16EF`) bit-banged on D+/D−, then clock pulses while the ROM trims its oscillator; only then can a USB host enumerate it. The ESP32-S3 does all of this on its **native USB pins** (GPIO19 = D−, GPIO20 = D+), then hands the same pins to its built-in **USB OTG host**:

```
computer ──USB-C──► board's UART port (USB-UART bridge, UART0 115200) ──► ESP32-S3
                                                                          │ GPIO19 D−, GPIO20 D+
                                                                          ▼
                                            the board's native USB port ──USB-C──► FM-1
```

| Step | On the RP2040 Transporter | On the ESP32-S3 ([main/](main)) |
|---|---|---|
| USB_KEY 0x16EF, 20 µs a bit (D+ clock, D− data) | PIO at 1 MHz | timed GPIO register writes, interrupts off, on core 1 ([recovery.c](main/recovery.c)) |
| the ROM's ACK, between packets | PIO | `gap_sense()`: D− driven high for 2 µs and read back |
| a 4 µs D+ low pulse every 1 ms until the ROM releases D+ (about 1.5 s) | PIO | `pulses()` |
| USB host, mass storage (Bulk-Only): CBW/CSW, INQUIRY | Pico-PIO-USB | the ESP32-S3's internal OTG PHY + ESP-IDF `usb_host` ([host.c](main/host.c)) |
| JieLi UBOOT commands, the flash loader upload | `jieli_uboot.c` | the same code, the loader embedded in the firmware ([jieli_uboot.c](main/jieli_uboot.c)) |
| the line protocol to the computer | USB CDC | UART0, through the board's USB-UART bridge ([proto.c](main/proto.c)) |

GPIO19/20 have one owner at a time: `recovery.c` bit-bangs them, and the moment the ROM releases D+ they go to the OTG host (about 31 ms on the real run).

### What the ESP32-S3 port had to fix

1. **USB-Serial-JTAG holds the pins.** After any reset that isn't a power-on, the ESP32-S3's USB-Serial-JTAG block can keep GPIO19/20 and its 1.5 kΩ D+ pull-up. `usj_release_pads()` turns its pull-up and pads off before the pins are used. Look for `bus idle: D+=0 D-=0` in the log.
2. **Weak drive.** Drive strength 0 (about 5 mA) and weak pull-downs, so the FM-1 can hold a line low to ACK, and its pull-up wins when it attaches.
3. **Nothing else on UART0.** [sdkconfig.defaults](sdkconfig.defaults) turns the console, USB-Serial-JTAG and the task watchdog off and sets FreeRTOS to 1000 Hz, so UART0 carries only the protocol. The board's own messages go to a ring buffer, read with `log`.
4. **DTR/RTS must never move.** On ESP32 boards they drive EN and GPIO0: a toggle resets the ESP32-S3 mid-session and drops the FM-1 out of its boot mode. [esp32_recover.py](tools/esp32_recover.py) holds both off before it opens the port.
5. **A lossy USB-UART link.** A CH340 under WCH's macOS driver loses bytes at every baud rate. So flash reads go as 4 KiB requests, each checked against the firmware's CRC-32 and retried up to 8 times; a sector write is retried only on `ERR crc`, the one reply that means nothing was erased yet; a payload that stalls for 3 s gets `ERR crc short`. On the real run 8 of about 1290 reads needed a retry and every write went through first time.

## What you need

- **An ESP32-S3 dev board with two USB-C ports**: one on a USB-UART bridge (CH340 or CP210x, often labelled *UART* or *COM*), one on the ESP32-S3's native USB (GPIO19/20, labelled *USB* or *OTG*). Most "ESP32-S3-DevKitC-1"-style boards have both.
- Two USB-C data cables. Plug the board straight into the computer, no hub.
- The FM-1, **charged**: it runs on its battery for the whole recovery.
- **ESP-IDF v5.4.1** ([install guide](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/get-started/)).
- **Python 3** with `pyserial` (`pip3 install pyserial`).
- The FM-1 firmware package (`.fwsc`) you want back: the one whose install was interrupted, or any you trust.

ESP-IDF can't build from a path with a space in it: clone this repository somewhere without one.

## Step by step

### 0. Get the parts this repository doesn't ship

JieLi's flash loader (`wl82loader.bin`) isn't ours to redistribute, and the recovery script lives in MvaveFM1Unbricker. This fetches both, at the commits tested here, and checks the loader's SHA-256:

```bash
sh tools/fetch_deps.sh
```

### 1. Back up the ESP32-S3 board (optional)

To put back what the board ran before (an Arduino sketch, a demo), save its flash first, in 64 KiB pieces retried over a lossy link:

```bash
sh tools/backup_board.sh /dev/cu.wchusbserial1110
```

It writes `backup/esp32s3-original-400000.bin` (4 MB). Use your board's port: `ls /dev/cu.*` on a Mac, `/dev/ttyUSB0` on Linux.

### 2. Build and flash the ESP32-S3

From an ESP-IDF shell (`. ~/esp/esp-idf/export.sh`):

```bash
idf.py build
```

```bash
idf.py -p /dev/cu.wchusbserial1110 -b 115200 flash
```

Keep `-b 115200` with a CH340 on macOS; faster fails. The build checks `wl82loader.bin` against its SHA-256 and refuses a wrong file.

Check the board answers. Its log should start with `bus idle: D+=0 D-=0`:

```bash
python3 tools/esp32_recover.py log --port /dev/cu.wchusbserial1110
```

### 3. Connect the FM-1 and get it into its boot mode

1. The FM-1 is **off**. Connect it to the board's **native USB** port.
2. Start `info`; it waits up to 180 s:

   ```bash
   python3 tools/esp32_recover.py info --port /dev/cu.wchusbserial1110 --wait 180
   ```

3. **Switch the FM-1 on** while it waits. The ESP32-S3 has been sending the key all along; the ROM answers at power-on.

Success:

```
Status      uboot=1 v15=0 loader=1 loader_running=0
Info        OK key=980F type=3 id=856014
Chip key 980F and flash 856014 (1 MiB): this is an FM-1 in UBOOT
```

The board's own log of the real handshake (`esp32_recover.py log`):

```
49.520 ACK (data line held low) after 98017 packets
49.521 PULSE: D+ high (ROM pull-up), sending 4 us pulses every 1 ms
51.021 PULSE: D+ released after 1499.9 ms - calibration done, host takes over
51.052 HANDOFF: pulses stopped, OTG host started in 30660 us
51.621 HOST: device mounted, addr=1 (568.9 ms after host start)
51.622   VID=4C4A PID=8057 bcdDevice=0300 bcdUSB=0110
53.123 INQUIRY OK: vendor="WL82    " product="UBOOT1.00       " rev="1.00"
53.152 JL: uploading wl82loader (24064 bytes) to 01C02000
53.244 JL: loader running
53.250 PROTO: chip key 980F, device type 3, flash id 856014
```

A "false ACK" with the FM-1 still off is normal; the board goes back to keying by itself.

### 4. Dump the flash and read the analysis

```bash
python3 tools/esp32_recover.py dump backups --package FIRMWARE.fwsc --port /dev/cu.wchusbserial1110 --wait 30
```

The whole 1 MiB is read twice and both reads must match (about 1 m 50 s each at 115200). The dump and its analysis go to `backups/`. On the real run it showed exactly what the interrupted install left:

```
0x04000..0x5DFFF   90 sectors  new firmware
0x5E000..0x8BFFF   46 sectors  other            ← the interrupted install
0x8C000..0x92FFF    7 sectors  new firmware
no torn sector, loader intact, update record valid
```

**Keep this dump.** It's the reference for the next step and your way back.

### 5. Restore: a dry run, then the write

The dry run lists what it would write and writes nothing:

```bash
python3 tools/esp32_recover.py restore FIRMWARE.fwsc --ref backups/DUMP.bin --port /dev/cu.wchusbserial1110 --wait 30
```

Then the write:

```bash
python3 tools/esp32_recover.py restore FIRMWARE.fwsc --ref backups/DUMP.bin --write --yes --port /dev/cu.wchusbserial1110 --wait 30
```

- **Never written:** the head (0x0000–0x3FFF, the boot code), and the device data above 0x93000 (presets, settings, samples).
- **Before writing:** a fresh full read, which must still match `--ref`.
- **Only the sectors that differ are written;** on the real run, 46 sectors in 20 s.
- **After writing:** everything is read back and compared with the expected image (`Final full read EQUALS the expected image`).

### 6. Boot

Unplug the FM-1 from the board and switch it off and on. It starts the restored firmware; from there, install whatever you like with the normal installer.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `log` shows `bus idle: D+=1` with nothing attached | USB-Serial-JTAG kept the pins: power-cycle the board (unplug both cables) |
| "D- pulled up instead of D+ - are D+/D- swapped?" | The cable or an adapter swaps the pair: try another cable |
| No ACK after the FM-1 is switched on | Off, wait 5 s, on again; or send `rekey`; make sure the FM-1 is charged |
| `read … failed 8 times: the serial link is losing too many bytes` | Board straight into the computer, no hub; stay at 115200 |
| The FM-1 drops out of its boot mode mid-session | Something toggled DTR/RTS (a serial monitor, another script): close it and use only `esp32_recover.py` |
| `ERR no-uboot` | The FM-1 isn't in its boot mode yet: run `info --wait 180` and switch it on |

`esp32_recover.py` passes everything to MvaveFM1Unbricker's `fm1_transporter_recover.py`: `info`, `dump`, `restore`, `recover`, and `--dry-run` against a mock. It refuses `wizard`, `flash-xiao` and `setup`, which are for the XIAO RP2040.

## Putting the board back

```bash
esptool.py --port /dev/cu.wchusbserial1110 --baud 115200 write_flash 0 backup/esp32s3-original-400000.bin
```

Or leave it flashed: it does nothing until it sees an FM-1.

## Files

| Path | What |
|---|---|
| [main/main.c](main/main.c) | the worker on core 1: the key or the pulses, the handoff, then the host and the protocol loop |
| [main/recovery.c](main/recovery.c) | USB_KEY, ACK probes, the ROM's pulses, releasing the USB-Serial-JTAG pins |
| [main/host.c](main/host.c) | ESP-IDF `usb_host` client, Bulk-Only mass storage, INQUIRY retries |
| [main/jieli_uboot.c](main/jieli_uboot.c) | JieLi UBOOT commands and the loader (from the RP2040 Transporter) |
| [main/proto.c](main/proto.c) | the Transporter's line protocol on UART0, plus `log` |
| [tools/esp32_recover.py](tools/esp32_recover.py) | MvaveFM1Unbricker's recovery script on this board's serial port |
| [tools/fetch_deps.sh](tools/fetch_deps.sh) | fetches the loader and the recovery script into `third_party/` |
| [tools/backup_board.sh](tools/backup_board.sh) | a retrying backup of the ESP32-S3's own flash |

Your dumps (`backups/`), the board's backup (`backup/`), logs (`logs/`) and firmware packages stay out of git (see [.gitignore](.gitignore)): they hold your device's data.

## Credits

- **[FM-1 Transporter](https://github.com/kurogedelic/FM-1-transporter)**, Leo Kuroshita / Hügelton Instruments (MIT): the USB_KEY timings, the pulse protocol, the JieLi UBOOT code and the line protocol this ports.
- **[MvaveFM1Unbricker](https://github.com/Quixotic7/MvaveFM1Unbricker)**, Michael P Jones (MIT): the recovery script, its flash analysis and safety checks.
- **[jl-uboot-tool](https://github.com/kagaimiq/jl-uboot-tool)**, Andrey Grigoryev (kagaimiq) (MIT): the JieLi UBOOT protocol, and where the flash loader comes from.

## Licence

MIT (see [LICENSE](LICENSE)), as the projects it builds on. No warranty: you're writing to your device's flash at your own risk. M-VAVE and FM-1 are trademarks of their owners; this project isn't affiliated with M-VAVE or JieLi.
