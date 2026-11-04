#!/bin/sh
# Backs up the first SIZE bytes of the ESP32-S3 board's own flash (what it ran before) in 64 KiB pieces, each retried until it reads back
# (the CH340 link here drops bytes), then joins them: backup/esp32s3-original-<size>.bin
#   sh tools/backup_board.sh PORT [SIZE]       (default 0x400000)
set -e
port=$1
size=${2:-0x400000}
esptool=${ESPTOOL:-esptool.py}             # (pip install esptool; or ESPTOOL=path)
here=$(cd "$(dirname "$0")/.." && pwd)
parts=$here/backup/parts
mkdir -p "$parts"
n=$((size / 0x10000))
i=0
while [ $i -lt $n ]; do
    off=$((i * 0x10000))
    f=$(printf '%s/%06x.bin' "$parts" $off)
    try=0
    until [ -s "$f" ] && [ "$(wc -c < "$f")" -eq 65536 ]; do
        try=$((try + 1))
        [ $try -gt 10 ] && { echo "backup: 0x$(printf %x $off) failed 10 times" >&2; exit 1; }
        rm -f "$f"
        "$esptool" --port "$port" --baud 115200 read_flash $off 0x10000 "$f" >/dev/null 2>&1 || true
    done
    i=$((i + 1))
    printf 'backup: %d/%d\r' $i $n
done
out=$(printf '%s/backup/esp32s3-original-%x.bin' "$here" $size)
cat "$parts"/*.bin > "$out"
echo
shasum -a 256 "$out"
