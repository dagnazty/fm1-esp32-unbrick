#!/bin/sh
# SPDX-License-Identifier: MIT
# Fetches what this repository doesn't ship, at the commits it was tested with, into third_party/:
#   - kagaimiq/jl-uboot-tool: JieLi's WL82 flash loader, data/loaderblobs/usb/wl82loader.bin, copied to
#     main/wl82loader.bin (the ESP32-S3 firmware embeds it; main/CMakeLists.txt checks its SHA-256 again)
#   - Quixotic7/MvaveFM1Unbricker: fm1_transporter_recover.py, which tools/esp32_recover.py drives
#   sh tools/fetch_deps.sh
set -e
here=$(cd "$(dirname "$0")/.." && pwd)
tp=$here/third_party
mkdir -p "$tp"

fetch() {   # fetch NAME URL COMMIT
    if [ ! -d "$tp/$1/.git" ]; then
        git clone -q "$2" "$tp/$1"
    fi
    git -C "$tp/$1" fetch -q origin "$3" 2>/dev/null || git -C "$tp/$1" fetch -q origin
    git -C "$tp/$1" checkout -q "$3"
    echo "fetch_deps: $1 at $(git -C "$tp/$1" rev-parse --short HEAD)"
}

fetch jl-uboot-tool https://github.com/kagaimiq/jl-uboot-tool.git adb3f18889e88ac512ce0a3c4d8cc3d3cb30696a
fetch MvaveFM1Unbricker https://github.com/Quixotic7/MvaveFM1Unbricker.git 517b9387e62e908115e5eec66c32d0ac3289f039

want=d41da6126760c9d66660bcc0cac8d27d221806c5e369a8036921efe68dca5376
src=$tp/jl-uboot-tool/data/loaderblobs/usb/wl82loader.bin
got=$(shasum -a 256 "$src" | cut -d' ' -f1)
if [ "$got" != "$want" ]; then
    echo "fetch_deps: $src has sha256 $got, expected $want" >&2
    exit 1
fi
cp "$src" "$here/main/wl82loader.bin"
echo "fetch_deps: main/wl82loader.bin ($want)"
