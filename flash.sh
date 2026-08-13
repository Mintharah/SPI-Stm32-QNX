#!/usr/bin/env bash
#
# flash.sh -- flash the STM32F401CC "Black Pill" over USB DFU.
#
#   ./flash.sh              flash firmware/firmware.bin
#   ./flash.sh -b           rebuild with PlatformIO first, then flash
#   ./flash.sh -f other.bin flash a specific binary
#
# The board must be in DFU mode. This script waits for it and tells you how.
#
set -euo pipefail

DFU_ID="0483:df11"          # STM32 BOOTLOADER (ST DfuSe)
FLASH_ADDR="0x08000000"     # start of internal flash on the F401
WAIT_SECS=60

here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BIN="$here/firmware/firmware.bin"
DO_BUILD=0

die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }
note() { printf '\033[36m==>\033[0m %s\n' "$*"; }
ok()   { printf '\033[32m%s\033[0m\n' "$*"; }

usage() {
    sed -n '3,10p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
    exit 0
}

while [ $# -gt 0 ]; do
    case "$1" in
        -b|--build) DO_BUILD=1; shift ;;
        -f|--file)  BIN="${2:-}"; [ -n "$BIN" ] || die "-f needs a path"; shift 2 ;;
        -h|--help)  usage ;;
        *)          die "unknown option '$1' (try --help)" ;;
    esac
done

command -v dfu-util >/dev/null 2>&1 || die "dfu-util not found. sudo apt install dfu-util"

# ---- optional rebuild -------------------------------------------------------
if [ "$DO_BUILD" -eq 1 ]; then
    pio_bin="$(command -v pio || true)"
    # Fall back to the venv this was originally built with, if it is still there.
    if [ -z "$pio_bin" ]; then
        for c in "$HOME"/.platformio/penv/bin/pio /tmp/claude-*/*/*/scratchpad/piovenv/bin/pio; do
            [ -x "$c" ] && { pio_bin="$c"; break; }
        done
    fi
    [ -n "$pio_bin" ] || die "PlatformIO not found; build manually with 'pio run -d Stm32-SPI'"
    note "building with $pio_bin"
    "$pio_bin" run -d "$here/Stm32-SPI"
    built="$here/Stm32-SPI/.pio/build/blackpill_f401cc/firmware.bin"
    [ -f "$built" ] || die "build produced no firmware.bin"
    mkdir -p "$here/firmware"
    cp "$built" "$here/firmware/firmware.bin"
    cp "$here/Stm32-SPI/.pio/build/blackpill_f401cc/firmware.elf" "$here/firmware/" 2>/dev/null || true
    BIN="$here/firmware/firmware.bin"
fi

[ -f "$BIN" ] || die "no firmware at $BIN (run with -b to build it)"
note "firmware: $BIN ($(stat -c%s "$BIN") bytes)"

# ---- sudo, unless a udev rule already grants access -------------------------
# dfu-util needs write access to the USB device. There are st-link rules on this
# machine but none for 0483:df11, so this normally needs root. If you would
# rather not use sudo:
#
#   echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="df11", MODE="0666"' \
#     | sudo tee /etc/udev/rules.d/50-stm32-dfu.rules
#   sudo udevadm control --reload-rules && sudo udevadm trigger
#
SUDO=""
if [ "$(id -u)" -ne 0 ]; then
    if ! dfu-util -l 2>/dev/null | grep -q "$DFU_ID"; then
        if command -v sudo >/dev/null 2>&1; then
            SUDO="sudo"
            # Authenticate ONCE here. The wait loop below polls dfu-util every
            # second, and without a cached credential every one of those would
            # be its own password prompt.
            note "need root for USB access (no udev rule for $DFU_ID)"
            sudo -v || die "sudo authentication failed"
        fi
    fi
fi

# ---- wait for the board in DFU mode ----------------------------------------
in_dfu() { $SUDO dfu-util -l 2>/dev/null | grep -q "$DFU_ID"; }

if ! in_dfu; then
    cat <<'EOT'

  Put the board in DFU mode:

     1. hold BOOT0
     2. tap NRST (keep holding BOOT0)
     3. release BOOT0

  It should enumerate as "STM32  BOOTLOADER". Waiting...

EOT
    for _ in $(seq "$WAIT_SECS"); do
        in_dfu && break
        sleep 1
    done
    in_dfu || die "no DFU device ($DFU_ID) after ${WAIT_SECS}s.
  - check 'lsusb | grep 0483' -- if it shows there but not in dfu-util, it is permissions
  - a USB cable that is charge-only will do exactly this"
fi

ok "DFU device found"
$SUDO dfu-util -l 2>/dev/null | grep "$DFU_ID" | head -2

# ---- flash ------------------------------------------------------------------
# -a 0        alt setting 0 = internal flash
# :leave      jump into the application when the download finishes, so the board
#             runs immediately instead of sitting in the bootloader
note "writing to $FLASH_ADDR"
$SUDO dfu-util -d "$DFU_ID" -a 0 -s "${FLASH_ADDR}:leave" -D "$BIN"

echo
ok "flashed"
cat <<'EOT'

Check the onboard LED (PC13). Count blinks per group:

   5 = PB0 raised, no transfer completes   <- the stuck state this build fixes
   6 = full pipeline working

Then confirm data-ready idles LOW with the Pi quiet. The producer prints:

   [ctrl] GPIO27 idle level low (pull-down active)   <- good
   WARNING: GPIO27 reads HIGH -- check wiring        <- still stuck
EOT
