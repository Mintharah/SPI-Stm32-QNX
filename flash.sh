#!/usr/bin/env bash
#
# flash.sh -- flash the STM32F401CC "Black Pill".
#
#   ./flash.sh              DFU over USB (default)
#   ./flash.sh --swd        SWD via ST-Link + openocd
#   ./flash.sh --hse-check  is the external crystal alive? (needs ST-Link)
#   ./flash.sh -b           rebuild with PlatformIO first, then flash
#   ./flash.sh -f other.bin flash a specific binary
#
# Why two methods: this board's application runs entirely on HSI (see
# system_clock_config.c -- RCC_OSCILLATORTYPE_HSI, PLLSOURCE_HSI), but the
# STM32F401's ROM DFU bootloader clocks USB from HSE, the external crystal.
# A dead or badly soldered crystal is therefore invisible in normal operation
# and breaks DFU only, showing up as:
#
#     usb 1-3: device not accepting address NN, error -71
#
# SWD does not care about any of that. If DFU fails that way, use --swd, and
# --hse-check to confirm the cause.
#
set -euo pipefail

DFU_ID="0483:df11"          # STM32 BOOTLOADER (ST DfuSe)
FLASH_ADDR="0x08000000"     # start of internal flash on the F401
WAIT_SECS=60

here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BIN="$here/firmware/firmware.bin"
MODE="dfu"
DO_BUILD=0

die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }
note() { printf '\033[36m==>\033[0m %s\n' "$*"; }
ok()   { printf '\033[32m%s\033[0m\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*"; }

usage() { sed -n '3,9p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0; }

while [ $# -gt 0 ]; do
    case "$1" in
        --swd)       MODE="swd"; shift ;;
        --dfu)       MODE="dfu"; shift ;;
        --hse-check) MODE="hse"; shift ;;
        -b|--build)  DO_BUILD=1; shift ;;
        -f|--file)   BIN="${2:-}"; [ -n "$BIN" ] || die "-f needs a path"; shift 2 ;;
        -h|--help)   usage ;;
        *)           die "unknown option '$1' (try --help)" ;;
    esac
done

# ---- optional rebuild -------------------------------------------------------
if [ "$DO_BUILD" -eq 1 ]; then
    pio_bin="$(command -v pio || true)"
    if [ -z "$pio_bin" ]; then
        for c in "$HOME"/.platformio/penv/bin/pio /tmp/claude-*/*/*/scratchpad/piovenv/bin/pio; do
            [ -x "$c" ] && { pio_bin="$c"; break; }
        done
    fi
    [ -n "$pio_bin" ] || die "PlatformIO not found; build manually with 'pio run -d Stm32-SPI'"
    note "building with $pio_bin"
    "$pio_bin" run -d "$here/Stm32-SPI"
    out="$here/Stm32-SPI/.pio/build/blackpill_f401cc"
    [ -f "$out/firmware.bin" ] || die "build produced no firmware.bin"
    mkdir -p "$here/firmware"
    cp "$out/firmware.bin" "$out/firmware.elf" "$here/firmware/" 2>/dev/null || \
        cp "$out/firmware.bin" "$here/firmware/"
    BIN="$here/firmware/firmware.bin"
fi

post_flash_hint() {
    cat <<'EOT'

Check the onboard LED (PC13). Count blinks per group:

   5 = PB0 raised, no transfer completes   <- the stuck state this build fixes
   6 = full pipeline working

Then confirm data-ready idles LOW with the Pi quiet. The producer prints:

   [ctrl] GPIO27 idle level low (pull-down active)   <- good
   WARNING: GPIO27 reads HIGH -- check wiring        <- still stuck
EOT
}

# =============================== SWD / openocd ===============================
openocd_run() {
    command -v openocd >/dev/null 2>&1 || die "openocd not found. sudo apt install openocd"
    # interface/stlink.cfg covers v2, v2-1 and v3.
    local sudo=""
    lsusb 2>/dev/null | grep -qiE "0483:(3748|374b|3752|374f|3753)" || \
        warn "no ST-Link seen on USB -- openocd will fail if one is not attached"
    openocd -f interface/stlink.cfg -f target/stm32f4x.cfg "$@" 2>&1 || {
        [ -n "$sudo" ] || warn "if that was a permission error, retry with sudo"
        return 1
    }
}

if [ "$MODE" = "swd" ]; then
    [ -f "$BIN" ] || die "no firmware at $BIN (run with -b to build it)"
    note "SWD flash: $BIN ($(stat -c%s "$BIN") bytes) -> $FLASH_ADDR"
    openocd_run -c "program \"$BIN\" verify reset exit $FLASH_ADDR"
    echo; ok "flashed via SWD"; post_flash_hint
    exit 0
fi

if [ "$MODE" = "hse" ]; then
    # Turn HSE on and read RCC_CR back. Bit 17 (HSERDY, 0x00020000) sets only if
    # the external crystal actually oscillates. The application never enables
    # HSE, so this is the only thing that will tell you.
    note "checking HSE via SWD (RCC_CR @ 0x40023800, HSERDY = bit 17)"
    out="$(openocd_run \
        -c "init" -c "reset halt" \
        -c "mww 0x40023800 0x00010001" \
        -c "sleep 200" \
        -c "mdw 0x40023800" \
        -c "reset run" -c "exit" || true)"
    echo "$out" | grep -iE "0x40023800|Error|Info : target" || echo "$out"
    val="$(echo "$out" | grep -oE '0x40023800: [0-9a-f]{8}' | awk '{print $2}' | tail -1)"
    if [ -z "$val" ]; then
        warn "could not read RCC_CR -- is an ST-Link attached and SWD wired (SWDIO/SWCLK/GND)?"
        exit 1
    fi
    echo
    note "RCC_CR = 0x$val"
    if [ $(( 0x$val & 0x20000 )) -ne 0 ]; then
        ok "HSERDY set -- the crystal IS oscillating, so DFU's USB clock is not the problem"
        echo "  Look instead at: the USB cable (charge-only cables do exactly this),"
        echo "  the R10 D+ pull-up (WeAct boards ship 10k where 1.5k is wanted),"
        echo "  or plugging into a USB 2.0 port / through a 2.0 hub instead of xHCI."
    else
        warn "HSERDY CLEAR -- the external crystal is not oscillating."
        echo "  That explains DFU failing with 'error -71': the ROM bootloader clocks"
        echo "  USB from HSE. The application is unaffected because it runs on HSI."
        echo "  Use --swd to flash; fix or reflow the crystal if you want DFU back."
    fi
    exit 0
fi

# =================================== DFU =====================================
command -v dfu-util >/dev/null 2>&1 || die "dfu-util not found. sudo apt install dfu-util"
[ -f "$BIN" ] || die "no firmware at $BIN (run with -b to build it)"
note "firmware: $BIN ($(stat -c%s "$BIN") bytes)"

# dfu-util needs write access to the USB device. There is no udev rule for
# 0483:df11 on this machine, so this normally needs root. To avoid sudo:
#
#   echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="df11", MODE="0666"' \
#     | sudo tee /etc/udev/rules.d/50-stm32-dfu.rules
#   sudo udevadm control --reload-rules && sudo udevadm trigger
#
SUDO=""
if [ "$(id -u)" -ne 0 ] && ! dfu-util -l 2>/dev/null | grep -q "$DFU_ID"; then
    if command -v sudo >/dev/null 2>&1; then
        SUDO="sudo"
        # Authenticate ONCE. The wait loop polls every second, and without a
        # cached credential each poll would be its own password prompt.
        note "need root for USB access (no udev rule for $DFU_ID)"
        sudo -v || die "sudo authentication failed"
    fi
fi

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
  If dmesg shows 'device not accepting address ... error -71', the board is
  enumerating and failing, not absent. On this firmware that usually means the
  HSE crystal is dead -- the ROM bootloader needs it for USB, the application
  does not. Confirm with:  ./flash.sh --hse-check
  and flash over SWD meanwhile:  ./flash.sh --swd"
fi

ok "DFU device found"
$SUDO dfu-util -l 2>/dev/null | grep "$DFU_ID" | head -2

# -a 0    alt setting 0 = internal flash
# :leave  jump into the application once the download finishes, instead of
#         sitting in the bootloader (which looks like a failed flash)
note "writing to $FLASH_ADDR"
$SUDO dfu-util -d "$DFU_ID" -a 0 -s "${FLASH_ADDR}:leave" -D "$BIN"

echo; ok "flashed"; post_flash_hint
