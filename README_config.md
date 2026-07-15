# Config integration -- file placement, wiring, and build

## Wiring

STM32F401CC Black Pill as slave, Raspberry Pi 5 (QNX) as master. All logic
3.3V; the STM is not 5V tolerant on ADC pins.

### Pi <-> STM (SPI + data-ready)

Pi 5 uses `/dev/spi0` (`spi-dwc` driver). Data-ready is a GPIO pulse from
the STM into a Pi GPIO input.

| signal    | Pi pin        | STM pin | notes                          |
|-----------|---------------|---------|--------------------------------|
| SPI2 NSS  | CE0           | PB12    | slave-select                   |
| SPI2 SCK  | SCLK          | PB13    | mode 0 (CPOL=0 CPHA=0)         |
| SPI2 MISO | MISO          | PB14    | STM -> Pi                      |
| SPI2 MOSI | MOSI          | PB15    | Pi -> STM                      |
| data-ready| GPIO17        | PB0     | rising edge = frame ready      |
| ground    | GND           | GND     | common                         |

Keep wires short (<10 cm). 4 MHz is the empirically reliable clock rate on
bench-jumper wiring; going higher gives drops/CRC errors.

### MPU6050 -> STM (I2C1)

Standard breakout module; almost all have 4.7k pull-ups on SCL/SDA built in.
If yours doesn't, add external pull-ups to 3V3.

| MPU6050 pin | STM pin  | notes                          |
|-------------|----------|--------------------------------|
| VCC         | 3V3      | do NOT use 5V                  |
| GND         | GND      |                                |
| SCL         | PB6      | I2C1 SCL (open-drain)          |
| SDA         | PB7      | I2C1 SDA (open-drain)          |
| AD0         | GND      | sets address to 0x68           |
| INT, XDA, XCL | NC     | unused                         |

If AD0 floats or is tied high, the address becomes 0x69 and firmware
initialization will silently skip the MPU (vib fields stay 0).

### Current sense -> STM (ADC1, 3-phase)

Analog inputs 0-3.3V single-ended. Use a current-sense amp (INA199 or
similar) between the shunt and the STM pin -- do NOT feed raw shunt voltage
in. Any input outside 0..3V3 damages the pin.

| phase | STM pin | ADC channel |
|-------|---------|-------------|
| A     | PA0     | ADC1_IN0    |
| B     | PA1     | ADC1_IN1    |
| C     | PA2     | ADC1_IN2    |

Leaving these pins floating is fine for bench-testing the wire itself --
they'll read ~1000-2000 counts of noise, and the pipeline still works.

### RPM tach -> STM

TTL square wave, rising-edge active. Typically one pulse per revolution;
if your tach outputs N pulses per rev, the reported RPM will be N times
actual.

| signal | STM pin | timer channel  |
|--------|---------|----------------|
| tach   | PB8     | TIM4 CH3       |

Timer counts at 1 MHz, 16-bit; minimum reportable RPM is ~915 (period
65 ms). Without hardware attached, PB8 has a pulldown and the RPM field
stays 0.

### Onboard LED (diagnostic)

Active-low on PC13 (Black Pill onboard LED). Blink groups indicate how
far the pipeline got:

| count | meaning                                                        |
|-------|----------------------------------------------------------------|
|   1   | main() runs but ADC DMA callbacks never fire                   |
|   2   | ADC DMA fires but motor_on_block_ready never called            |
|   3   | on_block_ready runs but arm_tx never entered                   |
|   4   | arm_tx entered but SPI DMA doesn't arm                         |
|   5   | PB0 raised but no transfer completes                           |
|   6   | full pipeline works                                            |
| flutter | Error_Handler() -- one of the *_init() faulted               |

## Where each file goes

```
rpi5/
  motor_wire.h          REPLACE  (also copy to stm/include/motor_wire.h -- must be identical)
  motor_controller.c    REPLACE
  config.h              NEW
  config.c              NEW
  cJSON.h               DROP IN  (https://github.com/DaveGamble/cJSON, MIT)
  cJSON.c               DROP IN

stm/include/
  motor_wire.h          REPLACE  (identical copy of the one in rpi5/)
  motor_send.h          REPLACE
  motor_source.h        REPLACE
  motor_acquire.h       NEW

stm/src/
  motor_send.c          REPLACE
  motor_acquire.c       NEW
  main.c                REPLACE
  motor_synth.c         DELETE from source list (do NOT link)
  motor_synth.h         DELETE from source list

/system/etc/
  spi.conf              EDIT clock_rate/cpha/word_width/idle_insert as needed
                        (the controller will rewrite this at startup to match
                        cfg.pi.spi_* and bounce spi-dwc)

<any dir>/
  config.json           NEW (path is a command-line argument)
```

motor_wire.h must remain byte-identical on both sides. Best done with a
shared `common/` directory and symlinks/submodule rather than two hand-edited
copies.

## Build

QNX (controller):
```
qcc -Vgcc_ntoaarch64le -std=gnu11 -O2 \
    -I/home/yasmine/hardware-component-samples/common/system/gpio \
    motor_controller.c config.c cJSON.c rpi_gpio.c rpi_spi.c \
    -lm \
    -o motor_controller
```

STM32: build all sources in `stm/src/` EXCEPT `motor_synth.c`. Including
motor_synth.c will produce duplicate-symbol errors on TIM2_IRQHandler,
HAL_TIM_PeriodElapsedCallback, and motor_on_block_ready.

## Running

```
./motor_controller config.json                  # explicit path
./motor_controller /path/to/config.json         # alternate location

slay -s SIGHUP motor_controller                 # reload after the file changes
```

If the file is missing or invalid at startup, the controller logs an error
and falls back to compiled-in defaults (same values as the sample
config.json). If reload fails, the previous good config stays active.

## Runtime behaviour

On startup, the controller:

1. Reads /system/etc/spi.conf and diffs `clock_rate` / `cpha` /
   `word_width` / `idle_insert` against the loaded JSON. If anything
   differs it rewrites spi.conf (atomic tmp+rename) and bounces
   `spi-dwc`. If they all match, no restart.
2. Opens the SPI device, sets up the GPIO event subscription, and
   queues an initial SET_CONFIG to the STM.

The STM may have booted with different defaults, so the sync step
always happens. The Pi accepts frames of any sane size until the STM
ACKs and tags the first frame with `CONFIG_APPLIED`; from that point
on, `n_rows` is locked to the configured value.

On SIGHUP the Pi reloads the file, applies pi-tier changes locally,
and diffs the stm-tier against the active config. If anything in the
stm tier changed, a fresh SET_CONFIG is queued. SIGHUP does NOT re-run
the spi.conf sync -- SPI parameter changes require a restart.

## Field reference

### `pi` (applied locally on the Pi, never sent over the wire)

| field            | type   | range / values     | notes                             |
|------------------|--------|--------------------|-----------------------------------|
| spi_bus          | int    | 0..7               | applied at startup only           |
| spi_dev          | int    | 0..7               | applied at startup only           |
| spi_mode         | int    | 0..3               | applied at startup only           |
| spi_clock_hz     | uint   | 100k..50M          | written into spi.conf at startup  |
| spi_cpha         | int    | 0..1               | written into spi.conf at startup  |
| spi_word_width   | int    | 8, 16, 32          | written into spi.conf at startup  |
| spi_idle_insert  | int    | 0..1               | written into spi.conf at startup  |
| rt_priority      | int    | 1..63              | applied live on SIGHUP            |
| dataready_pin    | int    | 0..27              | applied at startup only           |
| scaling.*        | float  | -1e6..1e6          | written to shm for consumers      |

### `stm` (pushed to STM32 via SET_CONFIG, takes effect only on ACK)

| field            | type   | range / values         | notes                          |
|------------------|--------|------------------------|--------------------------------|
| block_rows       | uint16 | 1..300                 | <= MOTOR_MAX_ROWS_PER_BLOCK    |
| source           | string | "adc"                  | only supported value           |
| run_state        | string | "run" \| "stop"        | stop = ADC halted, resume needs STM reset |
| sample_rate_hz   | uint32 | 100..100000 (0=leave)  | ADC scan trigger rate          |
| imu_rate_hz      | uint32 | 10..1000 (0=leave)     | MPU6050 poll rate; 1000=native max |

Out-of-range or wrong-type values are rejected and the whole file is
discarded; the previous config stays in effect. `synth_cycles` and
`block_period_us` from v1 are silently ignored if present. The reserved
bytes in `config_payload_t` are zeroed automatically.

## Wire-level protocol summary

* **Pi staging**: when a SET_CONFIG is pending, the Pi fills the tx side
  of the next SPI exchange with the command frame (else zeros).
* **STM sniff**: on every TxRxCplt, the first 4 bytes of rx are compared
  to `MOTOR_CMD_MAGIC`. On hit, the frame is copied into a pending buffer.
* **STM apply**: at the next `motor_on_block_ready` (called from ADC DMA
  half/full IRQ), after the current frame is assembled and armed, the
  command is validated (CRC, schema_version, ranges) and either applied
  or rejected. HAL-touching side effects (TIM3 reprogram, DMA re-arm,
  ADC start/stop, TIM2 reprogram) are deferred via flags to
  `motor_acquire_service()`, which runs from the main loop outside any
  ISR. Idempotent: a duplicate `cmd_seq` re-ACKs without re-apply.
* **STM ACK**: rides in the next outbound `frame_header_t`. `flags`
  carries `ACK_OK` / `ACK_NACK` (+ reason) plus a one-shot `CONFIG_APPLIED`
  on the first frame using the new config. `_reserved` carries the low
  16 bits of the cmd_seq the ACK refers to.
* **Pi pairing**: the Pi keeps resending the same command
  (`CMD_MAX_RETRIES` blocks, ~320 ms at 100 Hz) until it sees a header
  with both a matching `_reserved` and an `ACK_*` bit. On `ACK_OK` it
  updates its active stm config; on NACK or timeout it leaves active
  unchanged and logs.

## Diagnostics

Status log on stderr:

```
[ctrl] ok=N drops=N crc=N magic=N ver=N size=N dup=N rst=N to=N spi=N
       cfg(rld=N ack=N nack=N) last(flags=0xNNNN rsv=N)
       sens(cur=A/B/C vib=X/Y/Z rpm=R)
```

* `ok` should climb at approximately (sample_rate_hz / block_rows) per
  second: at defaults, ~100/sec.
* `drops` / `dup` / `rst` climbing linearly during steady state is
  normal alignment fuzz between STM sequence and Pi expectations; only
  worry if they climb at a large fraction of `ok`.
* `crc` / `magic` / `ver` / `size` should stay zero. Any climb indicates
  wire corruption -- lower `spi_clock_hz`, shorten wires.
* `sens(cur=...)` shows the last row's three ADC channels; `vib=...`
  shows the MPU6050 accel readings (16384 counts/g at +-2g range); `rpm=`
  is the tach speed.

STM-side counters (`g_cmd_seen`, `g_cmd_ok`, `g_cmd_nack`, `g_adc_cb`,
`g_imu_reads`, `g_imu_errs`, `g_rpm_caps`, `g_obr`, `g_arm_ok`, `g_sent`)
are declared volatile and can be read via a debugger, printed over SWO,
or piggybacked into a spare header slot for wire visibility.