STM32F401CC firmware -- dummy-data bring-up
===========================================
Drop this folder into PlatformIO (or copy include/ + src/ + platformio.ini into
an existing project) and build.

Active source path : motor_synth (20 kHz sine, no sensors needed)
Wire contract      : motor_wire.h  (v2: row = current, vib_x/y/z, rpm = 10 bytes)

Pins (SPI2 slave, mode 0, MSB-first):
  PB12 NSS   
  PB13 SCK   
  PB14 MISO   
  PB15 MOSI
  PB0  -> Pi GPIO   (data-ready, rising edge = "frame waiting")
  GND  -> Pi GND    (common ground required)

Build/flash:
  pio run -t upload

Then on the Pi: run motor_controller, then motor_monitor, and watch 'cur'
move (sine) while vib/rpm show their dummy values.

To go live later: replace motor_synth.* with motor_acquire.* (real ADC current
sensor) -- use ONE source module, never both (each owns TIM2).

HAL modules required in stm32f4xx_hal_conf.h (PlatformIO's default enables these):
  RCC, GPIO, DMA, SPI, TIM, PWR, CORTEX, FLASH.
