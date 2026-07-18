.PHONY: all qnx stm32 clean

all: qnx

qnx:
	$(MAKE) -C QNX-SPI qnx

stm32:
	@echo "==> Building Stm32-SPI (PlatformIO)..."
	$(MAKE) -C Stm32-SPI 2>/dev/null || \
		pio run -d Stm32-SPI 2>/dev/null || \
		echo "  (skip — install PlatformIO or add a Makefile to Stm32-SPI)"

clean:
	$(MAKE) -C QNX-SPI clean
