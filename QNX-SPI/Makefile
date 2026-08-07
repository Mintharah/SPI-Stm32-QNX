QNX800_DIR := ../../../qnx800
TOOLCHAIN_FILE := $(CURDIR)/qnxtoolchain.cmake

.PHONY: all qnx clean

all: qnx

qnx:
	@[ -n "$(QNX_HOST)" ] || [ -f "$(QNX800_DIR)/qnxsdp-env.sh" ] || { echo "QNX SDP not found at $(QNX800_DIR)/qnxsdp-env.sh"; exit 1; }
	@bash -c 'set -e; \
		[ -n "$$QNX_HOST" ] || . "$(QNX800_DIR)/qnxsdp-env.sh"; \
		cmake -S . -B build_qnx \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN_FILE) \
			-DCMAKE_SYSTEM_PROCESSOR=aarch64; \
		cmake --build build_qnx'

clean:
	rm -rf build_qnx

clean:
	rm -rf build_qnx
