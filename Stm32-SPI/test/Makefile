# Host-side unit tests for the SPI send state machine.
#
# motor_send.c is #included by the test rather than linked, so the tests can
# reach its statics and drive the real code. The HAL underneath is a recording
# stub in fakehal/ -- nothing emulates hardware; the point is to observe what
# the state machine actually does.
#
#   make        build and run
#   make clean
CC      ?= gcc
CFLAGS  ?= -std=gnu11 -g -O0 -Wall
INCS     = -Ifakehal -I../src -I../include

.PHONY: all clean
all: t
	./t

# ../src/motor_send.c is a real dependency -- the test #includes it. Leaving it
# out meant editing the firmware and re-running make silently retested the old
# binary, which is worse than having no tests at all.
t: test_motor_send.c fakehal/fakehal.c fakehal/acq_stub.c ../src/motor_send.c \
   ../include/motor_wire.h fakehal/stm32f4xx_hal.h
	$(CC) $(CFLAGS) $(INCS) -o $@ test_motor_send.c fakehal/fakehal.c fakehal/acq_stub.c

clean:
	rm -f t
