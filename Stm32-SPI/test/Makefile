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

t: test_motor_send.c fakehal/fakehal.c fakehal/acq_stub.c
	$(CC) $(CFLAGS) $(INCS) -o $@ $^

clean:
	rm -f t
