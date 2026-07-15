/*
 * motor_source.h
 * ----------------------------------------------------------------------------
 * Producer -> consumer contract. motor_acquire (or formerly motor_synth)
 * implements the producer; motor_send.c implements motor_on_block_ready as
 * the consumer.
 *
 * SIGNATURE CHANGE in schema v2: motor_on_block_ready now takes a const
 * motor_row_t * (fully composed rows, including IMU + RPM) instead of a
 * raw uint16_t * of current samples. The producer is responsible for row
 * composition because only it knows the live sensor cache; motor_send no
 * longer fabricates anything.
 *
 * The pointer is valid for the duration of the call only. motor_send must
 * memcpy out of `rows` before returning, because the producer will reuse
 * its row buffer on the next block.
 * ----------------------------------------------------------------------------
 */
#ifndef MOTOR_SOURCE_H
#define MOTOR_SOURCE_H

#include <stdint.h>
#include "motor_wire.h"   /* motor_row_t */

void motor_on_block_ready(const motor_row_t *rows, uint16_t n_rows);

#endif /* MOTOR_SOURCE_H */