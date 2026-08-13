#ifndef MOTOR_ACQUIRE_H
#define MOTOR_ACQUIRE_H
#include <stdint.h>
/* Recording stubs: the tests assert on WHICH of these a SET_CONFIG touched. */
extern int fa_set_block_rows, fa_set_sample_rate, fa_set_imu_rate, fa_set_run_state;
void motor_acquire_set_block_rows(uint16_t);
void motor_acquire_set_sample_rate(uint32_t);
void motor_acquire_set_imu_rate(uint32_t);
void motor_acquire_set_run_state(uint8_t);
#endif
