/*
 * motor_acquire.h
 * ----------------------------------------------------------------------------
 * Real-sensor producer. Replaces motor_synth.c entirely -- both can't be
 * linked at once (TIM2 conflict, motor_on_block_ready collision).
 *
 *   ADC1 IN0 (PA0)          current sample at sample_rate_hz, TIM3-triggered
 *   I2C1 (PB6 SCL, PB7 SDA) MPU6050 accel polled at imu_rate_hz via TIM2
 *   TIM4 CH3 (PB8)          tach input capture for RPM
 *
 * Data flow: ADC -> DMA2 Stream 0 -> circular buffer of 2 * block_rows
 * uint16. Each half/full DMA IRQ composes one block of motor_row_t by
 * zipping current samples with the latest cached IMU + RPM (ZOH), then
 * calls motor_on_block_ready(rows, n_rows).
 *
 * Runtime reconfig uses the deferred-service pattern: the apply path
 * (inside motor_send's ISR-context process_pending_cmd) only sets flags;
 * motor_acquire_service() called from main() performs the actual HAL
 * reconfig outside any ISR.
 *
 *   Expected call order at boot:
 *       motor_send_init(BLOCK_ROWS);
 *       motor_acquire_init(BLOCK_ROWS, SAMPLE_RATE_HZ, IMU_RATE_HZ);
 *       motor_acquire_start();
 *
 *   ...then in main loop:
 *       for (;;) { motor_acquire_service(); ... }
 * ----------------------------------------------------------------------------
 */
#ifndef MOTOR_ACQUIRE_H
#define MOTOR_ACQUIRE_H

#include <stdint.h>

void motor_acquire_init(uint16_t block_rows, uint32_t sample_rate_hz, uint32_t imu_rate_hz);
void motor_acquire_start(void);
void motor_acquire_stop(void);

/* Call periodically from main() (NOT from any ISR). Drains deferred
 * apply work: sample_rate change, block_rows change, imu_rate change,
 * run_state change. Cheap when idle.                                    */
void motor_acquire_service(void);

/* Deferred setters -- safe to call from ISR (apply path). Effects land
 * on the next motor_acquire_service() tick from the main loop.          */
void motor_acquire_set_block_rows(uint16_t block_rows);
void motor_acquire_set_sample_rate(uint32_t sample_rate_hz);
void motor_acquire_set_imu_rate(uint32_t imu_rate_hz);
void motor_acquire_set_run_state(uint8_t run_state);

#endif /* MOTOR_ACQUIRE_H */