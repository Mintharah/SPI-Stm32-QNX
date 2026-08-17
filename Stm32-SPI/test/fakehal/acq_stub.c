#include "motor_acquire.h"
int fa_set_block_rows=0, fa_set_sample_rate=0, fa_set_imu_rate=0, fa_set_run_state=0;
int fa_overrun_flag = 0;
void motor_acquire_set_block_rows(uint16_t v){(void)v; fa_set_block_rows++;}
void motor_acquire_set_sample_rate(uint32_t v){(void)v; fa_set_sample_rate++;}
void motor_acquire_set_imu_rate(uint32_t v){(void)v; fa_set_imu_rate++;}
void motor_acquire_set_run_state(uint8_t v){(void)v; fa_set_run_state++;}
uint8_t motor_acquire_take_overrun_flag(void)
{
    int f = fa_overrun_flag;
    fa_overrun_flag = 0;
    return (uint8_t)f;
}
