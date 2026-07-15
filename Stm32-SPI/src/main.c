/*
 * main.c -- STM32F401CC bring-up (motor_acquire build)
 * ----------------------------------------------------------------------------
 * LED reports how far the pipeline got. Count blinks per group:
 *
 *   1 = main() runs but ADC DMA callbacks never fire   -> TIM3/ADC/DMA dead
 *   2 = ADC DMA fires but motor_on_block_ready never called
 *   3 = on_block_ready runs but arm_tx never entered
 *   4 = arm_tx entered but HAL_SPI_TransmitReceive_DMA fails
 *   5 = PB0 raised, no transfer completes (wire or Pi-side)
 *   6 = a frame completed -> SUCCESS
 *
 *   fast flutter = Error_Handler() -> one of the *_init() faulted
 *
 * Main loop calls motor_acquire_service() to drain deferred reconfig work
 * (sample-rate change, block_rows change, run_state change) outside any ISR.
 *
 * MPU6050 wiring (I2C1, 3V3):
 *   VCC -> 3V3   GND -> GND   SCL -> PB6   SDA -> PB7   AD0 -> GND   INT -> NC
 * Current sense:    PA0..PA7 (ADC1_IN0..IN7, 0..3V3 range, 8-channel scan)
 * RPM tach (TTL):   PB8 (TIM4 CH3, rising-edge capture)
 * ----------------------------------------------------------------------------
 */
#include "stm32f4xx_hal.h"
#include "motor_acquire.h"
#include "motor_send.h"

void SystemClock_Config(void);

#define BLOCK_ROWS      200u
#define SAMPLE_RATE_HZ  20000u
#define IMU_RATE_HZ     1000u    /* MPU6050 max accel output rate */

/* diagnostic counters defined in motor_acquire.c / motor_send.c */
extern volatile uint32_t g_adc_cb;     /* DMA half/full callbacks (producer) */
extern volatile uint32_t g_obr;        /* motor_on_block_ready entered       */
extern volatile uint32_t g_arm_called; /* arm_tx entered                     */
extern volatile uint32_t g_arm_ok;     /* PB0 raised (DMA armed)             */
extern volatile uint32_t g_arm_fail;   /* DMA arm failed                     */
extern volatile uint32_t g_sent;       /* SPI transfer completed             */
extern volatile uint32_t g_imu_reads;  /* successful MPU6050 reads           */
extern volatile uint32_t g_imu_errs;   /* I2C errors                         */
extern volatile uint32_t g_rpm_caps;   /* tach captures                      */

static void led_on(void)  { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); }
static void led_off(void) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   }

static void led_init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_13;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
    led_off();
}

/* Blink-group helper that ALSO services deferred apply work between LED
 * edges, so reconfigs don't have to wait a whole blink cycle. service()
 * is cheap when there's nothing pending.                                 */
static void blink_group(int n)
{
    for (int i = 0; i < n; ++i) {
        motor_acquire_service();
        led_on();  HAL_Delay(150);
        motor_acquire_service();
        led_off(); HAL_Delay(250);
    }
    motor_acquire_service();
    HAL_Delay(1500);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    led_init();

    motor_send_init(BLOCK_ROWS);
    HAL_Delay(10);
    motor_acquire_init(BLOCK_ROWS, SAMPLE_RATE_HZ, IMU_RATE_HZ);
    HAL_Delay(10);
    motor_acquire_start();

    for (;;) {
        motor_acquire_service();

        int stage = 1;
        if (g_adc_cb)     stage = 2;
        if (g_obr)        stage = 3;
        if (g_arm_called) stage = 4;
        if (g_arm_ok)     stage = 5;
        if (g_sent)       stage = 6;
        blink_group(stage);
    }
}

void SysTick_Handler(void) { HAL_IncTick(); }

void Error_Handler(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_13; g.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &g);
    for (;;) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        for (volatile uint32_t d = 0; d < 300000u; ++d) { }
    }
}