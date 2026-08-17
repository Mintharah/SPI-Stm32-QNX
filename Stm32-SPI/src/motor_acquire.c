/*
 * motor_acquire.c
 * ----------------------------------------------------------------------------
 * Real-sensor data source. Replaces motor_synth.c (do not link both: TIM2
 * collision, HAL_TIM_PeriodElapsedCallback collision, motor_on_block_ready
 * collision).
 *
 * Architecture (v4, 8 ADC channels)
 * ==================================
 *
 *   TIM3 @ sample_rate_hz  ----TRGO---->  ADC1 SCAN of PA0..PA7 (IN0..IN7)
 *                                            |  (8 conversions per trigger)
 *                                            +-DMA2 Stream 0 Ch 0 (circular)-->
 *                                                |
 *                                                +-> s_adc_buf[2 * block_rows * 3]
 *                                                       |               |
 *                                          half-cplt ---+               +--- full-cplt
 *                                                |                       |
 *                                                v                       v
 *                                          fill_rows(0)           fill_rows(3*block_rows)
 *                                                  \                    /
 *                                                   v                  v
 *                                              motor_on_block_ready(rows, n_rows)
 *
 *   TIM2 update @ imu_rate_hz  ---> HAL_I2C_Mem_Read_IT(MPU6050 ACCEL_X, 6 bytes)
 *                                          |
 *                                          v MemRxCplt
 *                       s_vib_ring_{x,y,z}[VIB_RING_CAP] (ring of IMU polls;
 *                       fill_rows staircase-maps row i -> poll i / rows-per-poll)
 *
 *   TIM4 CH3 (PB8) input capture, 1 MHz counter ---> period_us between pulses
 *                                                          |
 *                                                          v
 *                                              s_rpm = 60_000_000 / period_us
 *
 * ADC uses SCAN mode with NbrOfConversion=8. Each TIM3 trigger initiates a
 * scan of the 8 channels in rank order; DMA writes 8 half-words per trigger.
 * Buffer layout after N triggers: [ ch0_0..ch7_0, ch0_1..ch7_1, ... ].
 * fill_rows unpacks N_CURRENT_CH-at-a-time into motor_row_t.current[0..7].
 * ----------------------------------------------------------------------------
 */
#include "stm32f4xx_hal.h"
#include <string.h>
#include "motor_wire.h"
#include "motor_source.h"
#include "motor_acquire.h"

extern void Error_Handler(void);

#ifndef MOTOR_DEFAULT_BLOCK_ROWS
#define MOTOR_DEFAULT_BLOCK_ROWS  200u
#endif

#define ACQUIRE_DEFAULT_SAMPLE_RATE_HZ  20000u
#define ACQUIRE_DEFAULT_IMU_RATE_HZ     1000u
#define ACQUIRE_MIN_IMU_RATE_HZ         10u
#define ACQUIRE_MAX_IMU_RATE_HZ         1000u

#define N_CURRENT_CH                    8u   /* PA0..PA7 (ADC1 IN0..IN7)         */

/* MPU6050 addressing and register map. AD0 low -> 0x68.                       */
#define MPU6050_ADDR_8       (0x68u << 1)   /* AD0 low  */
#define MPU6050_ADDR_8_ALT   (0x69u << 1)   /* AD0 high or floating */

/* How often to re-probe for an absent IMU, in TIM2 ticks. At the default
 * 1 kHz that is once a second -- cheap, and it means an MPU that is late to
 * power up, or reconnected while running, is picked up instead of being
 * written off at boot. */
#define IMU_REPROBE_TICKS    1000u
#define MPU_REG_SMPLRT       0x19u
#define MPU_REG_CONFIG       0x1Au
#define MPU_REG_GYRO_CFG     0x1Bu
#define MPU_REG_ACCEL_CFG    0x1Cu
#define MPU_REG_ACCEL_XOUT_H 0x3Bu
#define MPU_REG_PWR_MGMT_1   0x6Bu

#define TIM_PCLK_HZ          84000000u   /* TIM2/3/4 input clock                */

/* ---- HAL handles --------------------------------------------------------- */
static ADC_HandleTypeDef s_hadc1;
static DMA_HandleTypeDef s_hdma_adc;
static TIM_HandleTypeDef s_htim2;    /* IMU I2C poll @ imu_rate_hz             */
static TIM_HandleTypeDef s_htim3;    /* ADC scan trigger @ sample_rate_hz      */
static TIM_HandleTypeDef s_htim4;    /* RPM tach input capture (PB8, CH3)      */
static I2C_HandleTypeDef s_hi2c1;

/* ---- buffers ------------------------------------------------------------- */
/* ADC circular DMA target: 2 halves x block_rows rows x 3 channels per row.
 * Sized for MAX so runtime block_rows changes just adjust the DMA count.     */
static uint16_t    s_adc_buf[2u * MOTOR_MAX_ROWS_PER_BLOCK * N_CURRENT_CH];
static motor_row_t s_row_buf[2][MOTOR_MAX_ROWS_PER_BLOCK];
static volatile uint8_t s_row_idx = 0;
static uint8_t s_i2c_buf[6];

/* ---- live sensor state (set by ISRs, read by fill_rows) ------------------ */
/* Vibration polls are pushed into a ring by the I2C completion ISR. fill_rows
 * must not read a single ZOH cache: the I2C EV ISR and the ADC DMA ISR share
 * NVIC priority 5, so the callback cannot preempt fill_rows and a plain cache
 * would freeze to one value per whole block. The ring keeps every poll that
 * arrived during the block (10 @ 1 kHz per 200-row/10 ms block) so fill_rows
 * can staircase them across the rows. CAP is a power of two (> 10). */
#define VIB_RING_CAP                32u
static int16_t   s_vib_ring_x[VIB_RING_CAP];
static int16_t   s_vib_ring_y[VIB_RING_CAP];
static int16_t   s_vib_ring_z[VIB_RING_CAP];
static volatile uint32_t s_vib_count = 0;   /* completed polls (monotonic)   */
static volatile uint16_t s_rpm   = 0;
static volatile uint32_t s_last_capture = 0;
static volatile uint8_t  s_capture_seen = 0;
static volatile uint32_t s_rpm_last_ms  = 0;

/* Report 0 rather than a stale reading once the tach goes quiet.
 *
 * s_rpm was only ever assigned inside the capture interrupt, so when the shaft
 * stopped the last measured value stayed in every row forever -- a stopped
 * motor published its old running speed indefinitely.
 *
 * 200 ms is comfortably longer than the slowest period the 16-bit 1 MHz counter
 * can even represent (65.5 ms, ~915 RPM), so this cannot fire on a shaft that
 * is still turning fast enough to be measured at all. */
#define RPM_STALE_MS  200u

/* ---- active config ------------------------------------------------------- */
static uint16_t s_block_rows;
static uint32_t s_sample_rate_hz;
static uint32_t s_imu_rate_hz;
static uint8_t  s_running  = 0;
static uint8_t  s_have_imu  = 0;
static uint8_t  s_mpu_addr  = MPU6050_ADDR_8;  /* whichever answered */
static volatile uint32_t s_imu_absent_ticks = 0;

/* ---- deferred reconfig flags (set in ISR apply, drained in service) ------ */
static volatile uint8_t  s_pending_run_state    = 0xFFu;
static volatile uint32_t s_pending_sample_rate  = 0u;
static volatile uint32_t s_pending_imu_rate     = 0u;
static volatile uint16_t s_pending_block_rows   = 0u;

/* ---- diagnostic counters (extern in main.c for LED stages) --------------- */
volatile uint32_t g_adc_cb    = 0;
volatile uint32_t g_imu_reads = 0;
/* Absence used to be completely silent: s_have_imu==0 made the TIM2 callback
 * return without touching any counter, so g_imu_reads and g_imu_errs both sat
 * at 0 and there was no way to tell "no IMU fitted" from "IMU never polled".
 * These two make the difference visible.                                    */
volatile uint32_t g_imu_absent    = 0;   /* polls skipped, no IMU present   */
volatile uint32_t g_imu_addr_found = 0;  /* 7-bit address, 0 = none         */
volatile uint32_t g_imu_errs  = 0;
volatile uint32_t g_rpm_caps  = 0;

/* ADC DMA overrun counter. Set when a conversion completed while the DMA had
 * not yet drained the previous result (OVR latched) -- the DMA was starved of
 * bus time, typically by the boundary frame-assembly burst. The lost sample
 * shifts the sampled phase by a fraction of a period, which is exactly the
 * "drop" visible in the current waveforms at a block boundary. Latched into
 * the next frame's flags via motor_acquire_take_overrun_flag().             */
volatile uint32_t g_adc_ovr = 0;
static volatile uint8_t s_overrun_pending = 0;

/* Dynamic prescaler+ARR search to land a 16-bit (PSC, ARR) pair from Hz. */
static void compute_psc_arr(uint32_t target_hz, uint32_t *out_psc, uint32_t *out_arr)
{
    uint64_t total = (uint64_t)TIM_PCLK_HZ / target_hz;
    if (total == 0) total = 1;
    uint32_t presc = 1;
    while ((total + presc - 1) / presc > 0x10000u) {
        presc <<= 1;
        if (presc > 0x10000u) { presc = 0x10000u; break; }
    }
    uint32_t arr = (uint32_t)((total + presc - 1) / presc);
    if (arr == 0)       arr = 1;
    if (arr > 0x10000u) arr = 0x10000u;
    *out_psc = presc - 1u;
    *out_arr = arr - 1u;
}

/* ==========================================================================
 * ADC1 (3-channel scan) + TIM3 + DMA2 Stream 0
 * ========================================================================*/

static void adc_gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3   /* PA0..PA7   */
           | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;  /* IN0..IN7   */
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
}

static void adc_dma_init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    s_hdma_adc.Instance                 = DMA2_Stream0;
    s_hdma_adc.Init.Channel             = DMA_CHANNEL_0;
    s_hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_adc.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    s_hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    s_hdma_adc.Init.Mode                = DMA_CIRCULAR;
    s_hdma_adc.Init.Priority            = DMA_PRIORITY_HIGH;
    s_hdma_adc.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&s_hdma_adc) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&s_hadc1, DMA_Handle, s_hdma_adc);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void adc_init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    s_hadc1.Instance                   = ADC1;
    s_hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    s_hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    s_hadc1.Init.ScanConvMode          = ENABLE;              /* multi-channel */
    s_hadc1.Init.ContinuousConvMode    = DISABLE;
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;
    s_hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    s_hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO;
    s_hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    s_hadc1.Init.NbrOfConversion       = N_CURRENT_CH;        /* 3 per trigger */
    s_hadc1.Init.DMAContinuousRequests = ENABLE;
    s_hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV; /* DMA per conv  */
    if (HAL_ADC_Init(&s_hadc1) != HAL_OK) Error_Handler();

    ADC_ChannelConfTypeDef ch = {0};
    ch.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    /* Ranks 1..N map to ADC1 IN0..IN(N-1) == PA0..PA(N-1), contiguous, so the
     * channel number equals the loop index (ADC_CHANNEL_0..7 are the plain
     * integers 0..7 on the F4 HAL). NOTE: this identity only holds up to 8
     * channels -- IN8 is PB0, IN9 is PB1, so a count > 8 must extend
     * adc_gpio_init() to the correct ports and can no longer use i as the
     * channel number. Also mind the scan budget: N * (15+12) ADC cycles at
     * PCLK2/4 = 21 MHz must fit inside one sample period (8 ch = 10.3 us; fine
     * at 20 kHz / 50 us, but drop SamplingTime to 3 cycles above ~90 kHz).   */
    for (uint32_t i = 0; i < N_CURRENT_CH; ++i) {
        ch.Channel = i;
        ch.Rank    = i + 1u;
        if (HAL_ADC_ConfigChannel(&s_hadc1, &ch) != HAL_OK) Error_Handler();
    }
}

static void tim3_program(uint32_t sample_rate_hz)
{
    if (sample_rate_hz == 0) sample_rate_hz = ACQUIRE_DEFAULT_SAMPLE_RATE_HZ;

    uint32_t psc, arr;
    compute_psc_arr(sample_rate_hz, &psc, &arr);

    s_htim3.Init.Prescaler         = psc;
    s_htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim3.Init.Period            = arr;
    s_htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&s_htim3) != HAL_OK) Error_Handler();

    TIM_MasterConfigTypeDef m = {0};
    m.MasterOutputTrigger = TIM_TRGO_UPDATE;
    m.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&s_htim3, &m) != HAL_OK)
        Error_Handler();
}

static void tim3_init(uint32_t sample_rate_hz)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    s_htim3.Instance = TIM3;
    tim3_program(sample_rate_hz);
}

/* ==========================================================================
 * I2C1 + MPU6050 + TIM2 (configurable IMU poll rate)
 * ========================================================================*/

static void i2c1_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;     /* SCL=PB6, SDA=PB7           */
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);

    __HAL_RCC_I2C1_CLK_ENABLE();
    s_hi2c1.Instance             = I2C1;
    s_hi2c1.Init.ClockSpeed      = 400000;
    s_hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    s_hi2c1.Init.OwnAddress1     = 0;
    s_hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    s_hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    s_hi2c1.Init.OwnAddress2     = 0;
    s_hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    s_hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&s_hi2c1) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

static HAL_StatusTypeDef mpu_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&s_hi2c1, s_mpu_addr, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

/* Probe both possible addresses and keep the one that answers.
 *
 * AD0 low is 0x68 and AD0 high is 0x69, and the old code only ever looked at
 * 0x68 -- so a board with AD0 tied high, or left floating and reading high, was
 * reported as "no IMU" forever. Trying both costs one extra probe at startup.
 *
 * Returns 1 if an MPU answered and was configured. */
static uint8_t mpu6050_try_init(void)
{
    static const uint8_t addrs[2] = { MPU6050_ADDR_8, MPU6050_ADDR_8_ALT };

    for (unsigned i = 0; i < 2; ++i) {
        if (HAL_I2C_IsDeviceReady(&s_hi2c1, addrs[i], 2, 50) != HAL_OK)
            continue;

        s_mpu_addr = addrs[i];

        /* Check the writes. Previously their results were discarded, so a
         * device that answered the probe but failed to configure -- a wiring
         * fault appearing under load, say -- looked fully initialised and then
         * produced nothing. */
        if (mpu_write(MPU_REG_PWR_MGMT_1, 0x00) != HAL_OK) continue;
        if (mpu_write(MPU_REG_SMPLRT,     0x00) != HAL_OK) continue;
        if (mpu_write(MPU_REG_CONFIG,     0x00) != HAL_OK) continue;
        if (mpu_write(MPU_REG_GYRO_CFG,   0x00) != HAL_OK) continue;
        if (mpu_write(MPU_REG_ACCEL_CFG,  0x00) != HAL_OK) continue;

        g_imu_addr_found = addrs[i] >> 1;
        return 1u;
    }
    return 0u;
}

static void mpu6050_init(void)
{
    s_have_imu = mpu6050_try_init();
}

static void tim2_program(uint32_t imu_rate_hz)
{
    if (imu_rate_hz == 0)                       imu_rate_hz = ACQUIRE_DEFAULT_IMU_RATE_HZ;
    if (imu_rate_hz < ACQUIRE_MIN_IMU_RATE_HZ)  imu_rate_hz = ACQUIRE_MIN_IMU_RATE_HZ;
    if (imu_rate_hz > ACQUIRE_MAX_IMU_RATE_HZ)  imu_rate_hz = ACQUIRE_MAX_IMU_RATE_HZ;

    uint32_t psc, arr;
    compute_psc_arr(imu_rate_hz, &psc, &arr);

    s_htim2.Init.Prescaler         = psc;
    s_htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim2.Init.Period            = arr;
    s_htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&s_htim2) != HAL_OK) Error_Handler();
}

static void tim2_init(uint32_t imu_rate_hz)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    s_htim2.Instance = TIM2;
    tim2_program(imu_rate_hz);
    HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* ==========================================================================
 * TIM4 CH3 input capture (RPM tach on PB8)
 * ========================================================================*/

static void tim4_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_8;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLDOWN;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &g);

    __HAL_RCC_TIM4_CLK_ENABLE();
    s_htim4.Instance               = TIM4;
    s_htim4.Init.Prescaler         = 84u - 1u;    /* 1 MHz tick, 1 us res    */
    s_htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim4.Init.Period            = 0xFFFFu;
    s_htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&s_htim4) != HAL_OK) Error_Handler();

    TIM_IC_InitTypeDef ic = {0};
    ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = 0x3;
    if (HAL_TIM_IC_ConfigChannel(&s_htim4, &ic, TIM_CHANNEL_3) != HAL_OK)
        Error_Handler();

    HAL_NVIC_SetPriority(TIM4_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* ==========================================================================
 * Row composition (called from ADC DMA half/full IRQ)
 * ========================================================================*/

static void fill_rows(uint32_t adc_hword_offset)
{
    uint8_t         idx = s_row_idx;
    motor_row_t    *dst = s_row_buf[idx];
    const uint16_t *src = &s_adc_buf[adc_hword_offset];

    /* Detect lost conversions before composing the block. OVR latches when a
     * scan completed but the DMA had not drained the previous result (bus
     * starvation during the boundary frame-assembly burst). Clear it so the
     * ADC can resume; the latch rides the next frame's flags. */
    if (__HAL_ADC_GET_FLAG(&s_hadc1, ADC_FLAG_OVR)) {
        g_adc_ovr++;
        s_overrun_pending = 1;
        __HAL_ADC_CLEAR_FLAG(&s_hadc1, ADC_FLAG_OVR);
    }

    /* Age out a stale tach reading before it is stamped into another block. */
    uint16_t rpm = s_rpm;
    if (s_capture_seen && (HAL_GetTick() - s_rpm_last_ms) > RPM_STALE_MS) {
        s_rpm          = 0u;
        s_capture_seen = 0u;
        rpm            = 0u;
    }

    uint16_t n = s_block_rows;
    /* Staircase the IMU polls across the block: with the defaults (20 kHz
     * sample rate, 1 kHz IMU rate, 200 rows) each poll spans exactly 20 rows
     * (rpp) and each block carries 10 polls (nspb). Row i gets poll
     * (newest - nspb + 1) + i/rpp from the ring. Reads and writes cannot tear:
     * the I2C EV ISR and this ADC DMA ISR are both NVIC priority 5. */
    uint32_t rpp   = (s_sample_rate_hz && s_imu_rate_hz)
                     ? s_sample_rate_hz / s_imu_rate_hz : 1u;
    uint32_t nspb  = (n + rpp - 1u) / rpp;
    uint32_t newest = (s_vib_count >= nspb) ? s_vib_count - 1u : nspb - 1u;
    for (uint16_t i = 0; i < n; ++i) {
        /* Scan order in the DMA buffer matches the ADC channel ranks 1..N. */
        for (uint32_t c = 0; c < N_CURRENT_CH; ++c)
            dst[i].current[c] = src[i * N_CURRENT_CH + c];
        uint32_t ridx = (newest + 1u - nspb + (uint32_t)(i / rpp))
                        & (VIB_RING_CAP - 1u);
        dst[i].vib_x   = s_vib_ring_x[ridx];
        dst[i].vib_y   = s_vib_ring_y[ridx];
        dst[i].vib_z   = s_vib_ring_z[ridx];
        dst[i].rpm     = rpm;
    }
    s_row_idx ^= 1u;
    g_adc_cb++;
    motor_on_block_ready(dst, n);
}

/* ==========================================================================
 * HAL callbacks
 * ========================================================================*/

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;
    fill_rows(0);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;
    fill_rows((uint32_t)s_block_rows * N_CURRENT_CH);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) return;
    uint32_t slot = s_vib_count & (VIB_RING_CAP - 1u);
    s_vib_ring_x[slot] = (int16_t)(((uint16_t)s_i2c_buf[0] << 8) | s_i2c_buf[1]);
    s_vib_ring_y[slot] = (int16_t)(((uint16_t)s_i2c_buf[2] << 8) | s_i2c_buf[3]);
    s_vib_ring_z[slot] = (int16_t)(((uint16_t)s_i2c_buf[4] << 8) | s_i2c_buf[5]);
    s_vib_count++;
    g_imu_reads++;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) return;
    g_imu_errs++;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    if (!s_have_imu) {
        /* Count it, so an absent IMU is visible rather than silent, and retry
         * occasionally instead of giving up for the lifetime of the board. */
        g_imu_absent++;
        if (++s_imu_absent_ticks >= IMU_REPROBE_TICKS) {
            s_imu_absent_ticks = 0;
            s_have_imu = mpu6050_try_init();
        }
        return;
    }
    if (HAL_I2C_Mem_Read_IT(&s_hi2c1, MPU6050_ADDR_8,
                            MPU_REG_ACCEL_XOUT_H,
                            I2C_MEMADD_SIZE_8BIT,
                            s_i2c_buf, 6) != HAL_OK) {
        g_imu_errs++;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM4) return;
    uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    if (s_capture_seen) {
        uint32_t period_us = (now - s_last_capture) & 0xFFFFu;
        if (period_us > 0u) {
            uint32_t rpm = 60000000UL / period_us;
            if (rpm > 65535u) rpm = 65535u;
            s_rpm = (uint16_t)rpm;
        }
    } else {
        s_capture_seen = 1u;
    }
    s_last_capture = now;
    s_rpm_last_ms  = HAL_GetTick();
    g_rpm_caps++;
}

/* ==========================================================================
 * NVIC handlers (vector table entries)
 * ========================================================================*/

void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&s_hdma_adc); }
void ADC_IRQHandler(void)          { HAL_ADC_IRQHandler(&s_hadc1);    }
void I2C1_EV_IRQHandler(void)      { HAL_I2C_EV_IRQHandler(&s_hi2c1); }
void I2C1_ER_IRQHandler(void)      { HAL_I2C_ER_IRQHandler(&s_hi2c1); }
void TIM2_IRQHandler(void)         { HAL_TIM_IRQHandler(&s_htim2);    }
void TIM4_IRQHandler(void)         { HAL_TIM_IRQHandler(&s_htim4);    }

/* ==========================================================================
 * Public API
 * ========================================================================*/

void motor_acquire_init(uint16_t block_rows, uint32_t sample_rate_hz, uint32_t imu_rate_hz)
{
    if (block_rows == 0 || block_rows > MOTOR_MAX_ROWS_PER_BLOCK)
        block_rows = MOTOR_DEFAULT_BLOCK_ROWS;
    if (sample_rate_hz == 0u)
        sample_rate_hz = ACQUIRE_DEFAULT_SAMPLE_RATE_HZ;
    if (imu_rate_hz == 0u)
        imu_rate_hz = ACQUIRE_DEFAULT_IMU_RATE_HZ;
    if (imu_rate_hz < ACQUIRE_MIN_IMU_RATE_HZ) imu_rate_hz = ACQUIRE_MIN_IMU_RATE_HZ;
    if (imu_rate_hz > ACQUIRE_MAX_IMU_RATE_HZ) imu_rate_hz = ACQUIRE_MAX_IMU_RATE_HZ;

    s_block_rows     = block_rows;
    s_sample_rate_hz = sample_rate_hz;
    s_imu_rate_hz    = imu_rate_hz;

    adc_gpio_init();
    adc_dma_init();
    adc_init();
    tim3_init(sample_rate_hz);

    i2c1_init();
    mpu6050_init();
    tim2_init(imu_rate_hz);

    tim4_init();
}

void motor_acquire_start(void)
{
    if (s_running) return;
    /* DMA count = 2 (halves) * block_rows (rows/half) * N_CURRENT_CH (samples/row) */
    if (HAL_ADC_Start_DMA(&s_hadc1, (uint32_t *)s_adc_buf,
                          2u * s_block_rows * N_CURRENT_CH) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_Base_Start(&s_htim3) != HAL_OK) Error_Handler();
    if (HAL_TIM_Base_Start_IT(&s_htim2) != HAL_OK) Error_Handler();
    if (HAL_TIM_IC_Start_IT(&s_htim4, TIM_CHANNEL_3) != HAL_OK)
        Error_Handler();
    s_running = 1;
}

void motor_acquire_stop(void)
{
    if (!s_running) return;
    HAL_TIM_IC_Stop_IT(&s_htim4, TIM_CHANNEL_3);
    HAL_TIM_Base_Stop_IT(&s_htim2);
    HAL_TIM_Base_Stop(&s_htim3);
    HAL_ADC_Stop_DMA(&s_hadc1);
    s_running = 0;
}

void motor_acquire_set_block_rows(uint16_t block_rows)
{
    if (block_rows == 0 || block_rows > MOTOR_MAX_ROWS_PER_BLOCK) return;
    s_pending_block_rows = block_rows;
}

void motor_acquire_set_sample_rate(uint32_t sample_rate_hz)
{
    if (sample_rate_hz == 0u) return;
    s_pending_sample_rate = sample_rate_hz;
}

void motor_acquire_set_imu_rate(uint32_t imu_rate_hz)
{
    if (imu_rate_hz == 0u) return;
    if (imu_rate_hz < ACQUIRE_MIN_IMU_RATE_HZ) imu_rate_hz = ACQUIRE_MIN_IMU_RATE_HZ;
    if (imu_rate_hz > ACQUIRE_MAX_IMU_RATE_HZ) imu_rate_hz = ACQUIRE_MAX_IMU_RATE_HZ;
    s_pending_imu_rate = imu_rate_hz;
}

void motor_acquire_set_run_state(uint8_t run_state)
{
    if (run_state > 1u) return;
    s_pending_run_state = run_state;
}

uint8_t motor_acquire_take_overrun_flag(void)
{
    uint8_t f = s_overrun_pending;
    s_overrun_pending = 0;
    return f;
}

void motor_acquire_service(void)
{
    /* ADC/TIM3 group: sample_rate + block_rows together (DMA needs re-arm). */
    uint32_t new_rate = s_pending_sample_rate;
    uint16_t new_rows = s_pending_block_rows;
    if (new_rate != 0u || new_rows != 0u) {
        s_pending_sample_rate = 0u;
        s_pending_block_rows  = 0u;
        uint8_t was_running = s_running;
        if (was_running) {
            HAL_ADC_Stop_DMA(&s_hadc1);
            HAL_TIM_Base_Stop(&s_htim3);
        }
        if (new_rate != 0u) {
            s_sample_rate_hz = new_rate;
            tim3_program(new_rate);
        }
        if (new_rows != 0u) {
            s_block_rows = new_rows;
        }
        if (was_running) {
            HAL_TIM_Base_Start(&s_htim3);
            HAL_ADC_Start_DMA(&s_hadc1, (uint32_t *)s_adc_buf,
                              2u * s_block_rows * N_CURRENT_CH);
        }
    }

    /* TIM2 / IMU rate (independent of ADC group). */
    uint32_t new_imu = s_pending_imu_rate;
    if (new_imu != 0u) {
        s_pending_imu_rate = 0u;
        uint8_t was_running = s_running;
        if (was_running) HAL_TIM_Base_Stop_IT(&s_htim2);
        s_imu_rate_hz = new_imu;
        tim2_program(new_imu);
        if (was_running) HAL_TIM_Base_Start_IT(&s_htim2);
    }

    /* Run state deferred so the ACK frame ships before ADC actually stops. */
    uint8_t r = s_pending_run_state;
    if (r != 0xFFu) {
        s_pending_run_state = 0xFFu;
        if (r == 0u) motor_acquire_stop();
        else         motor_acquire_start();
    }
}