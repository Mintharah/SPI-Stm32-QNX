/*
 * system_clock_config.c
 * ----------------------------------------------------------------------------
 * Corrected SystemClock_Config() for the STM32F401CC "Black Pill".
 *
 * Replaces the broken version that left the PLL OFF -- the core ran on raw HSI
 * at 16 MHz while the build assumed 84 MHz, so every TIM/ADC/SPI timing was off
 * by 5.25x. This brings the part to its real 84 MHz target.
 *
 * HSI 16 MHz -> PLL -> 84 MHz:
 *     VCO in  = HSI / PLLM = 16 / 16   = 1   MHz   (valid 1-2 MHz)
 *     VCO out = VCO in * PLLN = 1 * 336 = 336 MHz   (valid 100-432 MHz)
 *     SYSCLK  = VCO out / PLLP = 336 / 4 = 84  MHz
 * Bus clocks: HCLK = 84, PCLK1 = 42 (APB1 max), PCLK2 = 84 MHz.
 *
 * Two things that MUST accompany 84 MHz on the F401 (at VDD 2.7-3.6 V):
 *   - Voltage scaling = Scale 2  (Scale 3 tops out at 60 MHz).
 *   - Flash latency   = 2 wait states, set BEFORE the clock is raised.
 * HAL_RCC_ClockConfig() applies the flash latency in the correct order relative
 * to the SYSCLK switch, so passing FLASH_LATENCY_2 here is what prevents the
 * bus-fault-on-first-instruction-fetch you'd get from an un-raised latency.
 *
 * This is the HAL/CubeMX form (you have a SystemClock_Config(), which is the HAL
 * convention). On LL or register-level, say so and I'll give that form -- the
 * numbers are identical, only the API changes.
 * ----------------------------------------------------------------------------
 */
#include "stm32f4xx_hal.h"

/* Declared, not assumed. This file calls Error_Handler() and had no
 * declaration in scope, so it linked only because the implicit-declaration
 * signature happened to match the real one -- and it warned on every build.
 * main.c defines it. */
extern void Error_Handler(void);

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* Scale 2 is required to reach 84 MHz on the F401. */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* HSI on, PLL on. PLLQ=7 -> 48 MHz (only matters if you use USB OTG/SDIO,
     * but it must still be a valid divider). */
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 16;
    osc.PLL.PLLN            = 336;
    osc.PLL.PLLP            = RCC_PLLP_DIV4;
    osc.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();                /* don't silently run at the wrong clock */
    }

    /* Switch SYSCLK to the PLL and set the bus prescalers.
     * AHB /1 = 84, APB1 /2 = 42 (must be <= 42), APB2 /1 = 84.
     * FLASH_LATENCY_2 is applied here, before the clock is raised. */
    clk.ClockType      = RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_SYSCLK
                       | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    /* After this, SystemCoreClock == 84000000. Anything that latched a timing
     * value off the old 16 MHz (e.g. a cached SystemCoreClock, UART baud, or a
     * TIM prescaler computed earlier) must be re-derived from 84 MHz. */
}
