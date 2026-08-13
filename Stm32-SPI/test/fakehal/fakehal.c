#include "stm32f4xx_hal.h"
GPIO_TypeDef g_gpiob_regs = { .IDR = GPIO_PIN_12 };  /* CS idle high by default */
SPI_Regs    g_spi2_regs   = {0};
int fh_dr_level = 0, fh_dma_armed = 0, fh_arm_should_fail = 0, fh_dmastop_calls = 0;
const uint8_t *fh_armed_buf = 0;
HAL_SPI_StateTypeDef fh_spi_state = HAL_SPI_STATE_BUSY_TX_RX;
void fh_reset(void){ fh_dr_level=0; fh_dma_armed=0; fh_armed_buf=0;
    fh_arm_should_fail=0; fh_dmastop_calls=0; fh_spi_state=HAL_SPI_STATE_BUSY_TX_RX;
    g_gpiob_regs.IDR = GPIO_PIN_12; g_spi2_regs.SR = 0; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef*h,uint8_t*tx,uint8_t*rx,uint16_t n){
    (void)h;(void)rx;(void)n; if(fh_arm_should_fail) return HAL_ERROR;
    fh_dma_armed=1; fh_armed_buf=tx; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef*h){ (void)h; fh_dma_armed=0; fh_dmastop_calls++; return HAL_OK; }
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef*h){ (void)h; return fh_spi_state; }
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef*h){ (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef*h){ (void)h; return HAL_OK; }
void HAL_GPIO_Init(GPIO_TypeDef*p,GPIO_InitTypeDef*g){ (void)p;(void)g; }
void HAL_GPIO_WritePin(GPIO_TypeDef*p,uint32_t pin,int v){ (void)p; if(pin==GPIO_PIN_0) fh_dr_level=v; }
void HAL_NVIC_SetPriority(int a,int b,int c){ (void)a;(void)b;(void)c; }
void HAL_NVIC_EnableIRQ(int a){ (void)a; }
static uint32_t s_tick=0;
uint32_t HAL_GetTick(void){ return ++s_tick; }
void Error_Handler(void){ }
void HAL_DMA_IRQHandler(void *h){ (void)h; }
void HAL_SPI_IRQHandler(void *h){ (void)h; }
