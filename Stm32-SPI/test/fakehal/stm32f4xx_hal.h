/* Minimal STM32 HAL stand-in: enough for motor_send.c to compile and run on the
 * host, with every HAL call recorded so the tests can assert on the sequence.
 * Nothing here emulates hardware -- the point is to drive the real state
 * machine and see what it does. */
#ifndef FAKE_HAL_H
#define FAKE_HAL_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_SPI_STATE_RESET = 0, HAL_SPI_STATE_READY,
               HAL_SPI_STATE_BUSY_TX_RX } HAL_SPI_StateTypeDef;

#define GPIO_PIN_0 (1u<<0)
#define GPIO_PIN_12 (1u<<12)
#define GPIO_PIN_13 (1u<<13)
#define GPIO_PIN_14 (1u<<14)
#define GPIO_PIN_15 (1u<<15)
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET   1
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_AF_PP 2
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_HIGH 3
#define GPIO_SPEED_FREQ_VERY_HIGH 4
#define GPIO_AF5_SPI2 5

typedef struct { uint32_t IDR; } GPIO_TypeDef;
extern GPIO_TypeDef g_gpiob_regs;
#define GPIOB (&g_gpiob_regs)
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;

typedef struct { uint32_t SR, DR, CR1; } SPI_Regs;
extern SPI_Regs g_spi2_regs;
#define SPI2 (&g_spi2_regs)
#define SPI_SR_OVR (1u<<6)
#define SPI_SR_FRE (1u<<8)
#define SPI_CR1_SPE (1u<<6)

#define SPI_MODE_SLAVE 0
#define SPI_DIRECTION_2LINES 0
#define SPI_DATASIZE_8BIT 0
#define SPI_POLARITY_LOW 0
#define SPI_PHASE_1EDGE 0
#define SPI_NSS_HARD_INPUT 0
#define SPI_FIRSTBIT_MSB 0
#define SPI_TIMODE_DISABLE 0
#define SPI_CRCCALCULATION_DISABLE 0

typedef struct {
    uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS,
             FirstBit, TIMode, CRCCalculation;
} SPI_InitTypeDef;
typedef struct { SPI_Regs *Instance; SPI_InitTypeDef Init;
                 void *hdmatx, *hdmarx; uint32_t ErrorCode; } SPI_HandleTypeDef;

#define DMA_CHANNEL_0 0
#define DMA_MEMORY_TO_PERIPH 0
#define DMA_PERIPH_TO_MEMORY 1
#define DMA_PINC_DISABLE 0
#define DMA_MINC_ENABLE 1
#define DMA_MINC_DISABLE 0
#define DMA_PDATAALIGN_BYTE 0
#define DMA_MDATAALIGN_BYTE 0
#define DMA_NORMAL 0
#define DMA_PRIORITY_HIGH 0
#define DMA_FIFOMODE_DISABLE 0
#define DMA_FIFOMODE_ENABLE 1
#define DMA_FIFO_THRESHOLD_FULL 3
#define DMA_MBURST_SINGLE 0
#define DMA_PBURST_SINGLE 0
#define DMA1_Stream3 ((void*)3)
#define DMA1_Stream4 ((void*)4)
#define DMA1_Stream3_IRQn 3
#define DMA1_Stream4_IRQn 4
#define SPI2_IRQn 36
#define HAL_SPI_ERROR_MODF 1u
#define HAL_SPI_ERROR_OVR  4u
#define HAL_SPI_ERROR_FRE  8u
#define HAL_SPI_ERROR_DMA  16u

typedef struct { void *Instance; struct { uint32_t Channel, Direction, PeriphInc,
    MemInc, PeriphDataAlignment, MemDataAlignment, Mode, Priority, FIFOMode,
    FIFOThreshold, MemBurst, PeriphBurst; } Init; } DMA_HandleTypeDef;

#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA1_CLK_ENABLE()  ((void)0)
#define __HAL_RCC_SPI2_CLK_ENABLE()  ((void)0)
#define __HAL_LINKDMA(h,f,d) ((h)->f = &(d))
#define __NOP() ((void)0)
#define _Alignas(x)

/* ---- recorded behaviour the tests assert on ---- */
extern int      fh_dr_level;        /* data-ready pin state */
extern int      fh_dma_armed;       /* is a transfer armed */
extern const uint8_t *fh_armed_buf; /* which buffer was armed */
extern int      fh_arm_should_fail;
extern int      fh_dmastop_calls;
extern HAL_SPI_StateTypeDef fh_spi_state;
void fh_reset(void);

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef*, uint8_t*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef*);
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef*);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef*);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef*);
void HAL_GPIO_Init(GPIO_TypeDef*, GPIO_InitTypeDef*);
void HAL_GPIO_WritePin(GPIO_TypeDef*, uint32_t, int);
void HAL_NVIC_SetPriority(int, int, int);
void HAL_DMA_IRQHandler(void*);
void HAL_SPI_IRQHandler(void*);
void HAL_NVIC_EnableIRQ(int);
uint32_t HAL_GetTick(void);
void Error_Handler(void);
#endif
