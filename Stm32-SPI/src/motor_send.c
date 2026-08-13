/*
 * motor_send.c  (instrumented + SET_CONFIG/ACK)
 * ----------------------------------------------------------------------------
 * STM32 send side. Implements motor_on_block_ready(): each block becomes a wire
 * frame [ frame_header | rows | CRC ] in a double buffer, shipped to the Pi as an
 * SPI2 slave (full-duplex DMA) with a data-ready GPIO handshake.
 *
 *   SPI2 slave:  PB12 NSS, PB13 SCK, PB14 MISO, PB15 MOSI  (AF5), mode 0, MSB-first
 *   data-ready:  PB0 (output, high = frame waiting)
 *   DMA1 Stream4 Ch0 = SPI2_TX, DMA1 Stream3 Ch0 = SPI2_RX (carries Pi -> STM cmds)
 *
 * SET_CONFIG / ACK protocol (Pi -> STM, ACK in next outbound frame header):
 *
 *   1. After every TxRxCplt, sniff the first 4 bytes of s_rx_dummy. If they
 *      look like MOTOR_CMD_MAGIC, copy MOTOR_CMD_FRAME_BYTES into a small
 *      pending buffer and set s_cmd_pending. CRC validation is deferred.
 *
 *   2. motor_on_block_ready first assembles & arms the CURRENT frame using
 *      the CURRENT config (so the in-flight transfer never sees a partial
 *      update). THEN it processes the pending command:
 *         - bad CRC / unknown cmd / unsupported schema / out-of-range value
 *           -> NACK bits latched, no apply.
 *         - duplicate cmd_seq (Pi re-sent because it missed the ACK)
 *           -> re-latch ACK, no re-apply.
 *         - valid SET_CONFIG -> apply now (synth + motor_send update their
 *           block_rows), latch ACK_OK + CONFIG_APPLIED for the NEXT frame.
 *
 *   3. assemble_frame stamps frame_header_t.flags with the latched ACK/NACK
 *      bits and writes the low 16 bits of the most recent cmd_seq into
 *      _reserved. CONFIG_APPLIED is a one-shot: cleared after one frame.
 *      Everything else is sticky until a new command arrives.
 * ----------------------------------------------------------------------------
 */
#include "stm32f4xx_hal.h"
#include <string.h>
#include "motor_wire.h"
#include "motor_source.h"     /* motor_on_block_ready() -- we implement it */
#include "motor_send.h"
#include "motor_acquire.h"    /* runtime reconfig of the real-sensor source */

extern void Error_Handler(void);

#ifndef MOTOR_DEFAULT_BLOCK_ROWS
#define MOTOR_DEFAULT_BLOCK_ROWS  200u
#endif

/* data-ready line */
#define DR_PORT            GPIOB
#define DR_PIN             GPIO_PIN_0

#ifndef MOTOR_DR_ASSERT_DELAY_CYCLES
#define MOTOR_DR_ASSERT_DELAY_CYCLES  1000u
#endif

/* ---- diagnostic counters (read by main.c) -------------------------------- */
volatile uint32_t g_obr        = 0;
volatile uint32_t g_arm_called = 0;
volatile uint32_t g_arm_ok     = 0;
volatile uint32_t g_arm_fail   = 0;
volatile uint32_t g_sent       = 0;
volatile uint32_t g_spi_err    = 0;
volatile uint32_t g_cmd_seen   = 0;   /* commands sniffed in rx (post-CRC TBD)  */
volatile uint32_t g_cmd_ok     = 0;   /* SET_CONFIG applied                     */
volatile uint32_t g_cmd_nack   = 0;   /* SET_CONFIG rejected                    */

/* ---- state --------------------------------------------------------------- */
static SPI_HandleTypeDef s_hspi2;
static DMA_HandleTypeDef s_hdma_tx;
static DMA_HandleTypeDef s_hdma_rx;

static uint16_t s_block_rows = MOTOR_DEFAULT_BLOCK_ROWS;
static uint16_t s_frame_len  = 0;
static uint32_t s_seq        = 0;

/* The Pi clocks MOTOR_MAX_FRAME_BYTES every transfer (worst-case sizing).
 * We size BOTH the tx frames and the rx buffer to MAX. Only the first
 * s_frame_len bytes are meaningful in tx; everything beyond is whatever
 * the buffer happened to contain (the Pi ignores it anyway, frame size
 * is in the header).                                                       */
static _Alignas(8) uint8_t s_frame[2][MOTOR_MAX_FRAME_BYTES];
static _Alignas(8) uint8_t s_rx_dummy[MOTOR_MAX_FRAME_BYTES];

static volatile int s_tx_idx  = -1;
static volatile int s_pending = 0;
static volatile uint32_t s_sent    = 0;
static volatile uint32_t s_skipped = 0;

/* ---- command/ACK state --------------------------------------------------- */
static volatile uint8_t s_cmd_pending = 0;        /* set by TxRxCplt, cleared at block boundary */
static uint8_t s_pending_cmd[MOTOR_CMD_FRAME_BYTES];

static uint16_t s_latched_ack_flags    = 0;       /* MOTOR_FLAG_ACK_* / NACK_*  */
static uint16_t s_latched_cmd_seq      = 0;       /* low 16 bits of last cmd    */
static uint8_t  s_config_applied_latch = 0;       /* one-shot CONFIG_APPLIED    */
static uint16_t s_last_applied_seq     = 0xFFFFu; /* sentinel: nothing applied  */
static uint8_t  s_have_last_applied    = 0;

/* Currently-applied STM-tier config. Initialised in motor_send_init() to the
 * compile-time defaults so process_pending_cmd can diff the incoming payload
 * against it and skip HAL-touching set functions when nothing has changed. */
static config_payload_t s_active_config;

/* ---- CRC (table-driven CRC-32/MPEG-2) ------------------------------------ */
static uint32_t s_crc_table[256];

static void crc32_init(void)
{
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i << 24;
        for (int j = 0; j < 8; ++j)
            c = (c & 0x80000000u) ? (c << 1) ^ 0x04C11DB7u : (c << 1);
        s_crc_table[i] = c;
    }
}

static uint32_t crc32_mpeg2(const uint8_t *d, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i)
        crc = (crc << 8) ^ s_crc_table[((crc >> 24) ^ d[i]) & 0xFFu];
    return crc;
}

/* ---- frame assembly ------------------------------------------------------ */
static void assemble_frame(uint8_t *buf, const motor_row_t *rows_in, uint16_t n_rows)
{
    uint32_t seq = s_seq++;

    frame_header_t *h = (frame_header_t *)buf;
    h->magic     = MOTOR_FRAME_MAGIC;
    h->seq       = seq;
    h->timestamp = (uint64_t)HAL_GetTick() * 1000ull;
    h->version   = MOTOR_CONTRACT_VERSION;
    h->n_rows    = n_rows;

    /* ACK bits: latched (sticky until next command) + CONFIG_APPLIED one-shot. */
    uint16_t f = s_latched_ack_flags;
    if (s_config_applied_latch) {
        f |= MOTOR_FLAG_CONFIG_APPLIED;
        s_config_applied_latch = 0;
    }
    h->flags     = f;
    h->_reserved = s_latched_cmd_seq;

    /* Rows are already fully composed by the producer (motor_acquire);
     * just copy them in.                                                    */
    motor_row_t *dst = (motor_row_t *)(buf + sizeof(frame_header_t));
    memcpy(dst, rows_in, (size_t)n_rows * sizeof(motor_row_t));

    size_t covered = sizeof(frame_header_t) + (size_t)n_rows * sizeof(motor_row_t);
    uint32_t crc = crc32_mpeg2(buf, covered);
    memcpy(buf + covered, &crc, sizeof crc);

    /* Zero the slack out to MAX. The Pi clocks MAX bytes per transfer (so its
     * command frame on tx can ride alongside any block size up to MAX), so we
     * must transmit MAX bytes too -- with everything past the real frame end
     * being deterministic zeros, not stale data from a previously-larger
     * frame. The Pi ignores the slack (frame size comes from h->n_rows). */
    size_t total = covered + sizeof crc;
    if (total < MOTOR_MAX_FRAME_BYTES)
        memset(buf + total, 0, MOTOR_MAX_FRAME_BYTES - total);
}

/* ---- forward declarations (defined after arm_tx but called from it on retry) */
static void spi_init(void);
static void dma_init(void);

/* ---- transmit ------------------------------------------------------------ */
static void spin_cycles(uint32_t n)
{
    volatile uint32_t i = n;
    while (i--) __NOP();
}

static void arm_tx(int idx)
{
    g_arm_called++;

    for (int attempt = 0; attempt < 50; attempt++) {
        if (attempt > 0) {
            HAL_SPI_DMAStop(&s_hspi2);

            uint32_t cs_wait = 50000u;
            while ((GPIOB->IDR & GPIO_PIN_12) == 0u && --cs_wait) {
                __NOP();
            }
        }

        /* Clear stale OVR/FRE on EVERY attempt. DMAStop (used instead of
         * the old HAL_SPI_DeInit/Abort path) does NOT clear these flags;
         * leaving them set causes the error interrupt to re-fire
         * immediately after ErrorCallback → arm_tx, locking the STM32 in
         * an infinite error loop. The old DeInit path toggled SPE which
         * cleared all flags as a side-effect, hiding this bug.           */
        if (s_hspi2.Instance->SR & SPI_SR_OVR) {
            volatile uint32_t tmp  = s_hspi2.Instance->DR;
            tmp = s_hspi2.Instance->SR;
            (void)tmp;
        }
        if (s_hspi2.Instance->SR & SPI_SR_FRE) {
            s_hspi2.Instance->CR1 &= ~SPI_CR1_SPE;
            s_hspi2.Instance->CR1 |=  SPI_CR1_SPE;
        }

        s_tx_idx = idx;

        if (HAL_SPI_TransmitReceive_DMA(&s_hspi2, s_frame[idx], s_rx_dummy,
                                        MOTOR_MAX_FRAME_BYTES) != HAL_OK) {
            s_tx_idx = -1;
            continue;
        }

        spin_cycles(MOTOR_DR_ASSERT_DELAY_CYCLES);

        uint32_t timeout = 5000u;
        while (HAL_SPI_GetState(&s_hspi2) != HAL_SPI_STATE_BUSY_TX_RX
               && --timeout) {
            __NOP();
        }
        if (timeout == 0u) {
            HAL_SPI_DMAStop(&s_hspi2);
            s_tx_idx = -1;
            continue;
        }

        g_arm_ok++;
        HAL_GPIO_WritePin(DR_PORT, DR_PIN, GPIO_PIN_SET);
        spin_cycles(50u);
        return;
    }

    s_skipped++;
    g_arm_fail++;
}

/* ---- command processing ------------------------------------------------- */
/* All of these run from motor_on_block_ready, which is itself called from
 * the TIM2 ISR (same NVIC priority as the SPI/DMA ISRs, so nothing here
 * preempts and the TX in flight can't be touched mid-DMA). */

static void process_pending_cmd(void)
{
    /* Snapshot the pending command into local space and clear the slot
     * before we do anything else: if TxRxCplt fires a fresh command in
     * the middle of validation (it can't preempt us, but defensive code
     * is cheap), it'll just set the flag again and we'll see it next
     * block.                                                              */
    uint8_t buf[MOTOR_CMD_FRAME_BYTES];
    memcpy(buf, s_pending_cmd, sizeof buf);
    s_cmd_pending = 0;

    const cmd_header_t *ch = (const cmd_header_t *)buf;

    /* Magic was already checked in the sniff, but re-check defensively. */
    if (ch->magic != MOTOR_CMD_MAGIC) return;

    /* CRC over [cmd_header_t][config_payload_t]. Checked BEFORE latching the
     * sequence number: cmd_seq comes out of the same frame the CRC is there to
     * vouch for, so latching first meant a corrupted frame could retarget the
     * ACK at a sequence number the Pi never sent, and the Pi matches ACKs by
     * exactly that field. A frame that fails its CRC tells us nothing at all,
     * including who it claims to be from. */
    size_t covered = sizeof(cmd_header_t) + sizeof(config_payload_t);
    uint32_t got;
    memcpy(&got, buf + covered, sizeof got);
    if (crc32_mpeg2(buf, covered) != got) {
        s_latched_ack_flags = MOTOR_FLAG_ACK_NACK | MOTOR_FLAG_NACK_CRC;
        g_cmd_nack++;
        return;
    }

    s_latched_cmd_seq = ch->cmd_seq;

    /* Schema version. We only support exactly the compiled-in version for
     * now; older fw declines newer schemas instead of misinterpreting.   */
    if (ch->schema_version != MOTOR_CONFIG_SCHEMA_VERSION) {
        s_latched_ack_flags = MOTOR_FLAG_ACK_NACK | MOTOR_FLAG_NACK_VER;
        g_cmd_nack++;
        return;
    }

    switch (ch->cmd) {

    case MOTOR_CMD_PING:
        s_latched_ack_flags = MOTOR_FLAG_ACK_OK;
        g_cmd_ok++;
        return;

    case MOTOR_CMD_SET_CONFIG: {
        const config_payload_t *p =
            (const config_payload_t *)(buf + sizeof(cmd_header_t));

        /* Idempotency: if we've already applied this exact cmd_seq, just
         * re-affirm the ACK (handles Pi-side retries when its ACK was
         * missed in transit). */
        if (s_have_last_applied && ch->cmd_seq == s_last_applied_seq) {
            s_latched_ack_flags = MOTOR_FLAG_ACK_OK;
            return;
        }

        /* Range check EVERY field before we touch anything.
         * sample_rate_hz and imu_rate_hz allow 0 as a "leave alone" sentinel;
         * non-zero values must fall in the valid band.                      */
        int bad_sample = (p->sample_rate_hz != 0u &&
                          (p->sample_rate_hz < 100u || p->sample_rate_hz > 100000u));
        int bad_imu    = (p->imu_rate_hz != 0u &&
                          (p->imu_rate_hz < 10u || p->imu_rate_hz > 1000u));
        /* `!=` not `>`. Only ADC is supported, and the old `>` let SOURCE_SYNTH
         * (0) through this gate -- it is not greater than ADC (1) -- to be
         * caught by a second check further down, AFTER block_rows, sample_rate,
         * imu_rate and run_state had already been pushed into the hardware. The
         * command was then NACK'd, leaving the board reconfigured by a request
         * it had just told the Pi it rejected. Catching it here restores what
         * the comment below always claimed: validate everything, then apply. */
        if (p->block_rows == 0u || p->block_rows > MOTOR_MAX_ROWS_PER_BLOCK ||
            p->source != MOTOR_SOURCE_ADC ||
            p->run_state > MOTOR_RUN_RUN ||
            bad_sample || bad_imu) {
            s_latched_ack_flags = MOTOR_FLAG_ACK_NACK | MOTOR_FLAG_NACK_RANGE;
            g_cmd_nack++;
            return;
        }

        /* Per-field diff vs the currently-applied config. Only touch HAL for
         * fields that actually changed. This is what makes a "push defaults at
         * boot" SET_CONFIG cost zero -- no HAL_TIM_Base_Init from inside the
         * TIM2 ISR, no pattern rebuild, just an ACK.                         */
        uint8_t any_change = 0;

        if (p->block_rows != s_active_config.block_rows) {
            motor_send_set_block_rows(p->block_rows);
            motor_acquire_set_block_rows(p->block_rows);
            any_change = 1;
        }
        /* synth_cycles is _reserved0 in schema v2; ignore whatever the Pi
         * happened to send.                                                 */
        if (p->sample_rate_hz != 0u &&
            p->sample_rate_hz != s_active_config.sample_rate_hz) {
            motor_acquire_set_sample_rate(p->sample_rate_hz);
            any_change = 1;
        }
        if (p->imu_rate_hz != 0u &&
            p->imu_rate_hz != s_active_config.imu_rate_hz) {
            motor_acquire_set_imu_rate(p->imu_rate_hz);
            any_change = 1;
        }
        if (p->run_state != s_active_config.run_state) {
            motor_acquire_set_run_state((uint8_t)p->run_state);
            any_change = 1;
        }
        /* (source is validated up front with the other fields, before anything
         * is applied -- see the range check above.) */

        /* Commit to the active-config record AFTER successful apply, so any
         * future diff is against what we actually programmed.                */
        s_active_config = *p;
        if (p->sample_rate_hz == 0u) {
            /* "leave alone" sentinel -- preserve for next diff */
            s_active_config.sample_rate_hz = 0u;
        }

        s_last_applied_seq  = ch->cmd_seq;
        s_have_last_applied = 1;
        s_latched_ack_flags = MOTOR_FLAG_ACK_OK;
        s_config_applied_latch = 1;     /* one-shot bit in NEXT outbound frame */
        g_cmd_ok++;
        (void)any_change;
        return;
    }

    default:
        s_latched_ack_flags = MOTOR_FLAG_ACK_NACK | MOTOR_FLAG_NACK_CMD;
        g_cmd_nack++;
        return;
    }
}

/* ---- stalled-transfer recovery ------------------------------------------
 *
 * Once arm_tx() succeeds the state machine has exactly one way out: the
 * master finishes clocking MOTOR_MAX_FRAME_BYTES, TxRxCplt fires, PB0 drops
 * and the next frame is armed. If those clocks never arrive, nothing here
 * ever runs again:
 *
 *   - TxRxCplt cannot fire, because the DMA never completes.
 *   - ErrorCallback cannot fire either -- an abandoned transfer produces no
 *     OVR/FRE, it just sits there.
 *   - s_tx_idx stays >= 0 forever, so every later motor_on_block_ready takes
 *     the `else` branch, sets s_pending and returns.
 *   - PB0 stays HIGH forever.
 *
 * That is a permanent deadlock, and it survives anything done on the Pi side:
 * the master sees data-ready asserted, reads a frame the slave is not
 * driving, gets garbage, and repeats. Only a reset of THIS board clears it,
 * which is why it looks like a Pi problem and is not one.
 *
 * The master abandoning a transfer mid-frame is routine, not exotic -- the
 * producer being restarted, spi-dwc being bounced (which motor_controller
 * does by itself on an SPI-tier config reload), or the Pi rebooting all leave
 * the slave part-way through a frame.
 *
 * So: if a transfer has been in flight for TX_STALL_BLOCKS block periods,
 * tear it down and re-arm on the next block. At the default 20 kHz / 200 rows
 * a block is 10 ms, so the budget below is ~1 s -- far longer than any real
 * transfer (a 4828-byte frame is ~5 ms at 8 MHz, and tens of ms even on a
 * badly misconfigured bus), and short enough that the link recovers on its
 * own a second after the master comes back.
 */
#ifndef TX_STALL_BLOCKS
#define TX_STALL_BLOCKS  100u
#endif

volatile uint32_t g_tx_stall = 0;      /* stalled transfers recovered */

static uint32_t s_inflight_blocks = 0; /* blocks elapsed with a tx in flight */

static void tx_recover(void)
{
    g_tx_stall++;

    /* Drop data-ready first: the master must stop starting new reads against
     * a slave that is about to be torn down. */
    HAL_GPIO_WritePin(DR_PORT, DR_PIN, GPIO_PIN_RESET);

    HAL_SPI_DMAStop(&s_hspi2);

    /* Re-arming while the master still holds CS asserted would splice us into
     * the middle of its current frame. Wait for the bus to go idle, bounded so
     * a CS stuck low cannot hang the ISR this runs from. */
    uint32_t cs_wait = 50000u;
    while ((GPIOB->IDR & GPIO_PIN_12) == 0u && --cs_wait) {
        __NOP();
    }

    /* An aborted transfer leaves OVR/FRE latched, and DMAStop does not clear
     * them -- same trap arm_tx() documents on its retry path. Left set, the
     * error interrupt re-fires the instant we re-enable. */
    if (s_hspi2.Instance->SR & SPI_SR_OVR) {
        volatile uint32_t tmp = s_hspi2.Instance->DR;
        tmp = s_hspi2.Instance->SR;
        (void)tmp;
    }
    if (s_hspi2.Instance->SR & SPI_SR_FRE) {
        s_hspi2.Instance->CR1 &= ~SPI_CR1_SPE;
        s_hspi2.Instance->CR1 |=  SPI_CR1_SPE;
    }

    s_tx_idx          = -1;
    s_pending         = 0;
    s_inflight_blocks = 0;
}

/* ---- the producer hook --------------------------------------------------- */
void motor_on_block_ready(const motor_row_t *rows, uint16_t n_rows)
{
    g_obr++;

    /* Watchdog the in-flight transfer before anything else, so a stalled one
     * is cleared in time for this block to be armed normally below. */
    if (s_tx_idx >= 0) {
        if (++s_inflight_blocks >= TX_STALL_BLOCKS)
            tx_recover();
    } else {
        s_inflight_blocks = 0;
    }

    int inflight = s_tx_idx;
    int build    = (inflight == 0) ? 1 : 0;

    /* Assemble + arm the CURRENT block first, using the CURRENT config.
     * Then process any pending command, so a SET_CONFIG only takes effect
     * for the next block. This keeps the in-flight DMA and the frame
     * header consistent.                                                  */
    assemble_frame(s_frame[build], rows, n_rows);

    if (inflight < 0) {
        arm_tx(build);
    } else {
        if (s_pending) s_skipped++;
        s_pending = 1;
    }

    if (s_cmd_pending) process_pending_cmd();
}

/* ---- SPI ISR callbacks --------------------------------------------------- */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h)
{
    if (h->Instance != SPI2) return;


    HAL_GPIO_WritePin(DR_PORT, DR_PIN, GPIO_PIN_RESET);
    s_sent++;
    g_sent++;

    /* A frame completed, so whatever is armed next starts its stall budget
     * from zero. Without this the counter carries over between transfers and
     * eventually trips tx_recover() on a perfectly healthy link. */
    s_inflight_blocks = 0;

    /* Sniff the rx for a command frame. Cheap: just compare 4 bytes. The
     * full validation (CRC, ranges) happens at the next block boundary in
     * process_pending_cmd. */
    uint32_t mag;
    memcpy(&mag, s_rx_dummy, sizeof mag);
    if (mag == MOTOR_CMD_MAGIC && !s_cmd_pending) {
        memcpy(s_pending_cmd, s_rx_dummy, MOTOR_CMD_FRAME_BYTES);
        s_cmd_pending = 1;
        g_cmd_seen++;
    }

    int just = s_tx_idx;

    /* Keep the DMAStop. It looks wrong and measures right.
     *
     * This callback is the transfer completing, so stopping the DMA here is
     * redundant on paper, and it manufactures errors: 80 HAL_SPI_ERROR_DMA per
     * second against 99 completed transfers, four in five. Removing it cleans
     * that up almost entirely -- spi_err 84/s -> 6/s, err_dma 81/s -> 3/s.
     *
     * It also makes the link worse. Matched 60-second windows, everything else
     * identical, measured at the Pi where it counts:
     *
     *     with DMAStop:     ok 95/s   drops 0/s   magic  5/s
     *     without:          ok 89/s   drops 0/s   magic 12/s
     *
     * So the aborts are benign bookkeeping and the stop is doing something real
     * -- resynchronising the peripheral between frames -- that the error
     * counter gives no credit for. Judge this by ok/magic at the Pi, never by
     * g_spi_err, which points the opposite way.
     */
    HAL_SPI_DMAStop(&s_hspi2);

    /* Re-arm immediately, so the slave is ALWAYS armed.
     *
     * This pairs with the Pi waiting on the rising edge rather than polling the
     * level (see dataready_wait). Two separate things were going wrong and each
     * needs its own half of the fix:
     *
     *   - the master could not tell one frame from the next, because the line
     *     only dipped for the ~150 us of the re-arm and a 1 ms poll never saw
     *     it. The EDGE fixes that; the dip is plenty for the GPIO resource
     *     manager's edge detector even though it is invisible to a poll.
     *
     *   - a read that started before the arm completed got stale bytes. Keeping
     *     the slave armed at all times fixes that: there is no window in which
     *     a transfer can begin against an unarmed DMA.
     *
     * Measured, not re-arming (with the Pi on edges) drove magic to 2 total and
     * held it there -- the diagnosis was right -- but cost throughput badly,
     * ok 32/s against 100 blocks/s produced, because the pending block is
     * discarded. Re-arming keeps the blocks. */
    if (s_pending) {
        s_pending = 0;
        arm_tx((just == 0) ? 1 : 0);
    } else {
        s_tx_idx = -1;
    }
}

/* Which error, specifically. g_spi_err alone says ~41/s are happening but not
 * what they are, and the two likely causes want opposite fixes:
 *   OVR -- the master clocked us while no DMA was armed (a read that landed in
 *          the gap between TxRxCplt and the next arm_tx, or on a stale-high
 *          data-ready). Timing problem.
 *   FRE -- NSS moved mid-frame. With SPI_NSS_HARD_INPUT that means the master
 *          deasserted chip-select part way through. Wiring or driver problem.
 * Diagnostic only; no behaviour change. */
volatile uint32_t g_err_ovr = 0, g_err_fre = 0, g_err_modf = 0,
                  g_err_dma = 0, g_err_other = 0;
volatile uint32_t g_err_idle    = 0;  /* arrived with nothing in flight       */
volatile uint32_t g_err_live    = 0;  /* arrived with a transfer in flight    */


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *h)
{
    if (h->Instance != SPI2) return;
    g_spi_err++;

    {
        uint32_t e = h->ErrorCode;
        if (e & HAL_SPI_ERROR_OVR)  g_err_ovr++;
        if (e & HAL_SPI_ERROR_FRE)  g_err_fre++;
        if (e & HAL_SPI_ERROR_MODF) g_err_modf++;
        if (e & HAL_SPI_ERROR_DMA)  g_err_dma++;
        if (!(e & (HAL_SPI_ERROR_OVR | HAL_SPI_ERROR_FRE |
                   HAL_SPI_ERROR_MODF | HAL_SPI_ERROR_DMA))) g_err_other++;
    }

    /* Ignore an error that arrives while a transfer is armed and running.
     *
     * HAL_SPI_DMAStop() in the completion path aborts streams that are still
     * retiring, and that abort surfaces here as HAL_SPI_ERROR_DMA -- about 80
     * times a second, four transfers in five. By the time it lands, the
     * completion callback has usually already armed the NEXT frame and raised
     * data-ready. Tearing down at that point kills a perfectly healthy
     * transfer, and if the master has already sampled data-ready high it then
     * clocks into a slave with no DMA armed. That is the OVR in the counters
     * and the bad magic the Pi reports: the error handler was causing the very
     * fault it exists to recover from.
     *
     * If SPI is BUSY_TX_RX with a live s_tx_idx then the armed transfer is
     * fine and this error belongs to the previous one. Drop it.
     *
     * It cannot mask a genuine stall: a transfer that really is wedged stops
     * completing, and the TX_STALL_BLOCKS watchdog in motor_on_block_ready
     * tears it down and re-arms within about a second. */
    /* Nothing in flight, so there is nothing to recover -- and acting anyway
     * does harm. The teardown below clears s_pending, so a stale error landing
     * between transfers silently discards the block that was queued for the
     * next one, and drops data-ready after the completion path has raised it.
     * If the master has already sampled that line high it then clocks into a
     * slave with no DMA armed, which is the OVR in the counters and the bad
     * magic at the Pi.
     *
     * These are counted separately so the split is visible rather than
     * assumed: g_err_idle vs g_err_live. */
    if (s_tx_idx < 0) {
        g_err_idle++;
        return;
    }
    g_err_live++;

    HAL_GPIO_WritePin(DR_PORT, DR_PIN, GPIO_PIN_RESET);

    /* Stop DMA but keep the peripheral enabled so the master can finish
     * the current frame without getting a FRE.  Do NOT re-arm here:
     * re-arming the same buffer creates a duplicate frame (same seq)
     * which wastes the controller's time and causes the next valid frame
     * to be skipped, inflating the drop count.  Instead, let the
     * completion callback or the next capture-completion handle re-arm
     * naturally via the normal s_tx_idx / s_pending state machine.       */
    HAL_SPI_DMAStop(&s_hspi2);
    s_tx_idx = -1;
    s_inflight_blocks = 0;   /* transfer is over; next one starts fresh */

    /* Drop the pending block too. Without this, s_pending survives the error
     * and the buffer it refers to gets armed later, out of order:
     *
     *   A in flight, B assembled and pending -> error kills A, s_tx_idx = -1
     *   -> next block C sees an idle link, assembles into A's buffer, arms it
     *   -> C completes, TxRxCplt still sees s_pending == 1 and arms B
     *
     * so B goes on the wire after C with a LOWER seq. The Pi reads that as the
     * stream going backwards. B is already lost the moment the transfer failed;
     * the next block supersedes it. */
    s_pending = 0;
}

/* ---- init ---------------------------------------------------------------- */
static void gpio_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &g);

    GPIO_InitTypeDef d = {0};
    d.Pin   = DR_PIN;
    d.Mode  = GPIO_MODE_OUTPUT_PP;
    d.Pull  = GPIO_NOPULL;
    d.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DR_PORT, &d);
    HAL_GPIO_WritePin(DR_PORT, DR_PIN, GPIO_PIN_RESET);
}

static void dma_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    s_hdma_tx.Instance                 = DMA1_Stream4;
    s_hdma_tx.Init.Channel             = DMA_CHANNEL_0;
    s_hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    s_hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    s_hdma_tx.Init.Mode                = DMA_NORMAL;
    s_hdma_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    s_hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&s_hdma_tx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&s_hspi2, hdmatx, s_hdma_tx);

    s_hdma_rx.Instance                 = DMA1_Stream3;
    s_hdma_rx.Init.Channel             = DMA_CHANNEL_0;
    s_hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    /* Was DMA_MINC_DISABLE -- but we now want to CAPTURE the Pi's tx into
     * s_rx_dummy so we can sniff command frames. Enable memory increment. */
    s_hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    s_hdma_rx.Init.Mode                = DMA_NORMAL;
    s_hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    s_hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&s_hdma_rx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&s_hspi2, hdmarx, s_hdma_rx);

    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
}

static void spi_init(void)
{
    __HAL_RCC_SPI2_CLK_ENABLE();
    s_hspi2.Instance               = SPI2;
    s_hspi2.Init.Mode              = SPI_MODE_SLAVE;
    s_hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    s_hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    s_hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    s_hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
    s_hspi2.Init.NSS               = SPI_NSS_HARD_INPUT;
    s_hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    s_hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    s_hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&s_hspi2) != HAL_OK) Error_Handler();
}

void motor_send_init(uint16_t block_rows)
{
    if (block_rows == 0 || block_rows > MOTOR_MAX_ROWS_PER_BLOCK)
        block_rows = MOTOR_DEFAULT_BLOCK_ROWS;
    s_block_rows = block_rows;
    s_frame_len  = (uint16_t)(sizeof(frame_header_t)
                 + (size_t)block_rows * sizeof(motor_row_t) + sizeof(frame_crc_t));

    /* Seed s_active_config with what motor_synth_init / motor_send_init will
     * actually have programmed. Must match the synth-side defaults in
     * motor_synth.c (SYNTH_DEFAULT_PERIOD_US, SYNTH_DEFAULT_CYCLES) so an
     * incoming SET_CONFIG with the same values is correctly recognised as a
     * no-op.                                                                 */
    s_active_config.block_rows     = block_rows;
    s_active_config._reserved0     = 0u;
    s_active_config.source         = MOTOR_SOURCE_ADC;
    s_active_config.run_state      = MOTOR_RUN_RUN;
    s_active_config.sample_rate_hz = 20000u;
    s_active_config.imu_rate_hz    = 1000u;

    crc32_init();
    gpio_init();
    dma_init();
    spi_init();
}

void motor_send_set_block_rows(uint16_t block_rows)
{
    if (block_rows == 0 || block_rows > MOTOR_MAX_ROWS_PER_BLOCK) return;
    s_block_rows = block_rows;
    s_frame_len  = (uint16_t)(sizeof(frame_header_t)
                 + (size_t)block_rows * sizeof(motor_row_t) + sizeof(frame_crc_t));
}

void motor_send_get_stats(uint32_t *frames_sent, uint32_t *frames_skipped)
{
    if (frames_sent)    *frames_sent    = s_sent;
    if (frames_skipped) *frames_skipped = s_skipped;
}

/* ---- ISR plumbing -------------------------------------------------------- */
void DMA1_Stream4_IRQHandler(void) { HAL_DMA_IRQHandler(&s_hdma_tx); }
void DMA1_Stream3_IRQHandler(void) { HAL_DMA_IRQHandler(&s_hdma_rx); }
void SPI2_IRQHandler(void)         { HAL_SPI_IRQHandler(&s_hspi2); }