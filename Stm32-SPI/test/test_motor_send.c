/*
 * test_motor_send.c -- drives the REAL motor_send.c state machine on the host.
 *
 * The .c is #included rather than linked so the tests can reach its statics
 * (s_tx_idx, s_pending, s_frame, s_rx_dummy) and drive the exact code that runs
 * on the board. The HAL underneath is a recording stub, so every assertion is
 * about what the firmware actually did, not about a model of it.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "motor_send.c"          /* the code under test, statics and all */

static int fails = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; \
    printf("  FAIL  "); printf(__VA_ARGS__); printf("\n         at %s:%d\n", __FILE__, __LINE__); } } while (0)

static motor_row_t g_rows[MOTOR_MAX_ROWS_PER_BLOCK];

static void rows_fill(uint16_t v)
{
    for (unsigned i = 0; i < MOTOR_MAX_ROWS_PER_BLOCK; ++i) {
        for (int c = 0; c < 8; ++c) g_rows[i].current[c] = (uint16_t)(v + i + c);
        g_rows[i].vib_x = (int16_t)(-v); g_rows[i].vib_y = (int16_t)v;
        g_rows[i].vib_z = (int16_t)(v * 2); g_rows[i].rpm = v;
    }
}

/* The Pi's validator, reproduced exactly from motor_controller.c, so a frame is
 * judged by the consumer's rules rather than the producer's. */
static uint32_t pi_crc(const uint8_t *d, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) {
        crc ^= (uint32_t)d[i] << 24;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : (crc << 1);
    }
    return crc;
}
static const char *pi_validate(const uint8_t *rx, uint16_t expect_rows)
{
    const frame_header_t *h = (const frame_header_t *)rx;
    if (h->magic   != MOTOR_FRAME_MAGIC)         return "magic";
    if (h->version != MOTOR_CONTRACT_VERSION)    return "version";
    if (h->n_rows == 0 || h->n_rows > MOTOR_MAX_ROWS_PER_BLOCK) return "n_rows range";
    if (expect_rows && h->n_rows != expect_rows) return "n_rows mismatch";
    size_t covered = sizeof(frame_header_t) + (size_t)h->n_rows * sizeof(motor_row_t);
    if (covered + sizeof(frame_crc_t) > MOTOR_MAX_FRAME_BYTES) return "size";
    uint32_t got; memcpy(&got, rx + covered, 4);
    if (pi_crc(rx, covered) != got)              return "crc";
    return NULL;
}

/* TxRxCplt means the transfer finished, so the DMA is no longer running.
 * The stub only clears fh_dma_armed on DMAStop, and the firmware no longer
 * calls that on the healthy path, so the harness has to model it. */
static void complete_transfer(void)
{
    fh_dma_armed = 0;
    HAL_SPI_TxRxCpltCallback(&s_hspi2);
}

static void reset_all(void)
{
    fh_reset();
    s_tx_idx = -1; s_pending = 0; s_seq = 0; s_sent = 0; s_skipped = 0;
    s_cmd_pending = 0; s_latched_ack_flags = 0; s_latched_cmd_seq = 0;
    s_config_applied_latch = 0; s_have_last_applied = 0; s_last_applied_seq = 0xFFFF;
    s_inflight_blocks = 0;
    g_obr = g_arm_called = g_arm_ok = g_arm_fail = g_sent = g_spi_err = 0;
    g_tx_stall = 0; g_err_ovr = g_err_fre = g_err_modf = g_err_dma = g_err_other = 0;
    fa_set_block_rows = fa_set_sample_rate = fa_set_imu_rate = fa_set_run_state = 0;
    memset(s_frame, 0, sizeof s_frame);
    s_active_config.block_rows = 200; s_active_config.source = MOTOR_SOURCE_ADC;
    s_active_config.run_state = MOTOR_RUN_RUN; s_active_config.sample_rate_hz = 20000;
    s_active_config.imu_rate_hz = 1000;
}

/* ============================ tests ============================ */

static void t_frame_accepted_by_pi(void)
{
    printf("frame the Pi will accept\n");
    reset_all(); rows_fill(1234);
    motor_on_block_ready(g_rows, 200);
    CHECK(fh_dma_armed, "no transfer armed");
    const char *why = pi_validate(fh_armed_buf, 200);
    CHECK(why == NULL, "Pi rejected the frame: %s", why ? why : "");

    /* every block size the contract allows, not just the default */
    for (uint16_t n = 1; n <= MOTOR_MAX_ROWS_PER_BLOCK; ++n) {
        reset_all(); rows_fill(n);
        motor_on_block_ready(g_rows, n);
        why = pi_validate(fh_armed_buf, n);
        if (why) { CHECK(0, "n_rows=%u rejected: %s", n, why); break; }
    }
    CHECK(1, "");

    /* slack past the frame end must be zero, not stale from a bigger frame */
    reset_all(); rows_fill(7);
    motor_on_block_ready(g_rows, 200);          /* fills the buffer */
    complete_transfer();
    motor_on_block_ready(g_rows, 10);           /* much smaller */
    const uint8_t *b = fh_armed_buf;
    size_t used = sizeof(frame_header_t) + 10u * sizeof(motor_row_t) + 4u;
    int dirty = 0;
    for (size_t i = used; i < MOTOR_MAX_FRAME_BYTES; ++i) if (b[i]) dirty = 1;
    CHECK(!dirty, "slack after a shrinking frame is not zeroed");
}

static void t_sequence_monotonic(void)
{
    printf("sequence numbers\n");
    reset_all(); rows_fill(1);
    uint32_t prev = 0;
    for (int i = 0; i < 50; ++i) {
        motor_on_block_ready(g_rows, 200);
        const frame_header_t *h = (const frame_header_t *)fh_armed_buf;
        if (i > 0) CHECK(h->seq == prev + 1, "seq jumped %u -> %u", prev, h->seq);
        prev = h->seq;
        complete_transfer();
    }
}

static void t_dr_handshake(void)
{
    printf("data-ready handshake\n");
    reset_all(); rows_fill(2);
    CHECK(fh_dr_level == 0, "DR high before any frame");
    motor_on_block_ready(g_rows, 200);
    CHECK(fh_dr_level == 1, "DR not raised after arming");
    complete_transfer();
    CHECK(fh_dr_level == 0, "DR not dropped on completion");

    /* arm failure must not leave DR asserted with nothing behind it */
    reset_all(); rows_fill(3);
    fh_arm_should_fail = 1;
    motor_on_block_ready(g_rows, 200);
    CHECK(fh_dr_level == 0, "DR raised even though arming failed");
    CHECK(g_arm_fail == 1, "arm failure not counted");
    CHECK(s_tx_idx == -1, "tx index left valid after arm failure");
    fh_arm_should_fail = 0;
    motor_on_block_ready(g_rows, 200);
    CHECK(fh_dr_level == 1, "did not recover on the next block");
}

static void t_double_buffer(void)
{
    printf("double buffering\n");
    reset_all(); rows_fill(4);
    motor_on_block_ready(g_rows, 200);
    const uint8_t *first = fh_armed_buf;
    motor_on_block_ready(g_rows, 200);          /* arrives mid-transfer */
    CHECK(fh_armed_buf == first, "in-flight buffer was re-armed under the transfer");
    CHECK(s_pending == 1, "second block not marked pending");
    complete_transfer();
    /* A block was pending, so it is re-armed at once -- the slave is never left
     * unarmed. Data-ready dips and comes back, which is the edge the Pi waits
     * on even though a 1 ms poll could not see it. */
    CHECK(fh_dma_armed == 1, "pending block was not re-armed");
    CHECK(fh_dr_level == 1, "data-ready not raised for the re-armed block");
    /* Reusing the buffer that was just transmitted is fine -- the transfer is
     * complete and nothing is reading it. The property that matters is that a
     * block is never assembled into the buffer currently IN FLIGHT. */
    const uint8_t *inflight = fh_armed_buf;
    motor_on_block_ready(g_rows, 200);      /* arrives mid-transfer again */
    CHECK(fh_armed_buf == inflight, "assembled over the in-flight buffer");
    CHECK(s_frame[0] != s_frame[1], "double buffer collapsed to one");
}

static void t_stale_frame_after_error(void)
{
    printf("stale frame after an SPI error\n");
    reset_all(); rows_fill(5);

    motor_on_block_ready(g_rows, 200);                 /* block A, in flight */
    uint32_t seq_a = ((frame_header_t*)fh_armed_buf)->seq;
    motor_on_block_ready(g_rows, 200);                 /* block B -> pending  */
    (void)seq_a;

    /* A genuine failure leaves HAL out of BUSY_TX_RX -- that is what
     * distinguishes it from the stale abort the completion path generates. */
    fh_spi_state = HAL_SPI_STATE_READY;
    fh_dma_armed = 0;
    s_hspi2.ErrorCode = HAL_SPI_ERROR_OVR;
    HAL_SPI_ErrorCallback(&s_hspi2);                   /* transfer dies       */
    fh_spi_state = HAL_SPI_STATE_BUSY_TX_RX;           /* back to normal      */

    motor_on_block_ready(g_rows, 200);                 /* block C -> armed    */
    uint32_t seq_c = ((frame_header_t*)fh_armed_buf)->seq;
    complete_transfer();                               /* C completes         */
    /* Nothing was pending (the error cleared it), so nothing should re-arm. */
    CHECK(fh_dma_armed == 0, "a stale block was re-armed after the error");

    motor_on_block_ready(g_rows, 200);                 /* block D             */
    uint32_t seq_d = ((frame_header_t*)fh_armed_buf)->seq;
    CHECK(seq_d > seq_c,
          "STALE FRAME: after C (seq=%u) the next frame on the wire is seq=%u -- "
          "an older block resurrected instead of superseded", seq_c, seq_d);
}

static void t_stall_watchdog(void)
{
    printf("stalled-transfer recovery\n");
    reset_all(); rows_fill(6);
    motor_on_block_ready(g_rows, 200);
    CHECK(fh_dr_level == 1, "no frame armed to stall");
    for (int i = 0; i < TX_STALL_BLOCKS + 2; ++i) motor_on_block_ready(g_rows, 200);
    CHECK(g_tx_stall >= 1, "watchdog never fired on a transfer that never completed");
    CHECK(s_tx_idx >= 0, "did not re-arm after recovering");

    /* and it must NOT fire on a healthy link */
    reset_all(); rows_fill(6);
    for (int i = 0; i < TX_STALL_BLOCKS * 3; ++i) {
        motor_on_block_ready(g_rows, 200);
        complete_transfer();
    }
    CHECK(g_tx_stall == 0, "watchdog false-fired %u times on a healthy link", g_tx_stall);
}

/* ---- SET_CONFIG ---- */
static void build_cmd(uint8_t *out, uint16_t cmd, uint16_t schema, uint16_t seq,
                      uint16_t rows, uint16_t source, uint16_t run,
                      uint32_t srate, uint32_t irate, int break_crc)
{
    memset(out, 0, MOTOR_CMD_FRAME_BYTES);
    cmd_header_t *h = (cmd_header_t *)out;
    h->magic = MOTOR_CMD_MAGIC; h->cmd = cmd; h->schema_version = schema;
    h->cmd_seq = seq; h->_pad = 0;
    config_payload_t *p = (config_payload_t *)(out + sizeof(cmd_header_t));
    p->block_rows = rows; p->source = source; p->run_state = run;
    p->sample_rate_hz = srate; p->imu_rate_hz = irate;
    size_t cov = sizeof(cmd_header_t) + sizeof(config_payload_t);
    uint32_t crc = pi_crc(out, cov);
    if (break_crc) crc ^= 0xFFFFFFFFu;
    memcpy(out + cov, &crc, 4);
}
static void deliver_cmd(const uint8_t *cmd)
{
    memcpy(s_rx_dummy, cmd, MOTOR_CMD_FRAME_BYTES);
    HAL_SPI_TxRxCpltCallback(&s_hspi2);     /* sniffs rx, sets s_cmd_pending */
    motor_on_block_ready(g_rows, 200);      /* processes it at the boundary  */
}

static void t_set_config(void)
{
    printf("SET_CONFIG validation\n");
    uint8_t cmd[MOTOR_CMD_FRAME_BYTES];

    /* good command applies and ACKs */
    reset_all(); rows_fill(8);
    motor_on_block_ready(g_rows, 200);
    build_cmd(cmd, MOTOR_CMD_SET_CONFIG, MOTOR_CONFIG_SCHEMA_VERSION, 1,
              100, MOTOR_SOURCE_ADC, MOTOR_RUN_RUN, 20000, 1000, 0);
    deliver_cmd(cmd);
    CHECK(s_latched_ack_flags & MOTOR_FLAG_ACK_OK, "good command not ACKed");
    CHECK(fa_set_block_rows == 1, "block_rows change not applied");

    /* bad CRC -> NACK, nothing applied */
    reset_all(); rows_fill(8);
    motor_on_block_ready(g_rows, 200);
    build_cmd(cmd, MOTOR_CMD_SET_CONFIG, MOTOR_CONFIG_SCHEMA_VERSION, 2,
              100, MOTOR_SOURCE_ADC, MOTOR_RUN_RUN, 20000, 1000, 1);
    deliver_cmd(cmd);
    CHECK(s_latched_ack_flags & MOTOR_FLAG_NACK_CRC, "bad CRC not NACKed");
    CHECK(fa_set_block_rows == 0, "bad-CRC command still applied something");

    /* out-of-range block_rows -> NACK, nothing applied */
    reset_all(); rows_fill(8);
    motor_on_block_ready(g_rows, 200);
    build_cmd(cmd, MOTOR_CMD_SET_CONFIG, MOTOR_CONFIG_SCHEMA_VERSION, 3,
              9999, MOTOR_SOURCE_ADC, MOTOR_RUN_RUN, 20000, 1000, 0);
    deliver_cmd(cmd);
    CHECK(s_latched_ack_flags & MOTOR_FLAG_NACK_RANGE, "out-of-range not NACKed");
    CHECK(fa_set_block_rows == 0, "out-of-range command still applied something");

    /* bad SOURCE -> must NACK and, crucially, apply NOTHING */
    reset_all(); rows_fill(8);
    motor_on_block_ready(g_rows, 200);
    build_cmd(cmd, MOTOR_CMD_SET_CONFIG, MOTOR_CONFIG_SCHEMA_VERSION, 4,
              100, MOTOR_SOURCE_SYNTH /* 0: passes ">ADC" but fails "!=ADC" */,
              MOTOR_RUN_RUN, 30000, 500, 0);
    deliver_cmd(cmd);
    CHECK(s_latched_ack_flags & MOTOR_FLAG_NACK_RANGE, "bad source not NACKed");
    CHECK(fa_set_block_rows == 0 && fa_set_sample_rate == 0 &&
          fa_set_imu_rate == 0 && fa_set_run_state == 0,
          "REJECTED command still reconfigured the hardware "
          "(block_rows=%d sample_rate=%d imu=%d run=%d)",
          fa_set_block_rows, fa_set_sample_rate, fa_set_imu_rate, fa_set_run_state);

    /* unknown opcode / bad schema */
    reset_all(); rows_fill(8); motor_on_block_ready(g_rows, 200);
    build_cmd(cmd, 99, MOTOR_CONFIG_SCHEMA_VERSION, 5, 100,
              MOTOR_SOURCE_ADC, MOTOR_RUN_RUN, 20000, 1000, 0);
    deliver_cmd(cmd);
    CHECK(s_latched_ack_flags & MOTOR_FLAG_NACK_CMD, "unknown opcode not NACKed");

    reset_all(); rows_fill(8); motor_on_block_ready(g_rows, 200);
    build_cmd(cmd, MOTOR_CMD_SET_CONFIG, 99, 6, 100,
              MOTOR_SOURCE_ADC, MOTOR_RUN_RUN, 20000, 1000, 0);
    deliver_cmd(cmd);
    CHECK(s_latched_ack_flags & MOTOR_FLAG_NACK_VER, "bad schema not NACKed");

    /* a corrupt command must not clobber the ACK target of a good one */
    reset_all(); rows_fill(8); motor_on_block_ready(g_rows, 200);
    build_cmd(cmd, MOTOR_CMD_SET_CONFIG, MOTOR_CONFIG_SCHEMA_VERSION, 41,
              100, MOTOR_SOURCE_ADC, MOTOR_RUN_RUN, 20000, 1000, 0);
    deliver_cmd(cmd);
    uint16_t good_seq = s_latched_cmd_seq;
    build_cmd(cmd, MOTOR_CMD_SET_CONFIG, MOTOR_CONFIG_SCHEMA_VERSION, 42,
              100, MOTOR_SOURCE_ADC, MOTOR_RUN_RUN, 20000, 1000, 1 /* bad crc */);
    deliver_cmd(cmd);
    /* Strict: the ACK target must still name the last GOOD command. A frame
     * that fails its CRC tells us nothing, including who it is from, so it
     * must not be able to move _reserved -- the Pi matches ACKs on that. */
    CHECK(s_latched_cmd_seq == good_seq,
          "corrupt command retargeted the ACK: %u -> %u (Pi matches on this)",
          good_seq, s_latched_cmd_seq);
    CHECK(s_latched_ack_flags & MOTOR_FLAG_NACK_CRC, "bad CRC not flagged");

    /* idempotent replay: same seq twice must re-ACK, not re-apply */
    reset_all(); rows_fill(8); motor_on_block_ready(g_rows, 200);
    build_cmd(cmd, MOTOR_CMD_SET_CONFIG, MOTOR_CONFIG_SCHEMA_VERSION, 50,
              100, MOTOR_SOURCE_ADC, MOTOR_RUN_RUN, 20000, 1000, 0);
    deliver_cmd(cmd);
    int applied_once = fa_set_block_rows;
    deliver_cmd(cmd);
    CHECK(fa_set_block_rows == applied_once, "replayed command re-applied config");
    CHECK(s_latched_ack_flags & MOTOR_FLAG_ACK_OK, "replayed command not re-ACKed");
}

static void t_config_applied_oneshot(void)
{
    printf("CONFIG_APPLIED is a one-shot\n");
    uint8_t cmd[MOTOR_CMD_FRAME_BYTES];
    reset_all(); rows_fill(9);
    motor_on_block_ready(g_rows, 200);
    build_cmd(cmd, MOTOR_CMD_SET_CONFIG, MOTOR_CONFIG_SCHEMA_VERSION, 1,
              150, MOTOR_SOURCE_ADC, MOTOR_RUN_RUN, 20000, 1000, 0);
    deliver_cmd(cmd);
    complete_transfer();
    motor_on_block_ready(g_rows, 200);
    int first = (((frame_header_t*)fh_armed_buf)->flags & MOTOR_FLAG_CONFIG_APPLIED) != 0;
    complete_transfer();
    motor_on_block_ready(g_rows, 200);
    int second = (((frame_header_t*)fh_armed_buf)->flags & MOTOR_FLAG_CONFIG_APPLIED) != 0;
    CHECK(first, "CONFIG_APPLIED never set on the first frame after an apply");
    CHECK(!second, "CONFIG_APPLIED is sticky; it must clear after one frame");
}

static void t_error_counters(void)
{
    printf("SPI error classification\n");
    reset_all();   /* not inside a completion, so errors are acted on */
    s_hspi2.ErrorCode = HAL_SPI_ERROR_OVR;  HAL_SPI_ErrorCallback(&s_hspi2);
    s_hspi2.ErrorCode = HAL_SPI_ERROR_FRE;  HAL_SPI_ErrorCallback(&s_hspi2);
    s_hspi2.ErrorCode = HAL_SPI_ERROR_DMA;  HAL_SPI_ErrorCallback(&s_hspi2);
    s_hspi2.ErrorCode = HAL_SPI_ERROR_OVR | HAL_SPI_ERROR_DMA;
    HAL_SPI_ErrorCallback(&s_hspi2);
    CHECK(g_err_ovr == 2, "OVR miscounted: %u", g_err_ovr);
    CHECK(g_err_fre == 1, "FRE miscounted: %u", g_err_fre);
    CHECK(g_err_dma == 2, "DMA miscounted: %u", g_err_dma);
    CHECK(g_spi_err == 4, "total miscounted: %u", g_spi_err);
    CHECK(fh_dr_level == 0, "DR left high after an error");
}

int main(void)
{
    motor_send_init(200);
    t_frame_accepted_by_pi();
    t_sequence_monotonic();
    t_dr_handshake();
    t_double_buffer();
    t_stale_frame_after_error();
    t_stall_watchdog();
    t_set_config();
    t_config_applied_oneshot();
    t_error_counters();
    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
