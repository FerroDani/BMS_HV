/**
 * @file    bms_log.c
 * @brief   Debug log on USART3 (PB10 TX, 115200 8N1) and periodic status report.
 *
 * The log never blocks the application: lines are copied into a ring buffer
 * that bms_log_service() drains by polling TXE (no interrupt, no DMA needed).
 * At 115200 baud the UART moves ~11.5 kB/s, the periodic report of 11 slaves
 * is ~1.6 kB/s. When the buffer is full the line is dropped and counted.
 */
#include "bms_log.h"

#if BMS_ENABLE_LOG

#include <stdarg.h>
#include <stdio.h>

#include "bms_afe.h"
#include "bms_monitor.h"
#include "main.h"
#include "usart.h"

#define LOG_LINE_MAX 240U
#define LOG_BUF_SIZE 4096U /* power of two */

static char log_buf[LOG_BUF_SIZE];
static volatile uint32_t log_head, log_tail; /* head: write index, tail: read index */
static uint32_t log_dropped;

static uint32_t log_free(void) {
    return LOG_BUF_SIZE - 1U - ((log_head - log_tail) & (LOG_BUF_SIZE - 1U));
}

static void log_push(const char *s, uint32_t len) {
    if (len > log_free()) {
        log_dropped++;
        return;
    }
    for (uint32_t i = 0; i < len; ++i) {
        log_buf[log_head] = s[i];
        log_head          = (log_head + 1U) & (LOG_BUF_SIZE - 1U);
    }
}

/** Moves bytes from the ring buffer to the UART while TXE is set (never waits). */
static void log_drain(void) {
    USART_TypeDef *u = huart3.Instance;
    while (log_tail != log_head && (u->SR & USART_SR_TXE)) {
        u->DR    = (uint8_t)log_buf[log_tail];
        log_tail = (log_tail + 1U) & (LOG_BUF_SIZE - 1U);
    }
}

void bms_log_flush(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while (log_tail != log_head && (HAL_GetTick() - t0) < timeout_ms) {
        log_drain();
    }
}

void bms_log(const char *fmt, ...) {
    char line[LOG_LINE_MAX];
    uint32_t t = HAL_GetTick();
    int n = snprintf(line, sizeof(line), "[%6lu.%03lu] ", (unsigned long)(t / 1000U), (unsigned long)(t % 1000U));
    va_list ap;

    va_start(ap, fmt);
    n += vsnprintf(line + n, sizeof(line) - (size_t)n - 2U, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof(line) - 3) {
        n = (int)sizeof(line) - 3;
    }
    line[n++] = '\r';
    line[n++] = '\n';
    log_push(line, (uint32_t)n);
    log_drain();
}

uint32_t bms_log_dropped(void) {
    return log_dropped;
}

static void log_status(void) {
    const bms_afe_t *afe   = bms_afe_get();
    const bms_status_t *st = bms_monitor_get();
    char line[LOG_LINE_MAX];
    int n;

    bms_log("STATUS afe=%s ready=%u init=%lu/%lu cycles=%lu cycle=%lums ams=%u faults=0x%02lX active=0x%02lX "
            "pack=%ldmV vbatt=%ldmV min=%ldmV max=%ldmV Tmin=%d Tmax=%d",
            bms_afe_state_name(afe->state),
            afe->ready,
            (unsigned long)afe->init_ok,
            (unsigned long)afe->init_attempts,
            (unsigned long)afe->cycles,
            (unsigned long)afe->last_cycle_ms,
            st->ams_error,
            (unsigned long)st->faults,
            (unsigned long)st->active,
            (long)st->pack_mv,
            (long)st->vbatt_mv,
            (long)st->cell_min_mv,
            (long)st->cell_max_mv,
            st->t_min_dc,
            st->t_max_dc);

    for (uint8_t i = 0; i < BMS_N_SLAVES; ++i) {
        const bms_afe_slave_t *s = &afe->slave[i];
        n = snprintf(line,
                     sizeof(line),
                     "S%02u %c fail=%lu V[mV]:",
                     i + 1U,
                     s->path ? 'L' : 'H',
                     (unsigned long)s->total_fail);
        for (uint8_t c = 0; c < BMS_CELLS_PER_SLAVE && n < (int)sizeof(line) - 8; ++c) {
            n += snprintf(line + n, sizeof(line) - (size_t)n, " %ld", (long)bms_afe_cell_mv(s->cell_raw[c]));
        }
        n += snprintf(line + n,
                      sizeof(line) - (size_t)n,
                      " sum=%ld vb=%ld T[0.1C]:",
                      (long)bms_afe_vsum_mv(s->vsum_raw),
                      (long)bms_afe_vbatt_mv(s->vbatt_div_raw));
        for (uint8_t g = 0; g < BMS_GPIO_PER_SLAVE && n < (int)sizeof(line) - 8; ++g) {
            if (BMS_NTC_GPIO_MASK & (1U << g)) {
                n += snprintf(line + n, sizeof(line) - (size_t)n, " %d", bms_afe_ntc_dc(s->gpio_raw[g]));
            }
        }
        bms_log("%s", line);
    }
}

void bms_log_service(uint32_t now_ms) {
    static uint32_t last = 0;
    log_drain();
    if (now_ms - last >= BMS_LOG_PERIOD_MS) {
        last = now_ms;
        log_status();
    }
}

#endif /* BMS_ENABLE_LOG */
