/**
 * @file    bms_app.c
 * @brief   Application layer of the BMS HV master (cooperative scheduler).
 *
 *  task                      period            notes
 *  ------------------------  ----------------  ---------------------------------
 *  bms_afe_service           every loop        init / recovery of the chain
 *  bms_afe_cycle_start/poll  BMS_MEAS_PERIOD   cells + VSUM/VBATT, GPIO every N;
 *                                              non blocking, one slave per loop pass
 *  bms_monitor_update        10 ms             thresholds, persistence, AMS
 *  bms_can_service           10/20/100/1000ms  HVCB telemetry
 *  bms_log_service           every loop        UART drain, status every 1 s
 *  LEDs                      10 ms             see below
 *  IWDG                      10 ms             also kicked inside DelayMs()
 *
 * LEDs: STAT1 heartbeat 1 Hz | STAT2 on = chain ready, fast blink = initialising
 *       WARN on = a condition is present (not yet latched) or data not valid
 *       ERR  on = AMS error latched
 */
#include "bms_app.h"

#include "bms_afe.h"
#include "bms_can.h"
#include "bms_log.h"
#include "bms_monitor.h"
#include "main.h"

#ifndef BMS_GIT_REV
#define BMS_GIT_REV "local"
#endif

#define APP_MONITOR_PERIOD_MS 10U
#define APP_HOUSEKEEPING_MS   10U /* LEDs + watchdog */

static uint32_t reset_flags;
static uint8_t iwdg_running;
static uint32_t next_meas_ms, next_monitor_ms, next_housekeeping_ms;
static uint32_t meas_count;
static uint8_t prev_ready;

/* ------------------------------- watchdog -------------------------------- */

void bms_platform_watchdog_kick(void) {
    if (iwdg_running) {
        IWDG->KR = 0xAAAAU;
    }
}

static void app_iwdg_start(void) {
#if BMS_ENABLE_IWDG
    /* stop the watchdog while the core is halted by the debugger */
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    /* LSI ~32 kHz / 64 = 500 Hz -> 2 ms per count (LSI spread 17..47 kHz) */
    uint32_t reload = BMS_IWDG_TIMEOUT_MS / 2U;
    if (reload > 0xFFFU) {
        reload = 0xFFFU;
    }
    IWDG->KR  = 0xCCCCU; /* start (turns on the LSI) */
    IWDG->KR  = 0x5555U; /* unlock PR/RLR */
    IWDG->PR  = 4U;      /* /64 */
    IWDG->RLR = reload - 1U;
    uint32_t t0 = HAL_GetTick();
    while (IWDG->SR != 0U && (HAL_GetTick() - t0) < 100U) {
    }
    IWDG->KR     = 0xAAAAU;
    iwdg_running = 1;
#endif
}

/* --------------------------------- LEDs ---------------------------------- */

static void app_leds(uint32_t now) {
    const bms_afe_t *afe   = bms_afe_get();
    const bms_status_t *st = bms_monitor_get();

    HAL_GPIO_WritePin(STAT1_LED_GPIO_OUT_GPIO_Port, STAT1_LED_GPIO_OUT_Pin, ((now / 500U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (afe->ready) {
        Stat2_LED_On();
    } else {
        HAL_GPIO_WritePin(STAT2_LED_GPIO_OUT_GPIO_Port, STAT2_LED_GPIO_OUT_Pin, ((now / 100U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    uint8_t warn = (st->active != 0U) || !st->cells_valid;
#if BMS_DUAL_RING
    warn |= afe->ready && !afe->ring_ok;
#endif
    if (warn) {
        Warn_LED_On();
    } else {
        Warn_LED_Off();
    }

    if (st->ams_error) {
        Err_LED_On();
    } else {
        Err_LED_Off();
    }
}

/* ------------------------------- scheduler -------------------------------- */

uint32_t bms_app_reset_flags(void) {
    return reset_flags;
}

void bms_app_init(void) {
    uint32_t now;

    reset_flags = RCC->CSR & 0xFE000000U;
    RCC->CSR |= RCC_CSR_RMVF;

    bms_log("BMS HV master fw %u.%u.%u (%s) - slaves %u, dual ring %u, meas %u ms, gpio every %u",
            BMS_FW_VERSION_MAJOR,
            BMS_FW_VERSION_MINOR,
            BMS_FW_VERSION_PATCH,
            BMS_GIT_REV,
            BMS_N_SLAVES,
            BMS_DUAL_RING,
            BMS_MEAS_PERIOD_MS,
            BMS_GPIO_EVERY_N_CYCLES);
    bms_log("reset cause:%s%s%s%s%s%s%s",
            (reset_flags & RCC_CSR_LPWRRSTF) ? " LOWPOWER" : "",
            (reset_flags & RCC_CSR_WWDGRSTF) ? " WWDG" : "",
            (reset_flags & RCC_CSR_IWDGRSTF) ? " IWDG" : "",
            (reset_flags & RCC_CSR_SFTRSTF) ? " SOFTWARE" : "",
            (reset_flags & RCC_CSR_PORRSTF) ? " POWER-ON" : "",
            (reset_flags & RCC_CSR_PINRSTF) ? " PIN" : "",
            (reset_flags & RCC_CSR_BORRSTF) ? " BROWN-OUT" : "");
    bms_log_flush(50U);

    now = HAL_GetTick();
    bms_monitor_init(now);
    if (reset_flags & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) {
        /* the previous run hung or crashed (HardFault/Error_Handler end in the watchdog):
         * never restart silently with the AMS error released, or a crash loop faster than
         * the start-up grace time would leave the pack unmonitored */
        bms_monitor_force_fault(BMS_FAULT_WATCHDOG, now);
        bms_monitor_update(now);
    }
    bms_afe_init();
    bms_can_init();
    app_iwdg_start();

    next_meas_ms         = now;
    next_monitor_ms      = now;
    next_housekeeping_ms = now;
    meas_count      = 0;
    prev_ready      = 0;
}

void bms_app_loop(void) {
    uint32_t now         = HAL_GetTick();
    const bms_afe_t *afe = bms_afe_get();

    bms_afe_service(now);

    if (afe->ready) {
        if (!prev_ready) {
            /* first cycle after an (re)initialisation: read the temperatures too */
            meas_count   = 0;
            next_meas_ms = now;
        }
        if (!bms_afe_cycle_busy() && (int32_t)(now - next_meas_ms) >= 0) {
            uint8_t with_gpio = (meas_count % BMS_GPIO_EVERY_N_CYCLES) == 0U;
            if (bms_afe_cycle_start(with_gpio, now)) {
                meas_count++;
                next_meas_ms += BMS_MEAS_PERIOD_MS;
                if ((int32_t)(now - next_meas_ms) >= (int32_t)BMS_MEAS_PERIOD_MS) {
                    next_meas_ms = now; /* overrun: resynchronise instead of bursting */
                }
            }
        }
        if (bms_afe_cycle_busy()) {
            (void)bms_afe_cycle_poll(HAL_GetTick()); /* at most one slave per loop */
        }
    }
    prev_ready = afe->ready;

    now = HAL_GetTick();
    if ((int32_t)(now - next_monitor_ms) >= 0) {
        next_monitor_ms = now + APP_MONITOR_PERIOD_MS;
        bms_monitor_update(now);
    }

    bms_can_service(now);
    bms_log_service(now);

    if ((int32_t)(now - next_housekeeping_ms) >= 0) {
        next_housekeeping_ms = now + APP_HOUSEKEEPING_MS;
        app_leds(now);
        bms_platform_watchdog_kick();
    }
}
