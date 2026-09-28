/**
 * @file    bms_monitor.c
 * @brief   Plausibility and safety checks on the AFE data, AMS error output.
 *
 * Fault timing ("time since the value was last known to be good"):
 *  - every cell / NTC keeps the time of its last sample that was in range;
 *  - a condition becomes a fault when all the samples taken since then were out
 *    of range and BMS_FAULT_TIME_*_MS have elapsed from that last good sample.
 *  The reaction time from the real onset is therefore at most
 *  BMS_FAULT_TIME_*_MS + APP monitor period (10 ms), whatever the measurement
 *  period and even if some measurement cycles are lost; a single out-of-range
 *  sample (noise, spike shorter than ~3 periods) never trips.
 *  - loss of data (cells or temperatures not refreshed) is itself a fault after
 *    BMS_FAULT_TIME_COMM_MS / BMS_FAULT_TIME_TEMP_STALE_MS.
 *
 * Once latched the AMS error output (AMS_ERROR, PC6, active high as in
 * bms_hv_fsm.c) stays asserted until the next reset of the MCU.
 */
#include "bms_monitor.h"

#include <string.h>

#include "bms_afe.h"
#include "bms_log.h"
#include "main.h"

#define N_CELLS (BMS_N_SLAVES * BMS_CELLS_PER_SLAVE)
#define N_NTC   (BMS_N_SLAVES * BMS_GPIO_PER_SLAVE)

/* per-sample condition flags */
#define C_OV   0x01U
#define C_UV   0x02U
#define C_OT   0x04U
#define C_UT   0x08U
#define C_NTC  0x10U
#define C_SEEN 0x80U /* at least one sample processed */

static bms_status_t st;
static uint32_t boot_ms;

static uint32_t cell_good_ms[N_CELLS]; /* time of the last in-range sample */
static uint8_t cell_cond[N_CELLS];
static uint32_t ntc_good_ms[N_NTC];
static uint8_t ntc_cond[N_NTC];
static uint32_t last_cell_seq[BMS_N_SLAVES], last_gpio_seq[BMS_N_SLAVES];

/* shared with bms_hv_fsm.c */
float vbattery_monitor; /* [V] */
float vbattery_sum;     /* [V] */
extern uint8_t ams_error;

static __attribute__((unused)) const char *fault_name(uint32_t f) {
    switch (f) {
        case BMS_FAULT_CELL_OV:
            return "CELL OVERVOLTAGE";
        case BMS_FAULT_CELL_UV:
            return "CELL UNDERVOLTAGE";
        case BMS_FAULT_CELL_OT:
            return "CELL OVERTEMPERATURE";
        case BMS_FAULT_CELL_UT:
            return "CELL UNDERTEMPERATURE";
        case BMS_FAULT_NTC:
            return "NTC OPEN/SHORT";
        case BMS_FAULT_COMM:
            return "SLAVE COMMUNICATION";
        case BMS_FAULT_TEMP_COMM:
            return "TEMPERATURES NOT MEASURED";
        case BMS_FAULT_WATCHDOG:
            return "WATCHDOG RESET";
        default:
            return "?";
    }
}

static void latch(uint32_t fault, uint16_t idx, int32_t value, uint32_t now) {
    if ((st.faults & fault) == 0U) {
        bms_log("FAULT %s: index %u value %ld", fault_name(fault), idx, (long)value);
    }
    if (st.faults == 0U) {
        st.first_fault       = fault;
        st.first_fault_idx   = idx;
        st.first_fault_value = value;
        st.first_fault_ms    = now;
    }
    st.faults |= fault;
}

/** elapsed time, 0 if @p since is in the future (never wraps to a huge value) */
static uint32_t elapsed(uint32_t now, uint32_t since) {
    int32_t d = (int32_t)(now - since);
    return d > 0 ? (uint32_t)d : 0U;
}

/** Records a new sample: the "good" time moves only when the sample is in range. */
static void sample(uint32_t *good_ms, uint8_t *cond, uint8_t new_cond, uint32_t t_sample) {
    if (new_cond == 0U || (*cond & C_SEEN) == 0U) {
        /* in range, or first sample ever (the persistence counts from it) */
        *good_ms = t_sample;
    }
    *cond = (uint8_t)(new_cond | C_SEEN);
}

void bms_monitor_init(uint32_t now_ms) {
    memset(&st, 0, sizeof(st));
    memset(cell_good_ms, 0, sizeof(cell_good_ms));
    memset(cell_cond, 0, sizeof(cell_cond));
    memset(ntc_good_ms, 0, sizeof(ntc_good_ms));
    memset(ntc_cond, 0, sizeof(ntc_cond));
    memset(last_cell_seq, 0, sizeof(last_cell_seq));
    memset(last_gpio_seq, 0, sizeof(last_gpio_seq));
    boot_ms          = now_ms;
    vbattery_monitor = 0.0f;
    vbattery_sum     = 0.0f;
    Reset_AMS_Error();
}

void bms_monitor_force_fault(uint32_t fault, uint32_t now_ms) {
    latch(fault, 0, 0, now_ms);
}

void bms_monitor_update(uint32_t now) {
    const bms_afe_t *afe = bms_afe_get();
    int64_t cell_acc = 0, t_acc = 0;
    uint32_t cell_n = 0, t_n = 0;
    int32_t vsum = 0, vbatt = 0;
    uint8_t all_cells = 1, all_temps = 1;

    st.active      = 0;
    st.cell_min_mv = INT32_MAX;
    st.cell_max_mv = INT32_MIN;
    st.t_min_dc    = INT16_MAX;
    st.t_max_dc    = INT16_MIN;

    for (uint8_t i = 0; i < BMS_N_SLAVES; ++i) {
        const bms_afe_slave_t *s = &afe->slave[i];

        /* ---- data freshness ---- */
        uint8_t cells_stale = (s->cell_seq == 0U) ? (elapsed(now, boot_ms) >= BMS_STARTUP_GRACE_MS)
                                                  : (elapsed(now, s->last_ok_ms) >= BMS_FAULT_TIME_COMM_MS);
        if (cells_stale) {
            st.active |= BMS_FAULT_COMM;
            latch(BMS_FAULT_COMM, i + 1U, (int32_t)elapsed(now, s->last_ok_ms), now);
        }
        if (BMS_NTC_GPIO_MASK != 0U) {
            uint8_t temps_stale = (s->gpio_seq == 0U) ? (elapsed(now, boot_ms) >= BMS_STARTUP_GRACE_MS)
                                                      : (elapsed(now, s->last_gpio_ok_ms) >= BMS_FAULT_TIME_TEMP_STALE_MS);
            if (temps_stale) {
                st.active |= BMS_FAULT_TEMP_COMM;
                latch(BMS_FAULT_TEMP_COMM, i + 1U, (int32_t)elapsed(now, s->last_gpio_ok_ms), now);
            }
        }

        /* ---- cell voltages ---- */
        if (s->cell_seq == 0U) {
            all_cells = 0;
        } else {
            uint8_t fresh    = (s->cell_seq != last_cell_seq[i]);
            last_cell_seq[i] = s->cell_seq;
            for (uint8_t c = 0; c < BMS_CELLS_PER_SLAVE; ++c) {
                uint16_t idx = (uint16_t)(i * BMS_CELLS_PER_SLAVE + c);
                int32_t mv   = bms_afe_cell_mv(s->cell_raw[c]);
                if (fresh) {
                    uint8_t cond = (uint8_t)((mv > BMS_CELL_OV_MV ? C_OV : 0U) | (mv < BMS_CELL_UV_MV ? C_UV : 0U));
                    sample(&cell_good_ms[idx], &cell_cond[idx], cond, s->last_ok_ms);
                }
                uint8_t cond = cell_cond[idx];
                if (cond & (C_OV | C_UV)) {
                    st.active |= (cond & C_OV) ? BMS_FAULT_CELL_OV : BMS_FAULT_CELL_UV;
                    if (elapsed(now, cell_good_ms[idx]) >= BMS_FAULT_TIME_VOLTAGE_MS) {
                        latch((cond & C_OV) ? BMS_FAULT_CELL_OV : BMS_FAULT_CELL_UV, idx, mv, now);
                    }
                }
                if (mv < st.cell_min_mv) {
                    st.cell_min_mv  = mv;
                    st.cell_min_idx = idx;
                }
                if (mv > st.cell_max_mv) {
                    st.cell_max_mv  = mv;
                    st.cell_max_idx = idx;
                }
                cell_acc += mv;
                cell_n++;
            }
            vsum += bms_afe_vsum_mv(s->vsum_raw);
            vbatt += bms_afe_vbatt_mv(s->vbatt_div_raw);
        }

        /* ---- temperatures ---- */
        if (s->gpio_seq == 0U) {
            all_temps = 0;
            continue;
        }
        uint8_t fresh    = (s->gpio_seq != last_gpio_seq[i]);
        last_gpio_seq[i] = s->gpio_seq;
        for (uint8_t g = 0; g < BMS_GPIO_PER_SLAVE; ++g) {
            if ((BMS_NTC_GPIO_MASK & (1U << g)) == 0U) {
                continue;
            }
            uint16_t idx = (uint16_t)(i * BMS_GPIO_PER_SLAVE + g);
            int32_t mv   = bms_afe_gpio_mv(s->gpio_raw[g]);
            int16_t t    = bms_afe_ntc_dc(s->gpio_raw[g]);
            uint8_t bad  = (mv > BMS_NTC_OPEN_MV) || (mv < BMS_NTC_SHORT_MV);
            if (fresh) {
                uint8_t cond = bad ? C_NTC : (uint8_t)((t > BMS_CELL_OT_DC ? C_OT : 0U) | (t < BMS_CELL_UT_DC ? C_UT : 0U));
                sample(&ntc_good_ms[idx], &ntc_cond[idx], cond, s->last_gpio_ok_ms);
            }
            uint8_t cond = ntc_cond[idx];
            if (cond & C_NTC) {
                all_temps = 0;
                st.active |= BMS_FAULT_NTC;
#if BMS_NTC_FAULT_IS_AMS_FAULT
                if (elapsed(now, ntc_good_ms[idx]) >= BMS_FAULT_TIME_TEMP_MS) {
                    latch(BMS_FAULT_NTC, idx, mv, now);
                }
#endif
                continue;
            }
            if (cond & (C_OT | C_UT)) {
                st.active |= (cond & C_OT) ? BMS_FAULT_CELL_OT : BMS_FAULT_CELL_UT;
                if (elapsed(now, ntc_good_ms[idx]) >= BMS_FAULT_TIME_TEMP_MS) {
                    latch((cond & C_OT) ? BMS_FAULT_CELL_OT : BMS_FAULT_CELL_UT, idx, t, now);
                }
            }
            if (t < st.t_min_dc) {
                st.t_min_dc  = t;
                st.t_min_idx = idx;
            }
            if (t > st.t_max_dc) {
                st.t_max_dc  = t;
                st.t_max_idx = idx;
            }
            t_acc += t;
            t_n++;
        }
    }

    st.cells_valid = all_cells;
    st.temps_valid = all_temps && (t_n > 0U);
    if (cell_n == 0U) {
        st.cell_min_mv = st.cell_max_mv = st.cell_mean_mv = 0;
        st.pack_mv                                        = 0;
    } else {
        st.cell_mean_mv = (int32_t)(cell_acc / (int64_t)cell_n);
        st.pack_mv      = (int32_t)cell_acc;
    }
    if (t_n == 0U) {
        st.t_min_dc = st.t_max_dc = st.t_mean_dc = 0;
    } else {
        st.t_mean_dc = (int16_t)(t_acc / (int64_t)t_n);
    }
    st.vsum_mv  = vsum;
    st.vbatt_mv = vbatt;

    vbattery_monitor = (float)st.vbatt_mv / 1000.0f;
    vbattery_sum     = (float)st.pack_mv / 1000.0f;

    if (st.faults != 0U && !st.ams_error) {
        st.ams_error = 1;
        Set_AMS_Error();
        Err_LED_On();
        bms_log("AMS ERROR asserted (faults 0x%04lX)", (unsigned long)st.faults);
    }
    if (st.ams_error) {
        Set_AMS_Error(); /* re-asserted at every update: robust against spurious writes */
        ams_error = 1;   /* keep the relay state machine informed */
    }
}

const bms_status_t *bms_monitor_get(void) {
    return &st;
}
