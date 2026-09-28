/**
 * @file    bms_afe.c
 * @brief   Management of the L9963E daisy chain (analog front end of the BMS HV).
 *
 * Datasheets: L9963E DS13636 rev 11 (DS), L9963T DS13590 rev 4 (DST).
 *
 * Initialisation (non blocking state machine, see bms_afe_service()):
 *  1. RESET_TRX : DIS released for 5 ms -> the L9963T goes to stand-by and, at
 *                 the next wake-up, latches ISOFREQ low (DST table 9).
 *  2. START_TRX : DIS low, CS/TXEN high, ISOFREQ low, 2 ms wait (T_WAKEUP 1.06 ms).
 *  3. ADDRESSING: - fast probe: a chain left awake in fast mode (e.g. MCU reset
 *                   while the slaves were running) is reset with SW_RST+GO2SLP;
 *                 - addressing procedure of DS 4.1.2.2 (fixed library);
 *                 - configuration in broadcast and read-back of every slave;
 *                 - dual ring: L9963TL switched to fast mode, ring check.
 *  4. READY     : measurements; after repeated failures -> BACKOFF.
 *  5. BACKOFF   : waits (at least the slave CommTimeout, so every slave falls
 *                 asleep and forgets the fast mode) and restarts from 1.
 */
#include "bms_afe.h"

#include <string.h>

#include "L9963E.h"
#include "L9963E_drv.h"
#include "bms_log.h"
#include "main.h"
#include "stm32_if.h"

/* C1..C8 + C12..C14 (pair C9-C10 and odd cell C11 not mounted, DS 6.10.1.3) */
#define AFE_CELL_MASK                                                                                         \
    ((uint16_t)(L9963E_CELL1 | L9963E_CELL2 | L9963E_CELL3 | L9963E_CELL4 | L9963E_CELL5 | L9963E_CELL6 | \
                L9963E_CELL7 | L9963E_CELL8 | L9963E_CELL12 | L9963E_CELL13 | L9963E_CELL14))


#define AFE_TRX_RESET_MS     5U  /* DIS released: L9963T to stand-by */
#define AFE_TRX_WAKE_MS      2U  /* > T_WAKEUP of the L9963T (1.06 ms) */
#define AFE_READ_TIMEOUT_MS  5U  /* timeout of a single register read (init, slow isoSPI) */
#define AFE_MEAS_TIMEOUT_MS  2U  /* timeout of a measurement read (fast isoSPI, round trip < 0.1 ms) */
#define AFE_PROBE_TIMEOUT_MS 3U
#define AFE_FAIL_RECOVERY    10U /* failed cycles of one slave before re-initialising the chain */
#define AFE_THRESH_LSB_UV    22784U
/* CSA_GPIO_MSK: Gpio3..9_OT_UT_MSK (bits 0..6) + ovc_norm_msk (10) + ovc_sleep_msk (11) */
#define AFE_CSA_GPIO_MSK 0x00C7FU

static L9963E_HandleTypeDef h_th; /* bottom of the ring: L9963TH, SPI3 */
#if BMS_DUAL_RING
static L9963E_HandleTypeDef h_tl; /* top of the ring: L9963TL, SPI2 */
#endif

static bms_afe_t afe;
static uint8_t afe_recovery_request;

/* state of the running measurement cycle (see bms_afe_cycle_poll) */
typedef enum {
    CYC_IDLE = 0,
    CYC_CONV,  /* waiting the end of the conversion */
    CYC_READ,  /* reading the slaves, one per poll */
    CYC_CONV2, /* dual ring just opened: second conversion from the top end */
    CYC_READ2, /* ... and read of the slaves that failed in the first pass */
} afe_cycle_phase_t;

static struct {
    afe_cycle_phase_t phase;
    uint8_t path_dead[2]; /* a slave did not answer at all on this path in this pass:
                           * the ones beyond it are unreachable too, do not wait for them */
    uint8_t with_gpio;
    uint8_t next;
    uint8_t ok_n;
    uint8_t recover;
    uint8_t use_tl;
    uint8_t n_failed;
    uint8_t failed[BMS_N_SLAVES];
    uint32_t t_start;
    uint32_t t_soc;
} cyc;

static const uint16_t afe_comm_timeout_ms[4] = {32U, 256U, 1024U, 2048U};

const char *bms_afe_state_name(bms_afe_state_t s) {
    switch (s) {
        case BMS_AFE_STATE_RESET_TRX:
            return "RESET_TRX";
        case BMS_AFE_STATE_START_TRX:
            return "START_TRX";
        case BMS_AFE_STATE_ADDRESSING:
            return "ADDRESSING";
        case BMS_AFE_STATE_READY:
            return "READY";
        case BMS_AFE_STATE_BACKOFF:
            return "BACKOFF";
        default:
            return "?";
    }
}

static void afe_set_state(bms_afe_state_t s) {
    afe.state          = s;
    afe.state_since_ms = HAL_GetTick();
}

/** 22.784 mV/LSB. @p round_up = 1 for under-voltage thresholds, so that the
 *  programmed value is never less conservative than the requested one. */
static uint8_t afe_thresh_code(int32_t mv, uint8_t round_up) {
    int64_t num  = (int64_t)mv * 1000;
    int32_t code = (int32_t)((num + (round_up ? (int64_t)AFE_THRESH_LSB_UV - 1 : 0)) / (int64_t)AFE_THRESH_LSB_UV);
    if (code < 0) {
        code = 0;
    }
    if (code > 255) {
        code = 255;
    }
    return (uint8_t)code;
}

/* ------------------------------------------------------------------------- */
/*                              link helpers                                  */
/* ------------------------------------------------------------------------- */

/** Sends a frame with a wrong CRC: it only moves bits on the isoline. */
static void afe_send_dummy(L9963E_HandleTypeDef *h) {
    uint8_t dummy[5] = {0x55, 0x55, 0x55, 0x55, 0x55};
    L9963E_DRV_TXEN_HIGH(&h->drv_handle);
    L9963E_DRV_CS_LOW(&h->drv_handle);
    (void)L9963E_DRV_SPI_TRANSMIT(&h->drv_handle, dummy, sizeof(dummy), 10);
    L9963E_DRV_CS_HIGH(&h->drv_handle);
}

/** Sets the isoSPI speed of a transceiver. The L9963T switches its TX side only
 *  after the frame latched with the new ISOFREQ (DST table 9): a dummy frame is
 *  sent so that the next real command already travels at the new speed. */
static void afe_set_speed(L9963E_HandleTypeDef *h, uint8_t fast) {
    if (fast) {
        L9963E_DRV_ISOFREQ_HIGH(&h->drv_handle);
    } else {
        L9963E_DRV_ISOFREQ_LOW(&h->drv_handle);
    }
    afe_send_dummy(h);
    DelayMs(1); /* a slow frame lasts ~130 us */
    (void)L9963E_DRV_flush_rx(&h->drv_handle);
}

static void afe_trx_standby(void) {
    L9963E_init(&h_th, L9963TH_interface, BMS_N_SLAVES);
    L9963E_DRV_trans_sleep(&h_th.drv_handle);
#if BMS_DUAL_RING
    L9963E_init(&h_tl, L9963TL_interface, BMS_N_SLAVES);
    L9963E_DRV_trans_sleep(&h_tl.drv_handle);
#endif
}

static void afe_trx_start(void) {
    /* L9963E_init(): CS high, TXEN high, ISOFREQ low, DIS low */
    L9963E_init(&h_th, L9963TH_interface, BMS_N_SLAVES);
#if BMS_DUAL_RING
    L9963E_init(&h_tl, L9963TL_interface, BMS_N_SLAVES);
#endif
}

#if BMS_DUAL_RING
/** GO2SLP without SW_RST: the slaves sleep but keep chip_ID and isotx_en_h (reset source A) */
static void afe_go2slp(L9963E_HandleTypeDef *h) {
    L9963E_RegisterUnionTypeDef reg = {.generic = L9963E_FSM_DEFAULT};
    reg.FSM.GO2SLP                  = 0b10;
    L9963E_DRV_reg_write(&h->drv_handle, L9963E_DEVICE_BROADCAST, L9963E_FSM_ADDR, &reg, 10);
}
#endif

/** A chain still awake in fast mode (MCU reset while it was running) cannot be
 *  addressed at low speed: detect it and reset it (SW_RST + GO2SLP, DS 4.1.2). */
static uint8_t afe_fast_probe_and_reset(void) {
    L9963E_RegisterUnionTypeDef reg = {.generic = 0};
    uint8_t alive;

    afe_set_speed(&h_th, 1);
    alive = (L9963E_DRV_reg_read(&h_th.drv_handle, 1, L9963E_DEV_GEN_CFG_ADDR, &reg, AFE_PROBE_TIMEOUT_MS) ==
             L9963E_OK);
    if (alive) {
        (void)L9963E_sw_rst(&h_th, L9963E_DEVICE_BROADCAST, 1);
        DelayMs(2);
    }
    afe_set_speed(&h_th, 0);
#if BMS_DUAL_RING
    /* Slaves beyond a break of the ring are reachable only from the top: they are
     * put to sleep WITHOUT SW_RST, so that they keep chip_ID and isotx_en_h and can
     * be woken and used again from the L9963TL (afe_complete_from_top). */
    afe_set_speed(&h_tl, 1);
    reg.generic = 0;
    if (L9963E_DRV_reg_read(&h_tl.drv_handle, BMS_N_SLAVES, L9963E_DEV_GEN_CFG_ADDR, &reg, AFE_PROBE_TIMEOUT_MS) ==
        L9963E_OK) {
        afe_go2slp(&h_tl);
        DelayMs(2);
        alive = 1;
    }
    afe_set_speed(&h_tl, 0);
#endif
    return alive;
}

/** Best effort reset of every slave that is awake, at both speeds. */
static void afe_reset_chain_all_speeds(void) {
    afe_set_speed(&h_th, 1);
    (void)L9963E_sw_rst(&h_th, L9963E_DEVICE_BROADCAST, 1);
    afe_set_speed(&h_th, 0);
    (void)L9963E_sw_rst(&h_th, L9963E_DEVICE_BROADCAST, 1);
#if BMS_DUAL_RING
    /* from the top only GO2SLP: see afe_fast_probe_and_reset() */
    afe_set_speed(&h_tl, 1);
    afe_go2slp(&h_tl);
    afe_set_speed(&h_tl, 0);
    afe_go2slp(&h_tl);
#endif
}

/* ------------------------------------------------------------------------- */
/*                              configuration                                 */
/* ------------------------------------------------------------------------- */

/** transceiver through which a slave is reached (dual ring: its current path) */
static L9963E_HandleTypeDef *afe_handle_of(uint8_t dev) {
#if BMS_DUAL_RING
    if (afe.slave[dev - 1U].path == BMS_AFE_PATH_L) {
        return &h_tl;
    }
#else
    (void)dev;
#endif
    return &h_th;
}

/** Broadcast write from the bottom end and, with the dual ring, from the top end
 *  too (reaches the slaves beyond a break; with the ring closed every slave simply
 *  receives it twice). */
static void afe_bcast_write(L9963E_RegistersAddrTypeDef addr, uint32_t value) {
    L9963E_RegisterUnionTypeDef reg = {.generic = value};
    L9963E_DRV_reg_write(&h_th.drv_handle, L9963E_DEVICE_BROADCAST, addr, &reg, 10);
#if BMS_DUAL_RING
    reg.generic = value;
    L9963E_DRV_reg_write(&h_tl.drv_handle, L9963E_DEVICE_BROADCAST, addr, &reg, 10);
#endif
}


/** Reads back a register configured in broadcast (a broadcast write has no
 *  acknowledge: a frame lost on the isoline goes unnoticed). A wrong value is
 *  repaired with a unicast write, whose answer is checked by the driver. */
static int8_t afe_verify_reg(uint8_t dev,
                             L9963E_RegistersAddrTypeDef addr,
                             uint32_t mask,
                             uint32_t expected,
                             const char *name) {
    L9963E_RegisterUnionTypeDef reg = {.generic = 0};
    L9963E_StatusTypeDef st         = L9963E_ERROR;

    for (uint8_t attempt = 0; attempt < BMS_READ_RETRIES; ++attempt) {
        st = L9963E_DRV_reg_read(&afe_handle_of(dev)->drv_handle, dev, addr, &reg, AFE_READ_TIMEOUT_MS);
        if (st != L9963E_OK) {
            continue;
        }
        if ((reg.generic & mask) == (expected & mask)) {
            return (int8_t)L9963E_OK;
        }
        bms_log("AFE: slave %u, %s = 0x%05lX, expected 0x%05lX: rewriting it",
                dev,
                name,
                (unsigned long)(reg.generic & mask),
                (unsigned long)(expected & mask));
        reg.generic = (reg.generic & ~mask) | (expected & mask);
        (void)L9963E_DRV_reg_write(&afe_handle_of(dev)->drv_handle, dev, addr, &reg, AFE_READ_TIMEOUT_MS);
        st = L9963E_READBACK_ERROR;
    }
    bms_log("AFE: slave %u, %s not configured (%d)", dev, name, (int)st);
    return (int8_t)st;
}

static int8_t afe_configure_chain(void) {
    L9963E_RegisterUnionTypeDef gpio_cfg, thresh, vref, comm, sum_th;
    int8_t st;

    /* GPIO7 and GPIO8 start as digital inputs: all GPIOs as analog inputs */
    gpio_cfg.generic                   = L9963E_GPIO9_3_CONF_DEFAULT;
    gpio_cfg.GPIO9_3_CONF.GPIO7_CONFIG = 0;
    gpio_cfg.GPIO9_3_CONF.GPIO8_CONFIG = 0;
    afe_bcast_write(L9963E_GPIO9_3_CONF_ADDR, gpio_cfg.generic);

    /* hardware OV/UV thresholds: second protection level inside the L9963E */
    thresh.generic                          = L9963E_VCELL_THRESH_UV_OV_DEFAULT;
    thresh.VCELL_THRESH_UV_OV.threshVcellOV = afe_thresh_code(BMS_HW_OV_MV, 0);
    thresh.VCELL_THRESH_UV_OV.threshVcellUV = afe_thresh_code(BMS_HW_UV_MV, 1);
    afe_bcast_write(L9963E_VCELL_THRESH_UV_OV_ADDR, thresh.generic);

    /* module sum thresholds disabled (monitored by the firmware) */
    sum_th.generic                      = L9963E_VBATT_SUM_TH_DEFAULT;
    sum_th.VBATT_SUM_TH.VBATT_SUM_OV_TH = 0xFF;
    afe_bcast_write(L9963E_VBATT_SUM_TH_ADDR, sum_th.generic);

    /* VTREF on: 5 V for the NTC pull-ups */
    vref.generic                = L9963E_NCYCLE_PROG_2_DEFAULT;
    vref.NCYCLE_PROG_2.VTREF_EN = 1;
    afe_bcast_write(L9963E_NCYCLE_PROG_2_ADDR, vref.generic);

    /* communication timeout of the slaves */
    comm.generic                  = L9963E_FASTCH_BALUV_DEFAULT;
    comm.fastch_baluv.CommTimeout = BMS_SLAVE_COMM_TIMEOUT;
    afe_bcast_write(L9963E_fastch_baluv_ADDR, comm.generic);

    /* enabled cells */
    afe_bcast_write(L9963E_VCELLS_EN_ADDR, AFE_CELL_MASK);

    /* GPIO OT/UT diagnostics masked: with the default thresholds (GPIOx_UT_TH = 0)
     * every NTC conversion would flag GPIOx_UT, assert the FAULT line and push the
     * conversion routine into Configuration Override (DS 4.11.18). Temperatures are
     * checked by the firmware (bms_monitor.c). The CSA (not used) over-current is masked too. */
    afe_bcast_write(L9963E_CSA_GPIO_MSK_ADDR, AFE_CSA_GPIO_MSK);

    /* read back every slave: a broadcast write has no acknowledge */
    for (uint8_t dev = 1; dev <= BMS_N_SLAVES; ++dev) {
        if ((st = afe_verify_reg(dev, L9963E_VCELLS_EN_ADDR, 0x3FFFU, AFE_CELL_MASK, "VCELLS_EN")) != 0) {
            return st;
        }
        if ((st = afe_verify_reg(dev, L9963E_GPIO9_3_CONF_ADDR, 0x3FFF0U, gpio_cfg.generic, "GPIO9_3_CONF")) != 0) {
            return st;
        }
        if ((st = afe_verify_reg(dev, L9963E_VCELL_THRESH_UV_OV_ADDR, 0xFFFFU, thresh.generic, "VCELL_THRESH")) != 0) {
            return st;
        }
        if ((st = afe_verify_reg(dev, L9963E_NCYCLE_PROG_2_ADDR, 0x20000U, vref.generic, "VTREF_EN")) != 0) {
            return st;
        }
        if ((st = afe_verify_reg(dev, L9963E_fastch_baluv_ADDR, 0x30000U, comm.generic, "CommTimeout")) != 0) {
            return st;
        }
        if ((st = afe_verify_reg(dev, L9963E_CSA_GPIO_MSK_ADDR, 0x3FFFFU, AFE_CSA_GPIO_MSK, "CSA_GPIO_MSK")) != 0) {
            return st;
        }
    }
    return 0;
}

#if BMS_DUAL_RING
/** Loop integrity (DS 4.2.3.2): slave 1 read from the top end crosses links N..1,
 *  slave N read from the bottom end crosses links 0..N-1: together every link. */
static uint8_t afe_ring_check(void) {
    L9963E_RegisterUnionTypeDef reg = {.generic = 0};
    if (L9963E_DRV_reg_read(&h_tl.drv_handle, 1, L9963E_DEV_GEN_CFG_ADDR, &reg, AFE_MEAS_TIMEOUT_MS) != L9963E_OK ||
        reg.DEV_GEN_CFG.chip_ID != 1U) {
        return 0;
    }
    reg.generic = 0;
    if (L9963E_DRV_reg_read(&h_th.drv_handle, BMS_N_SLAVES, L9963E_DEV_GEN_CFG_ADDR, &reg, AFE_MEAS_TIMEOUT_MS) !=
            L9963E_OK ||
        reg.DEV_GEN_CFG.chip_ID != BMS_N_SLAVES) {
        return 0;
    }
    return 1;
}
#endif

#if BMS_DUAL_RING
/** Switches the slaves reachable from @p h to fast isoSPI (port H kept open) and
 *  locks the setting, as the tail of the addressing procedure (DS 4.1.2.2).
 *  @p check: a slave that must answer at fast speed afterwards. */
static uint8_t afe_part_to_fast(L9963E_HandleTypeDef *h, uint8_t check) {
    L9963E_RegisterUnionTypeDef reg = {.generic = L9963E_DEV_GEN_CFG_DEFAULT};
    reg.DEV_GEN_CFG.isotx_en_h      = 1;
    reg.DEV_GEN_CFG.iso_freq_sel    = 0b11;
    L9963E_DRV_ISOFREQ_HIGH(&h->drv_handle); /* the broadcast leaves slow, the transceiver switches after it */
    L9963E_DRV_reg_write(&h->drv_handle, L9963E_DEVICE_BROADCAST, L9963E_DEV_GEN_CFG_ADDR, &reg, 10);
    reg.generic = 0;
    if (L9963E_DRV_reg_read(&h->drv_handle, check, L9963E_DEV_GEN_CFG_ADDR, &reg, AFE_READ_TIMEOUT_MS) != L9963E_OK ||
        reg.DEV_GEN_CFG.chip_ID != check) {
        return 0;
    }
    reg.generic                 = L9963E_BAL_3_DEFAULT;
    reg.Bal_3.Lock_isoh_isofreq = 1;
    L9963E_DRV_reg_write(&h->drv_handle, L9963E_DEVICE_BROADCAST, L9963E_Bal_3_ADDR, &reg, 10);
    return 1;
}

/** Dual ring with a break: slaves k+1..N are woken and read from the top end.
 *  This works only if they keep the address of a previous addressing (chip_ID
 *  and isotx_en_h are reset source A, retained in Sleep): a slave that lost its
 *  supply (POR: port H disabled) cannot be reached from its port H. */
static uint8_t afe_complete_from_top(uint8_t k) {
    L9963E_RegisterUnionTypeDef reg;

    for (uint8_t x = BMS_N_SLAVES; x > k; --x) {
        uint32_t t0 = HAL_GetTick();
        for (;;) {
            reg.generic = 0;
            if (L9963E_DRV_reg_read(&h_tl.drv_handle, x, L9963E_DEV_GEN_CFG_ADDR, &reg, AFE_READ_TIMEOUT_MS) ==
                    L9963E_OK &&
                reg.DEV_GEN_CFG.chip_ID == x) {
                break;
            }
            if (HAL_GetTick() - t0 >= L9963E_ADDR_SLAVE_BUDGET_MS) {
                bms_log("AFE: slave %u not reachable from the L9963TL either", x);
                return 0;
            }
            L9963E_DRV_wakeup(&h_tl.drv_handle);
            DelayMs(2);
        }
        afe.slave[x - 1U].path = BMS_AFE_PATH_L;
    }
    if (!afe_part_to_fast(&h_tl, (uint8_t)(k + 1U))) {
        bms_log("AFE: slaves %u..%u did not follow the switch to fast isoSPI", k + 1U, BMS_N_SLAVES);
        return 0;
    }
    return 1;
}
#endif

/** One initialisation attempt (blocking, at most ~50 ms per slave). */
static uint8_t afe_do_init(void) {
    uint32_t t0 = HAL_GetTick();
    L9963E_StatusTypeDef st;
    int8_t cfg;

    afe.init_attempts++;
    afe.ready = 0;

    afe.fast_reset = afe_fast_probe_and_reset();
    if (afe.fast_reset) {
        bms_log("AFE: chain found awake in fast mode, reset (SW_RST+GO2SLP)");
    }

    for (uint8_t i = 0; i < BMS_N_SLAVES; ++i) {
        afe.slave[i].path = BMS_AFE_PATH_H;
    }
    st              = L9963E_addressing_procedure(&h_th, 0b11, BMS_DUAL_RING, 0b00, 1);
    afe.addressed_n = h_th.addressed_n;
#if BMS_DUAL_RING
    if (st != L9963E_OK && afe.addressed_n < BMS_N_SLAVES) {
        uint8_t k = afe.addressed_n;
        bms_log("AFE: addressing from the L9963TH stopped at slave %u: completing from the L9963TL", k + 1U);
        if ((k == 0U || afe_part_to_fast(&h_th, k)) && afe_complete_from_top(k)) {
            st = L9963E_OK;
        }
    }
#endif
    if (st != L9963E_OK) {
        afe.last_status = (int8_t)st;
        if (afe.addressed_n < BMS_N_SLAVES) {
            bms_log("AFE: addressing failed, %u/%u slaves answered (no answer from slave %u), status %d",
                    afe.addressed_n,
                    BMS_N_SLAVES,
                    afe.addressed_n + 1U,
                    (int)st);
        } else {
            bms_log("AFE: switch to fast isoSPI failed on slave %u, status %d", BMS_N_SLAVES, (int)st);
        }
        afe_reset_chain_all_speeds();
        return 0;
    }

    cfg = afe_configure_chain();
    if (cfg != 0) {
        afe.last_status = cfg;
        bms_log("AFE: configuration failed (%d)", (int)cfg);
        afe_reset_chain_all_speeds();
        return 0;
    }

#if BMS_DUAL_RING
    if (!L9963E_DRV_ISOFREQ_READ(&h_tl.drv_handle)) {
        afe_set_speed(&h_tl, 1);
    }
    afe.ring_ok = afe_ring_check();
    bms_log("AFE: dual ring %s", afe.ring_ok ? "closed" : "OPEN");
#endif

    for (uint8_t i = 0; i < BMS_N_SLAVES; ++i) {
        afe.slave[i].consecutive_fail = 0;
    }
    afe.ready        = 1;
    afe.init_ok++;
    afe.last_init_ms = HAL_GetTick() - t0;
    bms_log("AFE: chain ready, %u slave(s), init %lu ms", BMS_N_SLAVES, (unsigned long)afe.last_init_ms);
    return 1;
}

/* ------------------------------------------------------------------------- */
/*                               public API                                   */
/* ------------------------------------------------------------------------- */

void bms_afe_init(void) {
    memset(&afe, 0, sizeof(afe));
    memset(&cyc, 0, sizeof(cyc));
    IF_Init();
    afe_recovery_request = 0;
    afe_trx_standby();
    afe_set_state(BMS_AFE_STATE_RESET_TRX);
}

void bms_afe_request_recovery(void) {
    afe_recovery_request = 1;
}

void bms_afe_service(uint32_t now_ms) {
    uint32_t elapsed = now_ms - afe.state_since_ms;

    switch (afe.state) {
        case BMS_AFE_STATE_RESET_TRX:
            if (elapsed >= AFE_TRX_RESET_MS) {
                afe_trx_start();
                afe_set_state(BMS_AFE_STATE_START_TRX);
            }
            break;

        case BMS_AFE_STATE_START_TRX:
            if (elapsed >= AFE_TRX_WAKE_MS) {
                afe_set_state(BMS_AFE_STATE_ADDRESSING);
                if (afe_do_init()) {
                    afe_recovery_request = 0;
                    afe_set_state(BMS_AFE_STATE_READY);
                } else {
                    afe_set_state(BMS_AFE_STATE_BACKOFF);
                }
            }
            break;

        case BMS_AFE_STATE_ADDRESSING:
            /* only transient inside afe_do_init() */
            afe_set_state(BMS_AFE_STATE_BACKOFF);
            break;

        case BMS_AFE_STATE_READY:
            if (afe_recovery_request && cyc.phase == CYC_IDLE) {
                afe_recovery_request = 0;
                afe.ready            = 0;
                bms_log("AFE: communication lost, re-initialising the chain");
                afe_reset_chain_all_speeds();
                afe_set_state(BMS_AFE_STATE_BACKOFF);
            }
            break;

        case BMS_AFE_STATE_BACKOFF: {
            /* every slave must have fallen asleep (and back to low speed) */
            uint32_t wait = afe_comm_timeout_ms[BMS_SLAVE_COMM_TIMEOUT] + 50U;
            if (wait < BMS_RECOVERY_PERIOD_MS) {
                wait = BMS_RECOVERY_PERIOD_MS;
            }
            if (elapsed >= wait) {
                afe_trx_standby();
                afe_set_state(BMS_AFE_STATE_RESET_TRX);
            }
            break;
        }

        default:
            afe_set_state(BMS_AFE_STATE_BACKOFF);
            break;
    }
}

/* Vcell registers of the mounted cells (C1..C8, C12..C14) */
static const uint8_t afe_cell_reg[BMS_CELLS_PER_SLAVE] = {L9963E_Vcell1_ADDR + 0U,
                                                          L9963E_Vcell1_ADDR + 1U,
                                                          L9963E_Vcell1_ADDR + 2U,
                                                          L9963E_Vcell1_ADDR + 3U,
                                                          L9963E_Vcell1_ADDR + 4U,
                                                          L9963E_Vcell1_ADDR + 5U,
                                                          L9963E_Vcell1_ADDR + 6U,
                                                          L9963E_Vcell1_ADDR + 7U,
                                                          L9963E_Vcell1_ADDR + 11U,
                                                          L9963E_Vcell1_ADDR + 12U,
                                                          L9963E_Vcell1_ADDR + 13U};

/** One measurement register (value in bits 0..15, d_rdy in bit 16).
 *  d_rdy is cleared by the read: when the answer of a previous attempt was lost
 *  the value is still the one of this conversion, so it is accepted. */
static uint8_t afe_read_meas(L9963E_HandleTypeDef *h, uint8_t dev, uint8_t addr, uint16_t *value) {
    L9963E_RegisterUnionTypeDef reg = {.generic = 0};
    L9963E_StatusTypeDef st         = L9963E_ERROR;
    uint8_t comm_retry              = 0;

    for (uint8_t t = 0; t < BMS_READ_RETRIES; ++t) {
        st = L9963E_DRV_reg_read(&h->drv_handle, dev, (L9963E_RegistersAddrTypeDef)addr, &reg, AFE_MEAS_TIMEOUT_MS);
        if (st == L9963E_OK) {
            if ((reg.generic & 0x10000U) != 0U || comm_retry) {
                *value = (uint16_t)(reg.generic & 0xFFFFU);
                return 1;
            }
            DelayMs(1); /* conversion not finished yet */
        } else {
            comm_retry = 1;
        }
    }
    afe.last_status = (int8_t)st;
    return 0;
}

static uint8_t afe_read_plain(L9963E_HandleTypeDef *h, uint8_t dev, uint8_t addr, uint32_t *value) {
    L9963E_RegisterUnionTypeDef reg = {.generic = 0};
    L9963E_StatusTypeDef st         = L9963E_ERROR;

    for (uint8_t t = 0; t < BMS_READ_RETRIES; ++t) {
        st = L9963E_DRV_reg_read(&h->drv_handle, dev, (L9963E_RegistersAddrTypeDef)addr, &reg, AFE_MEAS_TIMEOUT_MS);
        if (st == L9963E_OK) {
            *value = reg.generic;
            return 1;
        }
    }
    afe.last_status = (int8_t)st;
    return 0;
}

#define AFE_READ_FAIL    0U /* cells not read */
#define AFE_READ_OK      1U /* cells (and GPIOs if requested) read */
#define AFE_READ_NO_GPIO 2U /* cells read and committed, GPIO read failed */

/** Reads every cell, the module voltages and (optionally) the GPIOs of one
 *  slave. Cells and GPIOs are committed separately, each only if complete. */
static uint8_t afe_read_slave(L9963E_HandleTypeDef *h, uint8_t dev, uint8_t with_gpio, bms_afe_slave_t *s) {
    uint16_t cells[BMS_CELLS_PER_SLAVE];
    uint16_t gpios[BMS_GPIO_PER_SLAVE];
    uint32_t vdiv_reg = 0, vsum_reg = 0;

    for (uint8_t i = 0; i < BMS_CELLS_PER_SLAVE; ++i) {
        if (!afe_read_meas(h, dev, afe_cell_reg[i], &cells[i])) {
            return AFE_READ_FAIL;
        }
    }
    if (!afe_read_plain(h, dev, L9963E_VBATTDIV_ADDR, &vdiv_reg) ||
        !afe_read_plain(h, dev, L9963E_VSUMBATT_ADDR, &vsum_reg)) {
        return AFE_READ_FAIL;
    }
    memcpy(s->cell_raw, cells, sizeof(cells));
    s->vbatt_div_raw = (uint16_t)(vdiv_reg & 0xFFFFU);
    s->vsum_raw      = ((vsum_reg & 0x3FFFFU) << 2) | ((vdiv_reg >> 16) & 0x3U);
    s->last_ok_ms    = HAL_GetTick();
    s->cell_seq++;

    if (!with_gpio) {
        return AFE_READ_OK;
    }
    memcpy(gpios, s->gpio_raw, sizeof(gpios));
    for (uint8_t g = 0; g < BMS_GPIO_PER_SLAVE; ++g) {
        if ((BMS_NTC_GPIO_MASK & (1U << g)) == 0U) {
            continue;
        }
        if (!afe_read_meas(h, dev, (uint8_t)(L9963E_GPIO3_MEAS_ADDR + g), &gpios[g])) {
            return AFE_READ_NO_GPIO;
        }
    }
    memcpy(s->gpio_raw, gpios, sizeof(gpios));
    s->last_gpio_ok_ms = HAL_GetTick();
    s->gpio_seq++;
    return AFE_READ_OK;
}

/* ------------------------------------------------------------------------- */
/*                 measurement cycle (non blocking state machine)              */
/* ------------------------------------------------------------------------- */

static void afe_start_conversion(L9963E_HandleTypeDef *h, uint8_t with_gpio) {
    L9963E_start_conversion(h, L9963E_DEVICE_BROADCAST, BMS_ADC_FILTER_SOC, with_gpio ? (uint8_t)L9963E_GPIO_CONV : 0U);
}

/** @return 1 once at least BMS_CONV_WAIT_MS (+2 ms with GPIO) have elapsed since t_soc */
static uint8_t afe_conversion_done(uint32_t now) {
    uint32_t wait = BMS_CONV_WAIT_MS + (cyc.with_gpio ? 2U : 0U);
    return (now - cyc.t_soc) >= wait + 1U; /* +1: HAL tick granularity */
}

/** Reads a slave through the given path, unless the path is known to be broken
 *  in this pass (a slave closer to the transceiver did not answer at all). */
static uint8_t afe_read_via(uint8_t path, uint8_t dev, uint8_t with_gpio) {
    L9963E_HandleTypeDef *h = &h_th;
#if BMS_DUAL_RING
    if (path == BMS_AFE_PATH_L) {
        h = &h_tl;
    }
#endif
    if (cyc.path_dead[path]) {
        afe.last_status = (int8_t)L9963E_TIMEOUT;
        return AFE_READ_FAIL;
    }
    uint8_t r = afe_read_slave(h, dev, with_gpio, &afe.slave[dev - 1U]);
    if (r == AFE_READ_FAIL && afe.last_status == (int8_t)L9963E_TIMEOUT) {
        cyc.path_dead[path] = 1;
    }
    return r;
}

#if BMS_DUAL_RING
/** @p l_converted: the slaves reachable from the top received a start of conversion */
static uint8_t afe_read_slave_dual(uint8_t dev, uint8_t with_gpio, uint8_t first_path, uint8_t l_converted, uint8_t *path) {
    uint8_t other = (first_path == BMS_AFE_PATH_H) ? BMS_AFE_PATH_L : BMS_AFE_PATH_H;
    *path         = first_path;
    if ((first_path == BMS_AFE_PATH_H || l_converted) && afe_read_via(first_path, dev, with_gpio) == AFE_READ_OK) {
        return 1;
    }
    *path = other;
    if ((other == BMS_AFE_PATH_H || l_converted) && afe_read_via(other, dev, with_gpio) == AFE_READ_OK) {
        return 1;
    }
    return 0;
}
#endif

static void afe_account(uint8_t dev, uint8_t ok, uint8_t path) {
    bms_afe_slave_t *s = &afe.slave[dev - 1U];
    if (ok) {
        if (s->consecutive_fail != 0U || s->path != path) {
            bms_log("AFE: slave %u OK via L9963T%c", dev, path == BMS_AFE_PATH_H ? 'H' : 'L');
        }
        s->path             = path;
        s->consecutive_fail = 0;
        cyc.ok_n++;
        return;
    }
    if (s->consecutive_fail == 0U) {
        bms_log("AFE: slave %u not answering (%d)", dev, (int)afe.last_status);
    }
    if (s->consecutive_fail < 0xFFFFU) {
        s->consecutive_fail++;
    }
    s->total_fail++;
    if (s->consecutive_fail >= AFE_FAIL_RECOVERY) {
        cyc.recover = 1;
    }
}

static uint8_t afe_cycle_finish(uint32_t now) {
    afe.cycles++;
    afe.last_cycle_ms  = now - cyc.t_start;
    afe.last_cycle_ok  = cyc.ok_n;
    cyc.phase          = CYC_IDLE;
    if (cyc.recover) {
        afe_recovery_request = 1;
    }
    return 1;
}

uint8_t bms_afe_cycle_busy(void) {
    return cyc.phase != CYC_IDLE;
}

uint8_t bms_afe_cycle_start(uint8_t with_gpio, uint32_t now_ms) {
    if (!afe.ready || afe.state != BMS_AFE_STATE_READY || cyc.phase != CYC_IDLE) {
        return 0;
    }
    cyc.with_gpio = with_gpio;
    cyc.ok_n      = 0;
    cyc.recover   = 0;
    cyc.n_failed  = 0;
    cyc.next      = 1;
    cyc.t_start   = now_ms;
    cyc.path_dead[0] = cyc.path_dead[1] = 0;

    /* one start of conversion for the whole chain: every slave samples at the same time */
    afe_start_conversion(&h_th, with_gpio);
    cyc.use_tl = 0;
#if BMS_DUAL_RING
    cyc.use_tl = !afe.ring_ok;
    for (uint8_t i = 0; i < BMS_N_SLAVES; ++i) {
        cyc.use_tl |= (afe.slave[i].path == BMS_AFE_PATH_L);
    }
    if (cyc.use_tl) {
        /* slaves beyond a break are reached only from the top of the ring */
        afe_start_conversion(&h_tl, with_gpio);
    }
#endif
    cyc.t_soc = HAL_GetTick();
    cyc.phase = CYC_CONV;
    return 1;
}

uint8_t bms_afe_cycle_poll(uint32_t now_ms) {
    switch (cyc.phase) {
        case CYC_CONV:
            if (!afe_conversion_done(now_ms)) {
                return 0;
            }
            cyc.phase = CYC_READ;
            cyc.next  = 1;
            return 0;

        case CYC_READ: {
            uint8_t dev = cyc.next++;
#if BMS_DUAL_RING
            uint8_t path;
            /* start from the side that worked last time: a broken ring costs a timeout only once */
            if (afe_read_slave_dual(dev, cyc.with_gpio, afe.slave[dev - 1U].path, cyc.use_tl, &path)) {
                afe_account(dev, 1, path);
            } else {
                cyc.failed[cyc.n_failed++] = dev;
            }
#else
            /* a partial read (cells without temperatures) counts as a failed cycle for the
             * recovery logic; the stale temperatures are caught by bms_monitor.c */
            afe_account(dev, afe_read_via(BMS_AFE_PATH_H, dev, cyc.with_gpio) == AFE_READ_OK, BMS_AFE_PATH_H);
#endif
            if (cyc.next <= BMS_N_SLAVES) {
                return 0;
            }
#if BMS_DUAL_RING
            uint8_t ring = afe_ring_check();
            if (ring != afe.ring_ok) {
                bms_log("AFE: dual ring %s", ring ? "closed again" : "OPEN");
                if (ring) {
                    /* back to the bottom transceiver for everybody: the L9963TL is only used
                     * for the ring check, so its RX queue does not collect the commands */
                    for (uint8_t i = 0; i < BMS_N_SLAVES; ++i) {
                        afe.slave[i].path = BMS_AFE_PATH_H;
                    }
                }
            }
            afe.ring_ok = ring;
            if (cyc.n_failed != 0U && !cyc.use_tl) {
                /* the ring has just opened: the slaves beyond the break did not receive the
                 * start of conversion. Convert again from the top end and read them in this
                 * same cycle, so that not even one cycle of data is lost. */
                afe_start_conversion(&h_tl, cyc.with_gpio);
                cyc.t_soc                     = HAL_GetTick();
                cyc.path_dead[BMS_AFE_PATH_L] = 0;
                cyc.phase                     = CYC_CONV2;
                return 0;
            }
            for (uint8_t k = 0; k < cyc.n_failed; ++k) {
                afe_account(cyc.failed[k], 0, afe.slave[cyc.failed[k] - 1U].path);
            }
#endif
            return afe_cycle_finish(HAL_GetTick());
        }

#if BMS_DUAL_RING
        case CYC_CONV2:
            if (!afe_conversion_done(now_ms)) {
                return 0;
            }
            cyc.phase = CYC_READ2;
            cyc.next  = 0;
            return 0;

        case CYC_READ2: {
            uint8_t dev = cyc.failed[cyc.next++];
            uint8_t path;
            uint8_t ok = afe_read_slave_dual(dev, cyc.with_gpio, BMS_AFE_PATH_L, 1, &path);
            afe_account(dev, ok, path);
            if (cyc.next < cyc.n_failed) {
                return 0;
            }
            return afe_cycle_finish(HAL_GetTick());
        }
#endif

        case CYC_IDLE:
        default:
            cyc.phase = CYC_IDLE;
            return 0;
    }
}

uint8_t bms_afe_measure(uint8_t with_gpio) {
    if (!bms_afe_cycle_start(with_gpio, HAL_GetTick())) {
        return 0;
    }
    while (!bms_afe_cycle_poll(HAL_GetTick())) {
        bms_platform_watchdog_kick();
    }
    return afe.last_cycle_ok;
}

const bms_afe_t *bms_afe_get(void) {
    return &afe;
}

int16_t bms_afe_ntc_dc(uint16_t raw) {
    /* 4th order fit of the NTC used on the BMS LV (Lib: BMS_LV/Core/Src/ntc.c) */
    const float a = 121.77018530814999f;
    const float b = -92.86284471272114f;
    const float c = 39.33876760742037f;
    const float d = -8.798397291741638f;
    const float e = 0.7020187110716422f;

    float v  = (float)raw * 0.000089f;
    float v2 = v * v;
    float t  = a + b * v + c * v2 + d * v2 * v + e * v2 * v2;
    float dc = t * 10.0f;

    if (dc > 3000.0f) {
        dc = 3000.0f;
    }
    if (dc < -1000.0f) {
        dc = -1000.0f;
    }
    return (int16_t)(dc >= 0.0f ? dc + 0.5f : dc - 0.5f);
}

/* ------------------------------------------------------------------------- */
/*                    compatibility with bms_hv_fsm.c                          */
/* ------------------------------------------------------------------------- */
#include "L9963_utils.h"

L9963_Utils_StatusTypeDef L9963E_utils_balance_cells(void) {
    /* balancing has not been ported/validated yet: refuse it explicitly */
    bms_log("AFE: balancing requested but not enabled in this firmware");
    return L9963E_UTILS_ERROR;
}
