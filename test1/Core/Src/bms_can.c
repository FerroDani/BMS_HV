/**
 * @file    bms_can.c
 * @brief   Telemetry of the BMS HV on CAN1 (HVCB network, SCan SC26 artifacts).
 *
 * Messages (cycle times from HVCB.dbc):
 *   0x200 HVB_RX_Diagnosis  10 ms   fault flags
 *   0x203 HVB_RX_Status     20 ms   stSys: 0 init, 1 running, 2 AMS error
 *   0x204 HVB_RX_Measure    20 ms   uHvb = sum of the cell voltages
 *   0x206 HVB_RX_VCell     100 ms   min/mean/max cell voltage + indexes
 *   0x208 HVB_RX_TCell     100 ms   min/mean/max temperature + indexes
 *   0x20F HVB_RX_SWVersion 1000 ms
 * Transmission is non blocking: when the three mailboxes are still pending
 * (e.g. bench without other nodes acknowledging the frames) they are aborted
 * and the new message is dropped (counted in bms_can_tx_dropped()).
 */
#include "bms_can.h"

#if BMS_ENABLE_CAN

#include "bms_afe.h"
#include "bms_monitor.h"
#include "can.h"
#include "hvcb.h"
#include "main.h"

#define BMS_CAN_HANDLE hcan1

static uint32_t tx_count, tx_dropped;
static uint32_t dropped_at_window, drop_window_ms; /* frames dropped in the last second */
static uint8_t can_trouble;
static uint32_t t_10, t_20, t_100, t_1000;
static uint8_t can_ok;

static void can_send(uint32_t id, const uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef hdr = {0};
    uint32_t mailbox;

    if (!can_ok) {
        return;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&BMS_CAN_HANDLE) == 0U) {
        /* nobody acknowledges (bench) or bus overloaded: drop the stale frames so
         * that the mailboxes always carry fresh data */
        (void)HAL_CAN_AbortTxRequest(&BMS_CAN_HANDLE, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        tx_dropped++;
        return;
    }
    hdr.StdId              = id;
    hdr.IDE                = CAN_ID_STD;
    hdr.RTR                = CAN_RTR_DATA;
    hdr.DLC                = len;
    hdr.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&BMS_CAN_HANDLE, &hdr, (uint8_t *)data, &mailbox) == HAL_OK) {
        tx_count++;
    } else {
        tx_dropped++;
    }
}

void bms_can_init(void) {
    CAN_FilterTypeDef filter = {0};

    /* bit timing for APB1 = 45 MHz (the CubeMX default of the project is not a standard rate) */
    HAL_CAN_DeInit(&BMS_CAN_HANDLE);
#if BMS_CAN_BITRATE_KBPS == 1000U
    BMS_CAN_HANDLE.Init.Prescaler = 5; /* 45 MHz / 5 = 9 MHz, 9 tq */
    BMS_CAN_HANDLE.Init.TimeSeg1  = CAN_BS1_7TQ;
    BMS_CAN_HANDLE.Init.TimeSeg2  = CAN_BS2_1TQ;
#elif BMS_CAN_BITRATE_KBPS == 500U
    BMS_CAN_HANDLE.Init.Prescaler = 5; /* 45 MHz / 5 = 9 MHz, 18 tq */
    BMS_CAN_HANDLE.Init.TimeSeg1  = CAN_BS1_15TQ;
    BMS_CAN_HANDLE.Init.TimeSeg2  = CAN_BS2_2TQ;
#else
    BMS_CAN_HANDLE.Init.Prescaler = 10; /* 45 MHz / 10 = 4.5 MHz, 18 tq */
    BMS_CAN_HANDLE.Init.TimeSeg1  = CAN_BS1_15TQ;
    BMS_CAN_HANDLE.Init.TimeSeg2  = CAN_BS2_2TQ;
#endif
    BMS_CAN_HANDLE.Init.SyncJumpWidth      = CAN_SJW_1TQ;
    BMS_CAN_HANDLE.Init.Mode               = CAN_MODE_NORMAL;
    BMS_CAN_HANDLE.Init.AutoBusOff         = ENABLE;
    BMS_CAN_HANDLE.Init.AutoRetransmission = ENABLE; /* a frame that loses arbitration is retried */
    if (HAL_CAN_Init(&BMS_CAN_HANDLE) != HAL_OK) {
        return;
    }

    /* accept everything in FIFO0 (nothing is processed yet) */
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;
    (void)HAL_CAN_ConfigFilter(&BMS_CAN_HANDLE, &filter);

    can_ok = (HAL_CAN_Start(&BMS_CAN_HANDLE) == HAL_OK);
}

static uint8_t sys_state(void) {
    const bms_afe_t *afe   = bms_afe_get();
    const bms_status_t *st = bms_monitor_get();
    if (st->ams_error) {
        return 2;
    }
    return afe->ready ? 1 : 0;
}

static void send_diagnosis(void) {
    const bms_status_t *st = bms_monitor_get();
    struct hvcb_hvb_rx_diagnosis_t m;
    uint8_t buf[8];

    hvcb_hvb_rx_diagnosis_init(&m);
    m.hvb_diag_cell_ov  = (st->faults & BMS_FAULT_CELL_OV) != 0U;
    m.hvb_diag_cell_uv  = (st->faults & BMS_FAULT_CELL_UV) != 0U;
    m.hvb_diag_cell_ot  = (st->faults & BMS_FAULT_CELL_OT) != 0U;
    m.hvb_diag_cell_ut  = (st->faults & (BMS_FAULT_CELL_UT | BMS_FAULT_NTC)) != 0U;
    m.hvb_diag_cell_sna = (st->faults & BMS_FAULT_COMM) != 0U;
    m.hvb_diag_can      = can_trouble;
    if (hvcb_hvb_rx_diagnosis_pack(buf, &m, sizeof(buf)) > 0) {
        can_send(HVCB_HVB_RX_DIAGNOSIS_FRAME_ID, buf, HVCB_HVB_RX_DIAGNOSIS_LENGTH);
    }
}

static void send_status_measure(void) {
    const bms_status_t *st = bms_monitor_get();
    struct hvcb_hvb_rx_status_t s;
    struct hvcb_hvb_rx_measure_t m;
    uint8_t buf[8];

    hvcb_hvb_rx_status_init(&s);
    s.hvb_st_sys = hvcb_hvb_rx_status_hvb_st_sys_encode(sys_state());
    if (hvcb_hvb_rx_status_pack(buf, &s, sizeof(buf)) > 0) {
        can_send(HVCB_HVB_RX_STATUS_FRAME_ID, buf, HVCB_HVB_RX_STATUS_LENGTH);
    }

    hvcb_hvb_rx_measure_init(&m);
    m.hvb_u_hvb = hvcb_hvb_rx_measure_hvb_u_hvb_encode((double)st->pack_mv / 1000.0);
    if (hvcb_hvb_rx_measure_pack(buf, &m, sizeof(buf)) > 0) {
        can_send(HVCB_HVB_RX_MEASURE_FRAME_ID, buf, HVCB_HVB_RX_MEASURE_LENGTH);
    }
}

static void send_cells(void) {
    const bms_status_t *st = bms_monitor_get();
    struct hvcb_hvb_rx_v_cell_t v;
    struct hvcb_hvb_rx_t_cell_t t;
    uint8_t buf[8];

    hvcb_hvb_rx_v_cell_init(&v);
    v.hvb_u_cell_max      = hvcb_hvb_rx_v_cell_hvb_u_cell_max_encode((double)st->cell_max_mv / 1000.0);
    v.hvb_u_cell_mean     = hvcb_hvb_rx_v_cell_hvb_u_cell_mean_encode((double)st->cell_mean_mv / 1000.0);
    v.hvb_u_cell_min      = hvcb_hvb_rx_v_cell_hvb_u_cell_min_encode((double)st->cell_min_mv / 1000.0);
    v.hvb_idx_cell_u_max  = hvcb_hvb_rx_v_cell_hvb_idx_cell_u_max_encode((double)st->cell_max_idx);
    v.hvb_idx_cell_u_min  = hvcb_hvb_rx_v_cell_hvb_idx_cell_u_min_encode((double)st->cell_min_idx);
    if (hvcb_hvb_rx_v_cell_pack(buf, &v, sizeof(buf)) > 0) {
        can_send(HVCB_HVB_RX_V_CELL_FRAME_ID, buf, 8);
    }

    hvcb_hvb_rx_t_cell_init(&t);
    t.hvb_t_cell_max     = hvcb_hvb_rx_t_cell_hvb_t_cell_max_encode((double)st->t_max_dc / 10.0);
    t.hvb_t_cell_mean    = hvcb_hvb_rx_t_cell_hvb_t_cell_mean_encode((double)st->t_mean_dc / 10.0);
    t.hvb_t_cell_min     = hvcb_hvb_rx_t_cell_hvb_t_cell_min_encode((double)st->t_min_dc / 10.0);
    t.hvb_idx_cell_t_max = hvcb_hvb_rx_t_cell_hvb_idx_cell_t_max_encode((double)st->t_max_idx);
    t.hvb_idx_cell_t_min = hvcb_hvb_rx_t_cell_hvb_idx_cell_t_min_encode((double)st->t_min_idx);
    if (hvcb_hvb_rx_t_cell_pack(buf, &t, sizeof(buf)) > 0) {
        can_send(HVCB_HVB_RX_T_CELL_FRAME_ID, buf, 8);
    }
}

static void send_version(void) {
    struct hvcb_hvb_rx_sw_version_t m;
    uint8_t buf[8];

    hvcb_hvb_rx_sw_version_init(&m);
    m.hvb_no_sw_vers0 = (BMS_FW_VERSION_MAJOR << 16) | (BMS_FW_VERSION_MINOR << 8) | BMS_FW_VERSION_PATCH;
    m.hvb_no_sw_vers1 = BMS_N_SLAVES | ((uint32_t)BMS_DUAL_RING << 8);
    if (hvcb_hvb_rx_sw_version_pack(buf, &m, sizeof(buf)) > 0) {
        can_send(HVCB_HVB_RX_SW_VERSION_FRAME_ID, buf, HVCB_HVB_RX_SW_VERSION_LENGTH);
    }
}

void bms_can_service(uint32_t now) {
    if (now - drop_window_ms >= 1000U) {
        can_trouble       = (tx_dropped != dropped_at_window);
        dropped_at_window = tx_dropped;
        drop_window_ms    = now;
    }
    if (now - t_10 >= 10U) {
        t_10 = now;
        send_diagnosis();
    }
    if (now - t_20 >= 20U) {
        t_20 = now;
        send_status_measure();
    }
    if (now - t_100 >= 100U) {
        t_100 = now;
        send_cells();
    }
    if (now - t_1000 >= 1000U) {
        t_1000 = now;
        send_version();
    }
}

uint32_t bms_can_tx_count(void) {
    return tx_count;
}

uint32_t bms_can_tx_dropped(void) {
    return tx_dropped;
}

#endif /* BMS_ENABLE_CAN */
