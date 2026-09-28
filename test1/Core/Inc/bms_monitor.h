/**
 * @file    bms_monitor.h
 * @brief   Plausibility and safety checks on the AFE data, AMS error output.
 */
#ifndef BMS_MONITOR_H
#define BMS_MONITOR_H

#include <stdint.h>

#include "bms_config.h"

/* latched fault bits */
#define BMS_FAULT_CELL_OV (1UL << 0) /**< cell over-voltage */
#define BMS_FAULT_CELL_UV (1UL << 1) /**< cell under-voltage */
#define BMS_FAULT_CELL_OT (1UL << 2) /**< cell over-temperature */
#define BMS_FAULT_CELL_UT (1UL << 3) /**< cell under-temperature */
#define BMS_FAULT_NTC     (1UL << 4) /**< NTC open or shorted */
#define BMS_FAULT_COMM    (1UL << 5) /**< slave not readable (or chain never initialised) */
#define BMS_FAULT_TEMP_COMM (1UL << 6) /**< temperatures of a slave not refreshed */
#define BMS_FAULT_WATCHDOG  (1UL << 7) /**< the MCU was reset by a watchdog: firmware failure */

typedef struct {
    uint32_t faults;       /**< latched faults (BMS_FAULT_*) */
    uint32_t active;       /**< conditions present now (not filtered) */
    uint8_t ams_error;     /**< 1 = AMS error output asserted (latched until reset) */
    uint8_t cells_valid;   /**< every slave read at least once */
    uint8_t temps_valid;   /**< every enabled NTC read at least once and in range */
    int32_t cell_min_mv, cell_max_mv, cell_mean_mv;
    uint16_t cell_min_idx, cell_max_idx; /**< global index: slave*11 + cell */
    int16_t t_min_dc, t_max_dc, t_mean_dc;
    uint16_t t_min_idx, t_max_idx;       /**< global index: slave*7 + gpio */
    int32_t pack_mv;       /**< sum of all cell voltages */
    int32_t vsum_mv;       /**< sum of the module VSUM registers */
    int32_t vbatt_mv;      /**< sum of the module VBATT_DIV registers */
    uint32_t first_fault;  /**< first fault that asserted the AMS error */
    uint16_t first_fault_idx;
    int32_t first_fault_value;
    uint32_t first_fault_ms;
} bms_status_t;

void bms_monitor_init(uint32_t now_ms);
/** Latches @p fault immediately (e.g. BMS_FAULT_WATCHDOG at boot). */
void bms_monitor_force_fault(uint32_t fault, uint32_t now_ms);
void bms_monitor_update(uint32_t now_ms);
const bms_status_t *bms_monitor_get(void);

#endif /* BMS_MONITOR_H */
