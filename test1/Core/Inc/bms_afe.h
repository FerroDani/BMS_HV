/**
 * @file    bms_afe.h
 * @brief   Management of the L9963E daisy chain (analog front end of the BMS HV).
 *
 * Non blocking initialisation / recovery state machine, measurement cycle with
 * bounded retries, optional dual access ring through the L9963TL.
 */
#ifndef BMS_AFE_H
#define BMS_AFE_H

#include <stdint.h>

#include "bms_config.h"

typedef enum {
    BMS_AFE_STATE_RESET_TRX = 0, /**< transceivers in stand-by (DIS released) */
    BMS_AFE_STATE_START_TRX,     /**< DIS pulled low, waiting T_WAKEUP of the L9963T */
    BMS_AFE_STATE_ADDRESSING,    /**< addressing + configuration of the chain */
    BMS_AFE_STATE_READY,         /**< chain configured, measurements running */
    BMS_AFE_STATE_BACKOFF,       /**< waiting before a new initialisation attempt */
} bms_afe_state_t;

typedef enum {
    BMS_AFE_PATH_H = 0, /**< slave reached through the L9963TH (SPI3) */
    BMS_AFE_PATH_L = 1, /**< slave reached through the L9963TL (SPI2), dual ring only */
} bms_afe_path_t;

typedef struct {
    uint16_t cell_raw[BMS_CELLS_PER_SLAVE]; /**< 89 uV/LSB */
    uint16_t gpio_raw[BMS_GPIO_PER_SLAVE];  /**< GPIO3..GPIO9, 89 uV/LSB */
    uint16_t vbatt_div_raw;                 /**< module voltage from the VBAT divider, 1.33 mV/LSB */
    uint32_t vsum_raw;                      /**< digital sum of the cells, 89 uV/LSB */
    uint32_t last_ok_ms;                    /**< time of the last complete cell read */
    uint32_t last_gpio_ok_ms;               /**< time of the last complete GPIO read */
    uint32_t cell_seq;                      /**< incremented at every successful cell read */
    uint32_t gpio_seq;                      /**< incremented at every successful GPIO read */
    uint8_t path;                           /**< bms_afe_path_t used in the last successful read */
    uint16_t consecutive_fail;              /**< failed cycles in a row */
    uint32_t total_fail;                    /**< failed cycles since boot */
} bms_afe_slave_t;

typedef struct {
    bms_afe_state_t state;
    uint8_t ready;         /**< 1 when the chain is addressed and configured */
    uint8_t addressed_n;   /**< slaves that answered in the last addressing attempt */
    uint8_t ring_ok;       /**< dual ring: slave 1 reachable from the L9963TL */
    uint8_t fast_reset;    /**< a chain still awake in fast mode was reset at the last init */
    int8_t last_status;    /**< last L9963E_StatusTypeDef of a failed operation */
    uint32_t state_since_ms;
    uint32_t init_attempts;
    uint32_t init_ok;
    uint32_t cycles;
    uint32_t last_cycle_ms; /**< duration of the last measurement cycle (start of conversion -> last read) */
    uint8_t last_cycle_ok;  /**< slaves read correctly in the last cycle */
    uint32_t last_init_ms;  /**< duration of the last successful initialisation */
    bms_afe_slave_t slave[BMS_N_SLAVES];
} bms_afe_t;

/** Starts the initialisation of the chain (non blocking). */
void bms_afe_init(void);

/** Progresses initialisation / recovery. Call it often (it may block for the
 *  duration of one addressing attempt, at most ~50 ms per slave). */
void bms_afe_service(uint32_t now_ms);

/** Starts a measurement cycle: start of conversion in broadcast (cells + module
 *  voltages, GPIOs if @p with_gpio). Non blocking.
 *  @return 1 if started, 0 if the chain is not ready or a cycle is running. */
uint8_t bms_afe_cycle_start(uint8_t with_gpio, uint32_t now_ms);

/** Progresses the running cycle: waits the conversion without blocking, then
 *  reads one slave per call (~1 ms each at fast isoSPI).
 *  @return 1 when the cycle has just completed. */
uint8_t bms_afe_cycle_poll(uint32_t now_ms);

uint8_t bms_afe_cycle_busy(void);

/** Blocking helper: one complete measurement cycle (debug / tests).
 *  @return number of slaves read correctly. */
uint8_t bms_afe_measure(uint8_t with_gpio);

/** Requests a re-initialisation of the chain at the next service call. */
void bms_afe_request_recovery(void);

const bms_afe_t *bms_afe_get(void);
const char *bms_afe_state_name(bms_afe_state_t s);

/* ------------------------------ conversions ------------------------------ */
static inline int32_t bms_afe_cell_mv(uint16_t raw) {
    return (int32_t)(((uint32_t)raw * 89U + 500U) / 1000U);
}
static inline int32_t bms_afe_gpio_mv(uint16_t raw) {
    return (int32_t)(((uint32_t)raw * 89U + 500U) / 1000U);
}
static inline int32_t bms_afe_vbatt_mv(uint16_t raw) {
    return (int32_t)(((uint32_t)raw * 133U + 50U) / 100U);
}
static inline int32_t bms_afe_vsum_mv(uint32_t raw) {
    return (int32_t)(((uint64_t)raw * 89U + 500U) / 1000U);
}
/** NTC temperature [0.1 degC] from the GPIO reading (curve of the BMS LV NTC:
 *  10 kOhm NTC to GND, 10 kOhm pull-up to VTREF = 5 V). */
int16_t bms_afe_ntc_dc(uint16_t raw);

#endif /* BMS_AFE_H */
