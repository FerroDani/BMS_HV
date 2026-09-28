/**
 * @file    L9963_utils.h
 * @brief   Compatibility header for bms_hv_fsm.c.
 *
 * The L9963E chain is now managed by bms_afe.c (see bms_afe.h); this header
 * only keeps the names used by the relay state machine.
 */
#ifndef L9963_UTILS_H
#define L9963_UTILS_H

#include "L9963E.h"
#include "L9963E_drv.h"
#include "bms_afe.h"
#include "main.h"
#include "stm32_if.h"

#define N_SLAVES          BMS_N_SLAVES
#define N_CELLS_PER_SLAVE BMS_CELLS_PER_SLAVE

typedef enum {
    L9963_UTILS_OK = 0,
    L9963E_UTILS_ERROR,
} L9963_Utils_StatusTypeDef;

/** Not supported yet: always returns L9963E_UTILS_ERROR. */
L9963_Utils_StatusTypeDef L9963E_utils_balance_cells(void);

#endif  // L9963_UTILS_H
