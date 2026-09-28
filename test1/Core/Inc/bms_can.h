/**
 * @file    bms_can.h
 * @brief   Telemetry of the BMS HV on CAN (HVCB network, messages HVB_RX_*).
 */
#ifndef BMS_CAN_H
#define BMS_CAN_H

#include <stdint.h>

#include "bms_config.h"

#if BMS_ENABLE_CAN
void bms_can_init(void);
void bms_can_service(uint32_t now_ms);
uint32_t bms_can_tx_count(void);
uint32_t bms_can_tx_dropped(void);
#else
#define bms_can_init()         ((void)0)
#define bms_can_service(now)   ((void)(now))
#define bms_can_tx_count()     (0U)
#define bms_can_tx_dropped()   (0U)
#endif

#endif /* BMS_CAN_H */
