/**
 * @file    bms_app.h
 * @brief   Application layer of the BMS HV master: cooperative scheduler that
 *          runs the AFE chain, the safety monitor, CAN telemetry, log, LEDs and
 *          the independent watchdog.
 *
 * Integration in main.c (USER CODE sections only):
 *   USER CODE 2      -> bms_app_init();
 *   while (1) { ...  -> bms_app_loop();
 */
#ifndef BMS_APP_H
#define BMS_APP_H

#include <stdint.h>

#include "bms_config.h"

void bms_app_init(void);
void bms_app_loop(void);

/** Reset cause read at boot (RCC->CSR, bits 24..31). */
uint32_t bms_app_reset_flags(void);

#endif /* BMS_APP_H */
