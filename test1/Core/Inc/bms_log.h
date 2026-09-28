/**
 * @file    bms_log.h
 * @brief   Debug log on USART3 (PB10 TX, 115200 8N1). Integers only (no float printf).
 *          Non blocking: lines go to a ring buffer drained by bms_log_service().
 */
#ifndef BMS_LOG_H
#define BMS_LOG_H

#include <stdint.h>

#include "bms_config.h"

#if BMS_ENABLE_LOG
void bms_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
/** Drains the buffer and prints the periodic status (call it from the main loop). */
void bms_log_service(uint32_t now_ms);
/** Blocking drain of the buffer (boot banner, before a reset). */
void bms_log_flush(uint32_t timeout_ms);
uint32_t bms_log_dropped(void);
#else
#define bms_log(...)            ((void)0)
#define bms_log_service(now)    ((void)(now))
#define bms_log_flush(timeout)  ((void)(timeout))
#define bms_log_dropped()       (0U)
#endif

#endif /* BMS_LOG_H */
