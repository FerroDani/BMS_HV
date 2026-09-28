/**
 * @file    bms_config.h
 * @brief   Compile time configuration of the BMS HV firmware.
 *
 * Every value can be overridden from the compiler command line, e.g.
 *   make BMS_DEFS="-DBMS_N_SLAVES=11 -DBMS_DUAL_RING=1"
 *
 * Datasheet references: L9963E DS13636 rev 11, L9963T DS13590 rev 4.
 */
#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

/* ================================ Topology =============================== */

/** Number of L9963E slaves in the daisy chain (1 on the bench, 11 on the car). */
#ifndef BMS_N_SLAVES
#define BMS_N_SLAVES 1U
#endif

/** 1 = dual access ring: the chain is closed on the L9963TL (SPI2) as well.
 *  The last slave keeps its port H enabled, every slave can be reached from
 *  both ends and a single break of the isoSPI wiring is tolerated.            */
#ifndef BMS_DUAL_RING
#define BMS_DUAL_RING 0
#endif

#define BMS_MAX_SLAVES 31U /* 5 bit device id */

/* ================================= Cells ================================= */

/** Cells per module. The mapping on the 14 L9963E inputs is in bms_afe.c
 *  (C1..C8 + C12..C14: pair C9-C10 and odd cell C11 not mounted, DS 6.10.1.3). */
#define BMS_CELLS_PER_SLAVE 11U

/** GPIO3..GPIO9 of each slave that carry an NTC (bit0 = GPIO3 ... bit6 = GPIO9). */
#ifndef BMS_NTC_GPIO_MASK
#define BMS_NTC_GPIO_MASK 0x7FU
#endif
#define BMS_GPIO_PER_SLAVE 7U

/* ================================ Timing ================================= */

/** Period of the measurement cycle [ms]. */
#ifndef BMS_MEAS_PERIOD_MS
#define BMS_MEAS_PERIOD_MS 100U
#endif

/** GPIO (temperature) conversion every N measurement cycles. */
#ifndef BMS_GPIO_EVERY_N_CYCLES
#define BMS_GPIO_EVERY_N_CYCLES 2U
#endif

/** ADC_FILTER_SOC of on-demand conversions: 1 -> TCYCLEADC_001 (1.16 ms, TDATA_READY 1.34 ms). */
#ifndef BMS_ADC_FILTER_SOC
#define BMS_ADC_FILTER_SOC 1U
#endif

/** Wait between start of conversion and first read [ms] (>= TDATA_READY + GPIO step). */
#ifndef BMS_CONV_WAIT_MS
#define BMS_CONV_WAIT_MS 3U
#endif

/** CommTimeout of the slaves: 0 = 32 ms, 1 = 256 ms, 2 = 1024 ms, 3 = 2048 ms.
 *  Must be much longer than BMS_MEAS_PERIOD_MS. Use 3 when debugging with breakpoints. */
#ifndef BMS_SLAVE_COMM_TIMEOUT
#define BMS_SLAVE_COMM_TIMEOUT 1U
#endif

/** Attempts per register read before declaring the slave unreachable in this cycle. */
#ifndef BMS_READ_RETRIES
#define BMS_READ_RETRIES 3U
#endif

/** Minimum time between two recovery (re-initialisation) attempts of the chain [ms]. */
#ifndef BMS_RECOVERY_PERIOD_MS
#define BMS_RECOVERY_PERIOD_MS 1000U
#endif

/* =============================== Thresholds ============================== */

#ifndef BMS_CELL_OV_MV
#define BMS_CELL_OV_MV 4200 /**< cell over-voltage [mV] */
#endif
#ifndef BMS_CELL_UV_MV
#define BMS_CELL_UV_MV 2800 /**< cell under-voltage [mV] (check the cell datasheet) */
#endif
#ifndef BMS_CELL_OT_DC
#define BMS_CELL_OT_DC 600 /**< over-temperature [0.1 degC] */
#endif
#ifndef BMS_CELL_UT_DC
#define BMS_CELL_UT_DC (-200) /**< under-temperature [0.1 degC] */
#endif

/** NTC voltage outside this window = sensor open / short [mV]. VTREF is 4.8..5.1 V
 *  (DS 4.8), so an open NTC reads at least 4.8 V: the open threshold must stay
 *  below it. With the NTC curve of the BMS LV 4750 mV is about -17 degC, i.e. the
 *  usable range is about -17..+110 degC (colder is reported as an NTC fault). */
#define BMS_NTC_OPEN_MV  4750
#define BMS_NTC_SHORT_MV 100

/** Hardware thresholds programmed in every L9963E (second protection level). */
#define BMS_HW_OV_MV 4300
#define BMS_HW_UV_MV 2500

/* Fault times [ms] (FS rules: the AMS must open the shutdown circuit within 500 ms
 * for voltage and 1 s for temperature). They are counted from the last sample that
 * was in range (see bms_monitor.c), so the worst case reaction time from the real
 * onset is FAULT_TIME + 10 ms, independent of the measurement period and of lost
 * cycles. A condition must be seen in every sample taken in that window. */
#ifndef BMS_FAULT_TIME_VOLTAGE_MS
#define BMS_FAULT_TIME_VOLTAGE_MS 400U
#endif
#ifndef BMS_FAULT_TIME_TEMP_MS
#define BMS_FAULT_TIME_TEMP_MS 800U
#endif
/** cells not refreshed for this long = communication fault (from the last good read) */
#ifndef BMS_FAULT_TIME_COMM_MS
#define BMS_FAULT_TIME_COMM_MS 400U
#endif
/** temperatures not refreshed for this long = fault (from the last good read) */
#ifndef BMS_FAULT_TIME_TEMP_STALE_MS
#define BMS_FAULT_TIME_TEMP_STALE_MS 800U
#endif
/** after power-on the chain has this long to become ready before a comm fault [ms] */
#ifndef BMS_STARTUP_GRACE_MS
#define BMS_STARTUP_GRACE_MS 3000U
#endif
/** 1 = an open/shorted NTC is an AMS fault (temperature not measurable). */
#ifndef BMS_NTC_FAULT_IS_AMS_FAULT
#define BMS_NTC_FAULT_IS_AMS_FAULT 1
#endif

/* ================================ Features =============================== */

/** Run the relay state machine of bms_hv_fsm.c. Kept off: its outputs drive the
 *  AIRs and the precharge relay and it has not been validated on hardware yet. */
#ifndef BMS_ENABLE_FSM
#define BMS_ENABLE_FSM 0
#endif

/** Telemetry on CAN (HVCB network, messages HVB_RX_*). */
#ifndef BMS_ENABLE_CAN
#define BMS_ENABLE_CAN 1
#endif
#ifndef BMS_CAN_BITRATE_KBPS
#define BMS_CAN_BITRATE_KBPS 500U /* 500 or 1000 */
#endif

/** Debug log on USART3 (PB10 TX, 115200 8N1). */
#ifndef BMS_ENABLE_LOG
#define BMS_ENABLE_LOG 1
#endif
#ifndef BMS_LOG_PERIOD_MS
#define BMS_LOG_PERIOD_MS 1000U
#endif

/** Independent watchdog (frozen while the core is halted by the debugger). */
#ifndef BMS_ENABLE_IWDG
#define BMS_ENABLE_IWDG 1
#endif
#define BMS_IWDG_TIMEOUT_MS 2000U

/* ================================ Checks ================================= */
#if (BMS_N_SLAVES < 1U) || (BMS_N_SLAVES > BMS_MAX_SLAVES)
#error "BMS_N_SLAVES must be between 1 and 31"
#endif
#if (BMS_SLAVE_COMM_TIMEOUT > 3U)
#error "BMS_SLAVE_COMM_TIMEOUT must be 0..3"
#endif
#if (BMS_SLAVE_COMM_TIMEOUT == 0U)
#error "CommTimeout 32 ms is shorter than the measurement period: the slaves would fall asleep"
#endif
#if (BMS_FAULT_TIME_VOLTAGE_MS + 10U >= 500U) || (BMS_FAULT_TIME_COMM_MS + 10U >= 500U)
#error "voltage reaction time must stay below 500 ms"
#endif
#if (BMS_FAULT_TIME_TEMP_MS + 10U >= 1000U) || (BMS_FAULT_TIME_TEMP_STALE_MS + 10U >= 1000U)
#error "temperature reaction time must stay below 1 s"
#endif
#if (BMS_FAULT_TIME_VOLTAGE_MS < 2U * BMS_MEAS_PERIOD_MS)
#error "the voltage fault time must cover at least 2 measurement periods (noise filter)"
#endif
#if (BMS_FAULT_TIME_TEMP_MS < 2U * BMS_MEAS_PERIOD_MS * BMS_GPIO_EVERY_N_CYCLES) || \
    (BMS_FAULT_TIME_TEMP_STALE_MS < 3U * BMS_MEAS_PERIOD_MS * BMS_GPIO_EVERY_N_CYCLES)
#error "the temperature fault times must cover at least 2 (3 for staleness) GPIO periods"
#endif

#define BMS_FW_VERSION_MAJOR 1U
#define BMS_FW_VERSION_MINOR 0U
#define BMS_FW_VERSION_PATCH 0U

#endif /* BMS_CONFIG_H */
