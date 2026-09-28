/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you
 * think this stuff is worth it, you can buy us a beer in return.
 *
 * Authors
 * - Filippo Rossi <filippo.rossi.sc@gmail.com>
 * - Federico Carbone <federico.carbone.sc@gmail.com>
 */

#ifndef FSM_BMS_HV_H
#define FSM_BMS_HV_H
#define MAX_VOLTAGE ((uint16_t)600)

#include "L9963E.h"
#include "L9963_utils.h"
#include "L9963E_drv.h"
#include "fsm.h"
#include "main.h"

void run_callback_1(uint32_t state);
void transition_callback_1(uint32_t state);


enum FSM_BMS_HV_StateEnum {
    FSM_BMS_HV_active_idle              = 0,
    FSM_BMS_HV_balancing                = 1,
    FSM_BMS_HV_charging_idle            = 2,
    FSM_BMS_HV_driving_idle             = 3,
    FSM_BMS_HV_resetting_airs_precharge = 4,
    FSM_BMS_HV_resetting_errors         = 5,
    FSM_BMS_HV_closing_air_neg          = 6,
    FSM_BMS_HV_closing_precharge        = 7,
    FSM_BMS_HV_precharge                = 8,
    FSM_BMS_HV_closing_air_pos          = 9,
    FSM_BMS_HV_opening_precharge        = 10,
    FSM_BMS_HV_driving                  = 11,
    FSM_BMS_HV_charging                 = 12,
    FSM_BMS_HV_ams_imd_error            = 13,

    _FSM_BMS_HV_STATE_COUNT = 14,

    /** @brief Invalid state, leads to irrecoverable error i.e. hard fault */
    _FSM_BMS_HV_DIE = 15
};

typedef enum FSM_BMS_HV_StateEnum FSM_BMS_HV_StateTypeDef;

typedef struct {
    uint8_t air_neg_int_state_closed, air_pos_int_state_closed;
    uint8_t air_pos_mech_state_open, air_neg_mech_state_open;
    uint8_t prch_closed;
    uint8_t dcbus_rly_implausibility;
    uint8_t ams_error, imd_error;
    uint8_t dcbus_overvoltage, nstg_dcbus_overvoltage, charge_cmd, drive_cmd, balancing_cmd;
    float vbattery_monitor;
    float vbattery_sum;
} variables_t;

typedef enum ActiveMode_Enum { DRIVING_MODE = 0, CHARGING_MODE = 1, IDLE_MODE = 2 } ActiveMode_TypeDef;

STMLIBS_StatusTypeDef variables_update(variables_t *variables);
/**
 * @brief
 * @param handle FSM handle
 * @param event_count number of events
 * @param run_callback callback of a run event
 * @param transition_callback callback of a transition event
 * @return status
 */
STMLIBS_StatusTypeDef FSM_BMS_HV_init(FSM_HandleTypeDef *handle,
                                      uint8_t event_count,
                                      FSM_callback_function run_callback,
                                      FSM_callback_function transition_callback);

// State functions

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_active_idle_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_active_idle_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_active_idle_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_active_idle_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_balancing_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_balancing_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_balancing_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_balancing_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_charging_idle_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_charging_idle_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_charging_idle_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_charging_idle_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_driving_idle_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_driving_idle_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_driving_idle_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_driving_idle_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_resetting_airs_precharge_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_resetting_airs_precharge_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_resetting_airs_precharge_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_resetting_airs_precharge_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_resetting_errors_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_resetting_errors_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_resetting_errors_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_resetting_errors_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_closing_air_neg_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_closing_air_neg_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_closing_air_neg_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_closing_air_neg_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_closing_precharge_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_closing_precharge_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_closing_precharge_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_closing_precharge_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_precharge_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_precharge_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_precharge_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_precharge_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_closing_air_pos_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_closing_air_pos_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_closing_air_pos_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_closing_air_pos_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_opening_precharge_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_opening_precharge_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_opening_precharge_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_opening_precharge_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_driving_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_driving_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_driving_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_driving_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_charging_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_charging_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_charging_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_charging_exit();

/**
 * @brief
 * @param handle FSM handle
 * @param event event
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_ams_imd_error_event_handle(uint8_t event);

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_ams_imd_error_entry();

/**
 * @brief
 * @param handle FSM handle
 * @return next state
 */
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_ams_imd_error_do_work();

/**
 * @brief
 * @param handle FSM handle
 */
void FSM_BMS_HV_ams_imd_error_exit();

#endif  //FSM_BMS_HV_H