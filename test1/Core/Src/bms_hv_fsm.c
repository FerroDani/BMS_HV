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

#include "bms_hv_fsm.h"
#ifndef __weak
#define __weak __attribute__((weak))
#endif  // __weak

// How do we actually reset an error?

// Updated in data_reading_timebase
extern float vbattery_monitor;
extern float vbattery_sum;

// Variables
uint8_t air_neg_int_state_closed, air_pos_int_state_closed, air_pos_mech_state_open, air_neg_mech_state_open,
 imd_error, dcbus_overvoltage, nstg_dcbus_overvoltage, ams_error;

extern uint8_t charge_cmd, drive_cmd, balancing_cmd;

variables_t variables;

static uint32_t resetting_error_entry_time = 0;
static ActiveMode_TypeDef active_mode   = IDLE_MODE;

// Variable Update function
STMLIBS_StatusTypeDef variables_update(variables_t *variables) {
    variables->air_neg_int_state_closed = AIRs_Neg_Int_Closed();
    variables->air_pos_int_state_closed = AIRs_Pos_Int_Closed();
    variables->air_pos_mech_state_open  = AIRs_Pos_Mech_Open();
    variables->air_neg_mech_state_open  = AIRs_Neg_Mech_Open();
    variables->dcbus_overvoltage        = DCBUS_Over60V();
    variables->nstg_dcbus_overvoltage   = STG_DCBUS_Over60V();
    variables->vbattery_monitor         = vbattery_monitor;
    variables->vbattery_sum             = vbattery_sum;
    variables -> charge_cmd             = charge_cmd;
    variables -> drive_cmd              = drive_cmd;
    variables -> balancing_cmd          = balancing_cmd;
    variables -> prch_closed = PRCH_closed();
    variables -> imd_error = imd_error;
    variables -> ams_error = ams_error;
    variables -> dcbus_rly_implausibility = DCBUS_Rly_Implausibility();

    return STMLIBS_OK;
}

void run_callback_1(uint32_t state) {
    ((void)0U);
}

void transition_callback_1(uint32_t state) {
    ((void)0U);
}

extern uint8_t volatile error_code;

// Is AIRS closed function
static uint8_t is_AIRs_closed(){
    return (variables.air_neg_int_state_closed && variables.air_pos_int_state_closed && \
            !variables.air_pos_mech_state_open && !variables.air_neg_mech_state_open);
}

static uint8_t ams_error_present(){

    return (variables.ams_error || variables.dcbus_overvoltage || variables.nstg_dcbus_overvoltage \
        || vbattery_monitor < 0 || vbattery_monitor > MAX_VOLTAGE || variables.dcbus_rly_implausibility);
}

static uint8_t is_AIRs_open(){
    return (!variables.air_neg_int_state_closed && !variables.air_pos_int_state_closed && \
        variables.air_pos_mech_state_open && variables.air_neg_mech_state_open);
}



// Private wrapper function signatures

uint32_t _FSM_BMS_HV_active_idle_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_active_idle_do_work();

uint32_t _FSM_BMS_HV_balancing_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_balancing_do_work();

uint32_t _FSM_BMS_HV_charging_idle_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_charging_idle_do_work();

uint32_t _FSM_BMS_HV_driving_idle_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_driving_idle_do_work();

uint32_t _FSM_BMS_HV_resetting_airs_precharge_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_resetting_airs_precharge_do_work();

uint32_t _FSM_BMS_HV_resetting_errors_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_resetting_errors_do_work();

uint32_t _FSM_BMS_HV_closing_air_neg_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_closing_air_neg_do_work();

uint32_t _FSM_BMS_HV_closing_precharge_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_closing_precharge_do_work();

uint32_t _FSM_BMS_HV_precharge_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_precharge_do_work();

uint32_t _FSM_BMS_HV_closing_air_pos_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_closing_air_pos_do_work();

uint32_t _FSM_BMS_HV_opening_precharge_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_opening_precharge_do_work();

uint32_t _FSM_BMS_HV_driving_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_driving_do_work();

uint32_t _FSM_BMS_HV_charging_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_charging_do_work();

uint32_t _FSM_BMS_HV_ams_imd_error_event_handle(uint8_t event);
uint32_t _FSM_BMS_HV_ams_imd_error_do_work();

FSM_StateTypeDef state_table[_FSM_BMS_HV_STATE_COUNT] = {
    [FSM_BMS_HV_active_idle] =
        {
            .event_handler = _FSM_BMS_HV_active_idle_event_handle,
            .entry         = FSM_BMS_HV_active_idle_entry,
            .do_work       = _FSM_BMS_HV_active_idle_do_work,
            .exit          = FSM_BMS_HV_active_idle_exit,
        },
    [FSM_BMS_HV_balancing] =
        {
            .event_handler = _FSM_BMS_HV_balancing_event_handle,
            .entry         = FSM_BMS_HV_balancing_entry,
            .do_work       = _FSM_BMS_HV_balancing_do_work,
            .exit          = FSM_BMS_HV_balancing_exit,
        },
    [FSM_BMS_HV_charging_idle] =
        {
            .event_handler = _FSM_BMS_HV_charging_idle_event_handle,
            .entry         = FSM_BMS_HV_charging_idle_entry,
            .do_work       = _FSM_BMS_HV_charging_idle_do_work,
            .exit          = FSM_BMS_HV_charging_idle_exit,
        },
    [FSM_BMS_HV_driving_idle] =
        {
            .event_handler = _FSM_BMS_HV_driving_idle_event_handle,
            .entry         = FSM_BMS_HV_driving_idle_entry,
            .do_work       = _FSM_BMS_HV_driving_idle_do_work,
            .exit          = FSM_BMS_HV_driving_idle_exit,
        },
    [FSM_BMS_HV_resetting_airs_precharge] =
        {
            .event_handler = _FSM_BMS_HV_resetting_airs_precharge_event_handle,
            .entry         = FSM_BMS_HV_resetting_airs_precharge_entry,
            .do_work       = _FSM_BMS_HV_resetting_airs_precharge_do_work,
            .exit          = FSM_BMS_HV_resetting_airs_precharge_exit,
        },
    [FSM_BMS_HV_resetting_errors] =
        {
            .event_handler = _FSM_BMS_HV_resetting_errors_event_handle,
            .entry         = FSM_BMS_HV_resetting_errors_entry,
            .do_work       = _FSM_BMS_HV_resetting_errors_do_work,
            .exit          = FSM_BMS_HV_resetting_errors_exit,
        },
    [FSM_BMS_HV_closing_air_neg] =
        {
            .event_handler = _FSM_BMS_HV_closing_air_neg_event_handle,
            .entry         = FSM_BMS_HV_closing_air_neg_entry,
            .do_work       = _FSM_BMS_HV_closing_air_neg_do_work,
            .exit          = FSM_BMS_HV_closing_air_neg_exit,
        },
    [FSM_BMS_HV_closing_precharge] =
        {
            .event_handler = _FSM_BMS_HV_closing_precharge_event_handle,
            .entry         = FSM_BMS_HV_closing_precharge_entry,
            .do_work       = _FSM_BMS_HV_closing_precharge_do_work,
            .exit          = FSM_BMS_HV_closing_precharge_exit,
        },
    [FSM_BMS_HV_precharge] =
        {
            .event_handler = _FSM_BMS_HV_precharge_event_handle,
            .entry         = FSM_BMS_HV_precharge_entry,
            .do_work       = _FSM_BMS_HV_precharge_do_work,
            .exit          = FSM_BMS_HV_precharge_exit,
        },
    [FSM_BMS_HV_closing_air_pos] =
        {
            .event_handler = _FSM_BMS_HV_closing_air_pos_event_handle,
            .entry         = FSM_BMS_HV_closing_air_pos_entry,
            .do_work       = _FSM_BMS_HV_closing_air_pos_do_work,
            .exit          = FSM_BMS_HV_closing_air_pos_exit,
        },
    [FSM_BMS_HV_opening_precharge] =
        {
            .event_handler = _FSM_BMS_HV_opening_precharge_event_handle,
            .entry         = FSM_BMS_HV_opening_precharge_entry,
            .do_work       = _FSM_BMS_HV_opening_precharge_do_work,
            .exit          = FSM_BMS_HV_opening_precharge_exit,
        },
    [FSM_BMS_HV_driving] =
        {
            .event_handler = _FSM_BMS_HV_driving_event_handle,
            .entry         = FSM_BMS_HV_driving_entry,
            .do_work       = _FSM_BMS_HV_driving_do_work,
            .exit          = FSM_BMS_HV_driving_exit,
        },
    [FSM_BMS_HV_charging] =
        {
            .event_handler = _FSM_BMS_HV_charging_event_handle,
            .entry         = FSM_BMS_HV_charging_entry,
            .do_work       = _FSM_BMS_HV_charging_do_work,
            .exit          = FSM_BMS_HV_charging_exit,
        },
    [FSM_BMS_HV_ams_imd_error] =
        {
            .event_handler = _FSM_BMS_HV_ams_imd_error_event_handle,
            .entry         = FSM_BMS_HV_ams_imd_error_entry,
            .do_work       = _FSM_BMS_HV_ams_imd_error_do_work,
            .exit          = FSM_BMS_HV_ams_imd_error_exit,
        },
};

FSM_ConfigTypeDef config = {
    .state_length = _FSM_BMS_HV_STATE_COUNT,
    .state_table  = state_table,
};

STMLIBS_StatusTypeDef FSM_BMS_HV_init(FSM_HandleTypeDef *handle,
                                      uint8_t event_count,
                                      FSM_callback_function run_callback,
                                      FSM_callback_function transition_callback) {
    return FSM_init(handle, &config, event_count, run_callback, transition_callback);
}

// State control functions

// State Entry Actions (User defined)
void FSM_BMS_HV_resetting_errors_entry() {
    resetting_error_entry_time = HAL_GetTick();
    // Reset AMS Error
    Reset_AMS_Error();
    ams_error = RESET;
    return;
}
void FSM_BMS_HV_close_air_neg_entry() {
    // Close Negative Air
    Close_Air_Neg();
    return;
}
void FSM_BMS_HV_close_air_pos_entry() {
    // Close Negative Air
    Close_Air_Pos();
    return;
}
void FSM_BMS_HV_close_precharge_entry() {
    // Open Precharge
    Close_Precharge();
    return;
}
void FSM_BMS_HV_open_precharge_entry() {
    // Open Precharge
    Open_Precharge();
    return;
}
void FSM_BMS_HV_resetting_airs_precharge_entry() {
    Open_Precharge();
    Open_Air_Pos();
    Open_Air_Neg();
    return;
}
void FSM_BMS_HV_ams_imd_error_entry() {
    // Open Precharge
    SDC_On(); // Change to SDC open and SDC Close
    // When we close the SDC car can work
    Err_LED_On();
    return;
}
/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_active_idle_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_active_idle_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_active_idle:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_balancing:
        case FSM_BMS_HV_charging_idle:
        case FSM_BMS_HV_driving_idle:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}
// USER CODE OF ACTIVE IDLE DO WORK
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_active_idle_do_work() {

    variables_update(&variables);

    // Reset the active mode
    active_mode = IDLE_MODE;

    if (ams_error_present()) {
        ams_error = SET;
        Set_AMS_Error();
        return FSM_BMS_HV_ams_imd_error;
    }
    if (variables.imd_error) return FSM_BMS_HV_ams_imd_error;

    if (variables.charge_cmd)   return FSM_BMS_HV_charging_idle;

    if (variables.drive_cmd)    return FSM_BMS_HV_driving_idle;
    
    if (variables.balancing_cmd)    return FSM_BMS_HV_balancing;

    return FSM_BMS_HV_active_idle;
}
/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_active_idle_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_active_idle_do_work();

    switch (next) {
        case FSM_BMS_HV_active_idle:  // Added reentrance for current state
        case FSM_BMS_HV_balancing:
        case FSM_BMS_HV_charging_idle:
        case FSM_BMS_HV_driving_idle:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_balancing_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_balancing_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_balancing:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_active_idle:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}
// USER CODE OF BALANCING DO WORK
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_balancing_do_work() {
    variables_update(&variables);

    if (ams_error_present()){
        ams_error = SET;
        Set_AMS_Error();
        return FSM_BMS_HV_ams_imd_error;
    }

    if (variables.imd_error) return FSM_BMS_HV_ams_imd_error;

    if (L9963E_utils_balance_cells() != L9963_UTILS_OK) {
        balancing_cmd = RESET;
        return FSM_BMS_HV_ams_imd_error;
        }

    balancing_cmd = RESET;
    return FSM_BMS_HV_active_idle;
}
/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_balancing_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_balancing_do_work();

    switch (next) {
        case FSM_BMS_HV_balancing:  // Added reentrance for current state
        case FSM_BMS_HV_active_idle:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_charging_idle_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_charging_idle_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_charging_idle:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_active_idle:
        case FSM_BMS_HV_closing_air_neg:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}
// USER CODE OF CHARGING IDLE DO WORK
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_charging_idle_do_work() {
    variables_update(&variables);

    active_mode = CHARGING_MODE;
    
    if (ams_error_present()){
        ams_error = SET;
        Set_AMS_Error();
        return FSM_BMS_HV_ams_imd_error;
    }

    if (variables.imd_error) return FSM_BMS_HV_ams_imd_error;

    if (variables.charge_cmd == 0) {
        return FSM_BMS_HV_active_idle;
    }

    return FSM_BMS_HV_closing_air_neg;
}
/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_charging_idle_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_charging_idle_do_work();

    switch (next) {
        case FSM_BMS_HV_charging_idle:  // Added reentrance for current state
        case FSM_BMS_HV_active_idle:
        case FSM_BMS_HV_closing_air_neg:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_driving_idle_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_driving_idle_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_driving_idle:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_active_idle:
        case FSM_BMS_HV_closing_air_neg:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}
// USER CODE OF DRIVING IDLE DO WORK
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_driving_idle_do_work() {
    variables_update(&variables);
    active_mode = DRIVING_MODE;

    if (ams_error_present()){
        ams_error = SET;
        Set_AMS_Error();
        return FSM_BMS_HV_ams_imd_error;
    }

    if (variables.imd_error) return FSM_BMS_HV_ams_imd_error;

    if (variables.drive_cmd == 0) {
        return FSM_BMS_HV_active_idle;
    }

    return FSM_BMS_HV_closing_air_neg;
}
/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_driving_idle_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_driving_idle_do_work();

    switch (next) {
        case FSM_BMS_HV_driving_idle:  // Added reentrance for current state
        case FSM_BMS_HV_active_idle:
        case FSM_BMS_HV_closing_air_neg:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_resetting_airs_precharge_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_resetting_airs_precharge_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_resetting_airs_precharge:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_active_idle:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}
// USER CODE FOR RESETTING_AIRS_DO_WORK
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_resetting_airs_precharge_do_work(){
    variables_update(&variables);

    if (ams_error_present()){
        ams_error = SET;
        Set_AMS_Error();
        return FSM_BMS_HV_ams_imd_error;
    }

    if (variables.imd_error) return FSM_BMS_HV_ams_imd_error;

    if (is_AIRs_open() && !PRCH_closed()){
        // If AIRs and precharge are open 
        return  FSM_BMS_HV_active_idle;
    }

    return FSM_BMS_HV_resetting_airs_precharge;
}
/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_resetting_airs_precharge_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_resetting_airs_precharge_do_work();

    switch (next) {
        case FSM_BMS_HV_resetting_airs_precharge:  // Added reentrance for current state
        case FSM_BMS_HV_active_idle:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_resetting_errors_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_resetting_errors_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_resetting_errors:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_active_idle:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}
// Resetting errors do_work
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_resetting_errors_do_work() {
    if (HAL_GetTick() - resetting_error_entry_time > 10000) {
        return FSM_BMS_HV_active_idle;
    }

    return FSM_BMS_HV_resetting_errors;
}
/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_resetting_errors_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_resetting_errors_do_work();

    switch (next) {
        case FSM_BMS_HV_resetting_errors:  // Added reentrance for current state
        case FSM_BMS_HV_active_idle:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_closing_air_neg_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_closing_air_neg_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_closing_air_neg:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_closing_precharge:
        case FSM_BMS_HV_resetting_airs_precharge:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}
// User defined closing_air_neg_do_work
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_closing_air_neg_do_work() {
    variables_update(&variables);
    
    if (ams_error_present()){
        ams_error = SET;
        Set_AMS_Error();
        return FSM_BMS_HV_ams_imd_error;
    }

    if (variables.imd_error) return FSM_BMS_HV_ams_imd_error;

    // Check if air negative is closed
    if ( AIRs_Neg_Int_Closed() && !AIRs_Neg_Mech_Open() ) {
        return FSM_BMS_HV_closing_precharge;

    } else {
        // If air negative is not closed, invoke the error
        return FSM_BMS_HV_resetting_airs_precharge;
    }

    return FSM_BMS_HV_closing_air_neg;
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_closing_air_neg_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_closing_air_neg_do_work();

    switch (next) {
        case FSM_BMS_HV_closing_air_neg:  // Added reentrance for current state
        case FSM_BMS_HV_closing_precharge:
        case FSM_BMS_HV_resetting_airs_precharge:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_closing_precharge_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_closing_precharge_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_closing_precharge:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_precharge:
        case FSM_BMS_HV_resetting_airs_precharge:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}
// User defined closing_precharge_do_work
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_closing_precharge_do_work() {
    variables_update(&variables);

    if (ams_error_present()){
        ams_error = SET;
        Set_AMS_Error();
        return FSM_BMS_HV_ams_imd_error;
    }

    if (variables.imd_error) return FSM_BMS_HV_ams_imd_error;

    // Check if precharge is closed
    if ( PRCH_closed() ) {
        return FSM_BMS_HV_precharge;
    } else {
        // If precharge is not closed, invoke the error
        return FSM_BMS_HV_resetting_airs_precharge;
    }
}
/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_closing_precharge_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_closing_precharge_do_work();

    switch (next) {
        case FSM_BMS_HV_closing_precharge:  // Added reentrance for current state
        case FSM_BMS_HV_precharge:
        case FSM_BMS_HV_resetting_airs_precharge:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_precharge_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_precharge_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_precharge:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_closing_air_pos:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}
// User defined precharge_do_work
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_precharge_do_work() {
    variables_update(&variables);

    if (ams_error_present()){
        ams_error = SET;
        Set_AMS_Error();
        return FSM_BMS_HV_ams_imd_error;
    }

    if (variables.imd_error) return FSM_BMS_HV_ams_imd_error;

    // How to read DCBUS voltage??
    // if (variables.vbattery_monitor > MAX_VOLTAGE * 0.95) {
    //     return FSM_BMS_HV_closing_air_pos;
    // }

    return FSM_BMS_HV_precharge;
}
/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_precharge_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_precharge_do_work();

    switch (next) {
        case FSM_BMS_HV_precharge:  // Added reentrance for current state
        case FSM_BMS_HV_closing_air_pos:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_closing_air_pos_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_closing_air_pos_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_closing_air_pos:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_opening_precharge:
        case FSM_BMS_HV_resetting_airs_precharge:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

FSM_BMS_HV_StateTypeDef FSM_BMS_HV_closing_air_pos_do_work(){
    variables_update(&variables);
    
    if (ams_error_present()){
        ams_error = SET;
        Set_AMS_Error();
        return FSM_BMS_HV_ams_imd_error;
    }

    if (variables.imd_error) return FSM_BMS_HV_ams_imd_error;

    // Check if air positive is closed
    if ( AIRs_Pos_Int_Closed() && !AIRs_Pos_Mech_Open() ) {
        return FSM_BMS_HV_opening_precharge;

    } else {
        // If air negative is not closed, invoke the error
        return FSM_BMS_HV_resetting_airs_precharge;
    }

}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_closing_air_pos_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_closing_air_pos_do_work();

    switch (next) {
        case FSM_BMS_HV_closing_air_pos:  // Added reentrance for current state
        case FSM_BMS_HV_opening_precharge:
        case FSM_BMS_HV_resetting_airs_precharge:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_opening_precharge_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_opening_precharge_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_opening_precharge:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_driving:
        case FSM_BMS_HV_charging:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

// USER DEFINED DO_WORK
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_opening_precharge_do_work(){
    variables_update(&variables);
    
    if (ams_error_present()){
        ams_error = SET;
        Set_AMS_Error();
        return FSM_BMS_HV_ams_imd_error;
    }

    if (variables.imd_error) return FSM_BMS_HV_ams_imd_error;

    if (PRCH_closed() == 0){
        if (active_mode == DRIVING_MODE) return FSM_BMS_HV_driving;
        if (active_mode == CHARGING_MODE) return FSM_BMS_HV_charging;
        return FSM_BMS_HV_resetting_airs_precharge; // If mode is undefined i hope it doesn happen
    }
    
    return FSM_BMS_HV_opening_precharge;
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_opening_precharge_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_opening_precharge_do_work();

    switch (next) {
        case FSM_BMS_HV_opening_precharge:  // Added reentrance for current state
        case FSM_BMS_HV_driving:
        case FSM_BMS_HV_charging:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_driving_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_driving_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_driving:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_resetting_airs_precharge:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}
// USER DEFINED DO_WORK (constantly check for errors and if relays are closed)
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_driving_do_work(){
    variables_update(&variables);
    
    if (ams_error_present()){
        ams_error = SET;
        Set_AMS_Error();
        return FSM_BMS_HV_ams_imd_error;
    }

    if (variables.imd_error) return FSM_BMS_HV_ams_imd_error;

    if (is_AIRs_closed() != 1) return FSM_BMS_HV_resetting_airs_precharge;

    if (variables.drive_cmd != 1) return FSM_BMS_HV_resetting_airs_precharge; // Stop driving

    return FSM_BMS_HV_driving;
}


/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_driving_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_driving_do_work();

    switch (next) {
        case FSM_BMS_HV_driving:  // Added reentrance for current state
        case FSM_BMS_HV_resetting_airs_precharge:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_charging_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_charging_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_charging:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_resetting_airs_precharge:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}
// USER CODE FOR CHARGING DO_WORK
FSM_BMS_HV_StateTypeDef FSM_BMS_HV_charging_do_work(){
    variables_update(&variables);
    
    if (ams_error_present()){
        ams_error = SET;
        Set_AMS_Error();
        return FSM_BMS_HV_ams_imd_error;
    }

    if (variables.imd_error) return FSM_BMS_HV_ams_imd_error;

    if (is_AIRs_closed() != 1) return FSM_BMS_HV_resetting_airs_precharge;

    if (variables.charge_cmd != 1) return FSM_BMS_HV_resetting_airs_precharge; // Stop charging

    return FSM_BMS_HV_charging;
}
/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_charging_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_charging_do_work();

    switch (next) {
        case FSM_BMS_HV_charging:  // Added reentrance for current state
        case FSM_BMS_HV_resetting_airs_precharge:
        case FSM_BMS_HV_ams_imd_error:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_event_handle, with exit state checking */
uint32_t _FSM_BMS_HV_ams_imd_error_event_handle(uint8_t event) {
    uint32_t next = (uint32_t)FSM_BMS_HV_ams_imd_error_event_handle(event);

    switch (next) {
        case FSM_BMS_HV_ams_imd_error:  // Reentrance is always supported on event handlers
        case FSM_BMS_HV_resetting_errors:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}

/** @brief wrapper of FSM_BMS_HV_do_work, with exit state checking */
uint32_t _FSM_BMS_HV_ams_imd_error_do_work() {
    uint32_t next = (uint32_t)FSM_BMS_HV_ams_imd_error_do_work();

    switch (next) {
        case FSM_BMS_HV_ams_imd_error:  // Added reentrance for current state
        case FSM_BMS_HV_resetting_errors:
            return next;
        default:
            return _FSM_BMS_HV_DIE;
    }
}
