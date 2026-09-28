/* host stub of main.h for the unit tests of bms_monitor.c */
#ifndef MAIN_H_STUB
#define MAIN_H_STUB
#include <stdint.h>
extern int stub_ams_pin, stub_err_led;
#define Set_AMS_Error()   (stub_ams_pin = 1)
#define Reset_AMS_Error() (stub_ams_pin = 0)
#define Err_LED_On()      (stub_err_led = 1)
#endif
