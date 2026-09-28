/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you
 * think this stuff is worth it, you can buy us a beer in return.
 *
 * Authors
 * - Federico Carbone [federico.carbone.sc@gmail.com]
 *
 * Hardware abstraction of the two L9963T transceivers of the BMS HV master:
 *  - L9963TH: SPI3, NCS PB5, TXEN PD1, ISOFREQ PD2, BNE PD3, DIS PD4  (bottom of the ring)
 *  - L9963TL: SPI2, NCS PA8, BNE PA9, TXEN PA10, ISOFREQ PC8, DIS PC9 (top of the ring)
 */

#ifndef STM32_IF_H
#define STM32_IF_H

#include "L9963E_interface.h"

/* L9963TH (SPI3) */
L9963E_IF_PinState L9963TH_GPIO_ReadPin(L9963E_IF_PINS pin);
L9963E_StatusTypeDef L9963TH_GPIO_WritePin(L9963E_IF_PINS pin, L9963E_IF_PinState state);
L9963E_StatusTypeDef L9963TH_SPI_Receive(uint8_t *data, uint8_t size, uint8_t timeout_ms);
L9963E_StatusTypeDef L9963TH_SPI_Transmit(uint8_t *data, uint8_t size, uint8_t timeout_ms);

/* L9963TL (SPI2) */
L9963E_IF_PinState L9963TL_GPIO_ReadPin(L9963E_IF_PINS pin);
L9963E_StatusTypeDef L9963TL_GPIO_WritePin(L9963E_IF_PINS pin, L9963E_IF_PinState state);
L9963E_StatusTypeDef L9963TL_SPI_Receive(uint8_t *data, uint8_t size, uint8_t timeout_ms);
L9963E_StatusTypeDef L9963TL_SPI_Transmit(uint8_t *data, uint8_t size, uint8_t timeout_ms);

uint32_t GetTickMs(void);
void DelayMs(uint32_t delay);

/** Pin setup not expressed in the CubeMX project (BNE pull-downs). */
void IF_Init(void);

/** busy wait of at least @p us microseconds (no timer needed) */
void IF_DelayUs(uint32_t us);

/** Watchdog refresh hook, called by DelayMs() (weak default in stm32_if.c,
 *  implemented by bms_app.c). */
void bms_platform_watchdog_kick(void);

extern const L9963E_IfTypeDef L9963TH_interface;
extern const L9963E_IfTypeDef L9963TL_interface;

#endif  //STM32_IF_H
