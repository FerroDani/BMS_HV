/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you
 * think this stuff is worth it, you can buy us a beer in return.
 *
 * Authors
 * - Federico Carbone [federico.carbone.sc@gmail.com]
 *
 * BMS HV: interface of the L9963E library towards the two L9963T of the master.
 * Timing requirements handled here (L9963T DS13590 rev 4, digital inputs):
 *  - TXEN / ISOFREQ are latched on the NCS falling edge and must be stable for
 *    at least TTXEN_DEGLITCH / TISOFREQ_DEGLITCH (687.5 ns max) before it
 *    -> after every change of TXEN or ISOFREQ we wait >= 1 us;
 *  - NCS minimum high time Tonncs = 300 ns -> short wait after NCS rising edge.
 * SPI reads send 0x00 on MOSI: an all-zero frame fails the CRC, so even if TXEN
 * were latched high nothing meaningful could be forwarded on the isoline.
 */

#include "stm32_if.h"

#include "main.h"
#include "spi.h"

/* TTXEN_DEGLITCH/TISOFREQ_DEGLITCH (687.5 ns max) + input RC filter (~0.7 us) + setup */
#define IF_SETTLE_US 2U

static const uint8_t if_zeros[8] = {0};

void IF_DelayUs(uint32_t us) {
    /* >= 3 core cycles per iteration -> the wait is never shorter than requested */
    volatile uint32_t n = us * (SystemCoreClock / 1000000U) / 3U + 1U;
    while (n--) {
    }
}

__weak void bms_platform_watchdog_kick(void) {
}

void IF_Init(void) {
    /* BNE inputs with pull-down: with the transceiver unpowered or its output in HiZ
     * the line must read "buffer empty", never a floating "not empty". The CubeMX
     * project (.ioc) configures them without pull: keep this here or update the .ioc. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode             = GPIO_MODE_INPUT;
    gpio.Pull             = GPIO_PULLDOWN;
    gpio.Pin              = L9963TH_BNE_GPIO_IN_Pin;
    HAL_GPIO_Init(L9963TH_BNE_GPIO_IN_GPIO_Port, &gpio);
    gpio.Pin = L9963TL_BNE_GPIO_IN_Pin;
    HAL_GPIO_Init(L9963TL_BNE_GPIO_IN_GPIO_Port, &gpio);
}

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} if_pin_t;

typedef struct {
    if_pin_t cs, txen, bne, isofreq, dis;
    SPI_HandleTypeDef *hspi;
} if_trx_t;

static const if_trx_t trx_h = {
    .cs      = {L9963TH_NCS_GPIO_OUT_GPIO_Port, L9963TH_NCS_GPIO_OUT_Pin},
    .txen    = {L9963TH_TXEN_GPIO_OUT_GPIO_Port, L9963TH_TXEN_GPIO_OUT_Pin},
    .bne     = {L9963TH_BNE_GPIO_IN_GPIO_Port, L9963TH_BNE_GPIO_IN_Pin},
    .isofreq = {L9963TH_ISOFREQ_GPIO_OUT_GPIO_Port, L9963TH_ISOFREQ_GPIO_OUT_Pin},
    .dis     = {L9963TH_DIS_GPIO_INOUT_GPIO_Port, L9963TH_DIS_GPIO_INOUT_Pin},
    .hspi    = &hspi3,
};

static const if_trx_t trx_l = {
    .cs      = {L9963TL_NCS_GPIO_OUT_GPIO_Port, L9963TL_NCS_GPIO_OUT_Pin},
    .txen    = {L9963TL_TXEN_GPIO_OUT_GPIO_Port, L9963TL_TXEN_GPIO_OUT_Pin},
    .bne     = {L9963TL_BNE_GPIO_IN_GPIO_Port, L9963TL_BNE_GPIO_IN_Pin},
    .isofreq = {L9963TL_ISOFREQ_GPIO_OUT_GPIO_Port, L9963TL_ISOFREQ_GPIO_OUT_Pin},
    .dis     = {L9963TL_DIS_GPIO_INOUT_GPIO_Port, L9963TL_DIS_GPIO_INOUT_Pin},
    .hspi    = &hspi2,
};

static const if_pin_t *if_get_pin(const if_trx_t *t, L9963E_IF_PINS pin) {
    switch (pin) {
        case L9963E_IF_CS:
            return &t->cs;
        case L9963E_IF_TXEN:
            return &t->txen;
        case L9963E_IF_BNE:
            return &t->bne;
        case L9963E_IF_ISOFREQ:
            return &t->isofreq;
        case L9963E_IF_DIS:
            return &t->dis;
        default:
            return NULL;
    }
}

static L9963E_IF_PinState if_read(const if_trx_t *t, L9963E_IF_PINS pin) {
    const if_pin_t *p = if_get_pin(t, pin);
    if (p == NULL) {
        return L9963E_IF_GPIO_PIN_RESET;
    }
    return HAL_GPIO_ReadPin(p->port, p->pin) == GPIO_PIN_RESET ? L9963E_IF_GPIO_PIN_RESET : L9963E_IF_GPIO_PIN_SET;
}

static L9963E_StatusTypeDef if_write(const if_trx_t *t, L9963E_IF_PINS pin, L9963E_IF_PinState state) {
    const if_pin_t *p = if_get_pin(t, pin);
    if (p == NULL || pin == L9963E_IF_BNE) {
        return L9963E_ERROR; /* BNE is an input */
    }

    GPIO_PinState new_state = (state == L9963E_IF_GPIO_PIN_RESET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    GPIO_PinState old_state = (p->port->ODR & p->pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(p->port, p->pin, new_state);

    if ((pin == L9963E_IF_TXEN || pin == L9963E_IF_ISOFREQ) && new_state != old_state) {
        IF_DelayUs(IF_SETTLE_US); /* deglitch filter of the L9963T */
    } else if (pin == L9963E_IF_CS && new_state == GPIO_PIN_SET) {
        IF_DelayUs(IF_SETTLE_US); /* NCS minimum high time */
    }

    return L9963E_OK;
}

static L9963E_StatusTypeDef if_map_hal(HAL_StatusTypeDef errorcode) {
    switch (errorcode) {
        case HAL_OK:
            return L9963E_OK;
        case HAL_TIMEOUT:
            return L9963E_TIMEOUT;
        default:
            return L9963E_ERROR;
    }
}

static L9963E_StatusTypeDef if_receive(const if_trx_t *t, uint8_t *data, uint8_t size, uint8_t timeout_ms) {
    if (size > sizeof(if_zeros)) {
        return L9963E_ERROR;
    }
    return if_map_hal(HAL_SPI_TransmitReceive(t->hspi, (uint8_t *)if_zeros, data, size, timeout_ms));
}

static L9963E_StatusTypeDef if_transmit(const if_trx_t *t, uint8_t *data, uint8_t size, uint8_t timeout_ms) {
    return if_map_hal(HAL_SPI_Transmit(t->hspi, data, size, timeout_ms));
}

/* ---------------- L9963TH ---------------- */
L9963E_IF_PinState L9963TH_GPIO_ReadPin(L9963E_IF_PINS pin) {
    return if_read(&trx_h, pin);
}
L9963E_StatusTypeDef L9963TH_GPIO_WritePin(L9963E_IF_PINS pin, L9963E_IF_PinState state) {
    return if_write(&trx_h, pin, state);
}
L9963E_StatusTypeDef L9963TH_SPI_Receive(uint8_t *data, uint8_t size, uint8_t timeout_ms) {
    return if_receive(&trx_h, data, size, timeout_ms);
}
L9963E_StatusTypeDef L9963TH_SPI_Transmit(uint8_t *data, uint8_t size, uint8_t timeout_ms) {
    return if_transmit(&trx_h, data, size, timeout_ms);
}

/* ---------------- L9963TL ---------------- */
L9963E_IF_PinState L9963TL_GPIO_ReadPin(L9963E_IF_PINS pin) {
    return if_read(&trx_l, pin);
}
L9963E_StatusTypeDef L9963TL_GPIO_WritePin(L9963E_IF_PINS pin, L9963E_IF_PinState state) {
    return if_write(&trx_l, pin, state);
}
L9963E_StatusTypeDef L9963TL_SPI_Receive(uint8_t *data, uint8_t size, uint8_t timeout_ms) {
    return if_receive(&trx_l, data, size, timeout_ms);
}
L9963E_StatusTypeDef L9963TL_SPI_Transmit(uint8_t *data, uint8_t size, uint8_t timeout_ms) {
    return if_transmit(&trx_l, data, size, timeout_ms);
}

uint32_t GetTickMs(void) {
    return HAL_GetTick();
}

void DelayMs(uint32_t delay) {
    bms_platform_watchdog_kick();
    HAL_Delay(delay);
    bms_platform_watchdog_kick();
}

const L9963E_IfTypeDef L9963TH_interface = {
    .L9963E_IF_GPIO_ReadPin  = L9963TH_GPIO_ReadPin,
    .L9963E_IF_GPIO_WritePin = L9963TH_GPIO_WritePin,
    .L9963E_IF_SPI_Receive   = L9963TH_SPI_Receive,
    .L9963E_IF_SPI_Transmit  = L9963TH_SPI_Transmit,
    .L9963E_IF_GetTickMs     = GetTickMs,
    .L9963E_IF_DelayMs       = DelayMs,
};

const L9963E_IfTypeDef L9963TL_interface = {
    .L9963E_IF_GPIO_ReadPin  = L9963TL_GPIO_ReadPin,
    .L9963E_IF_GPIO_WritePin = L9963TL_GPIO_WritePin,
    .L9963E_IF_SPI_Receive   = L9963TL_SPI_Receive,
    .L9963E_IF_SPI_Transmit  = L9963TL_SPI_Transmit,
    .L9963E_IF_GetTickMs     = GetTickMs,
    .L9963E_IF_DelayMs       = DelayMs,
};
