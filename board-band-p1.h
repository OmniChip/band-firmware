/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 *
 * Author: Michał Mirosław
 */
#ifndef BOARD_BAND_P1_H
#define BOARD_BAND_P1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define LEDS_NUMBER    2

#define LED_START      28
#define LED_1          28
#define LED_2          29
#define LED_STOP       29

#define LEDS_ACTIVE_STATE 1

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1, LED_2 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2

#define BUTTONS_NUMBER 2

#define BUTTON_START   25
#define BUTTON_1       25
#define BUTTON_2       26
#define BUTTON_STOP    26
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1, BUTTON_2 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2

#define RX_PIN_NUMBER  19
#define TX_PIN_NUMBER  20
#define HWFC           false

#define SPIM1_SCK_PIN   4   // SPI clock GPIO pin number.
#define SPIM1_MOSI_PIN  2   // SPI Master Out Slave In GPIO pin number.
#define SPIM1_MISO_PIN  3   // SPI Master In Slave Out GPIO pin number.
#define SPIM1_SS_PIN    5   // SPI Slave Select GPIO pin number.
#define SENSOR_INT1	13
#define SENSOR_INT2	14

#ifdef __cplusplus
}
#endif

#endif // BOARD_BAND_P1_H
