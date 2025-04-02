/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 *
 * Author: Michał Mirosław
 */
#ifndef BOARD_BAND_P2_H
#define BOARD_BAND_P2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define DEVICE_NAME		"Band_P2"
#define MANUFACTURER_NAME	"OmniChip"
#define HW_VERSION "P2"

#define BT_CLEAR_ON_REPROGRAM	0

#define LEDS_NUMBER    2

#define LED_1          23	// LED2
#define LED_2          27	// LED1

#define LEDS_ACTIVE_STATE 1

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1, LED_2 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2

#define BUTTONS_NUMBER 1

#define BUTTON_START   26
#define BUTTON_1       26
#define BUTTON_STOP    26
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define PS_HOLD_PIN    12

#define RX_PIN_NUMBER  19
#define TX_PIN_NUMBER  20
#define HWFC           false

#define SPIM1_SCK_PIN   4	// MEMS_SCK
#define SPIM1_MOSI_PIN  2	// MEMS_SDI
#define SPIM1_MISO_PIN  3	// MEMS_SDO
#define SPIM1_SS_PIN    5	// MEMS_CS
#define SENSOR_INT1	13	// MEMS_INT1
#define SENSOR_INT2	14	// MEMS_INT2
// present accel vector / rotation in ENU convention (ground vehicle)
// antenna points to +X, board elements point to +Z
#define GYRO_ORIENT	0x00	// +X,+Y,+Z
#define ACCL_ORIENT	0x38	// -X,-Y,-Z

#define TWIM0_SCL_PIN	17	// LRA_SCL
#define TWIM0_SDA_PIN	18	// LRA_SDA
#define MOTOR_TRIG	16	// LRA_TRIG
#define MOTOR_EN	15	// LRA_EN

#define MOTOR_I2C_ADDR	0x5A

#define ADC_VBAT_IN	NRF_SAADC_INPUT_AIN4	// TEST1 @P0.28
#define ADC_VBUS_IN	NRF_SAADC_INPUT_AIN5	// TEST5 @P0.29
#define ADC_R0		243
#define ADC_R1		470
#define ADC_NUM		((ADC_R0+ADC_R1)*1000*4*3/5)	// V=(R0+R1)/R0 * 4 * 0.6V * x/MAX	[gain = 1/4]
#define ADC_DENOM	ADC_R0
#define ADC_ACQTIME	NRF_SAADC_ACQTIME_10US		// Rwe <= 10 kOhm [3us; WA erratas]

#define CLK_OUT_PIN	11

#ifdef __cplusplus
}
#endif

#endif // BOARD_BAND_P2_H
