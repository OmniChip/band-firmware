/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 *
 * Author: Michał Mirosław
 */
#ifndef BOARD_BAND_DEVB_H
#define BOARD_BAND_DEVB_H

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_LSM9DS1 0

#undef SPIM1_SCK_PIN
#undef SPIM1_MOSI_PIN
#undef SPIM1_MISO_PIN
#undef SPIM1_SS_PIN

#define SPIM1_FREQ NRF_SPIM_FREQ_2M

#if !SENSOR_LSM9DS1	// LSM6DS3 + LISM3MDL

#define SPIM1_SCK_PIN	24   // SPI clock GPIO pin number.
#define SPIM1_MOSI_PIN	26   // SPI Master Out Slave In GPIO pin number.
#define SPIM1_MISO_PIN	27   // SPI Master In Slave Out GPIO pin number.
#define SPIM1_SS_PIN	23   // SPI Slave Select GPIO pin number.
#define SENSOR_INT1	22
#define SENSOR_INT2	 3	// (unused)

#define MAG_I2C_ADDR	0x1E	// SA1=1

#else	// LSM9DS1

#define SPIM1_SCK_PIN	 3
#define SPIM1_MOSI_PIN	 4
#define SPIM1_MISO_PIN	23
#define SPIM1_SS_PIN	24	// CS_AG
#define SPIM1_SSM_PIN	25	// CS_M
#define SENSOR_INT1	27	// INT1 (INT1_AG)
#define SENSOR_INT2	26	// RDY (DRDY_M)

#endif
#ifdef __cplusplus
}
#endif

#endif // BOARD_BAND_DEVB_H
