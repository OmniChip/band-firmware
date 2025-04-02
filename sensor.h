/**
 * Copyright (c) 2019, OmniChip sp. z o.o.
 */

#include "boards.h"
#ifdef BOARD_PCA10040
#include "board-band-devb.h"
#endif

#ifndef SENSOR_LSM9DS1
#define SENSOR_LSM9DS1 0
#endif

#ifndef MAG_I2C_ADDR
#define MAG_I2C_ADDR 0
#endif

#if SENSOR_LSM9DS1

# define SENSOR_ID 0x68
# ifdef SPIM1_SSM_PIN
#  define MAG_DATA_REG (0x28|0x40)
#  define MAG_ON_SPI 1
# else
#  define MAG_DATA_REG 0
#  define MAG_ON_SPI 0
#  define SPIM1_SSM_PIN 0
# endif

#else	// LSM6DS3 + LISM3MDL

# define SENSOR_ID 0x69
# define MAG_ON_SPI 0
# define SPIM1_SSM_PIN 0
# if MAG_I2C_ADDR
#  define MAG_DATA_REG 0x2E
# else
#  define MAG_DATA_REG 0
# endif

#endif

struct band_input_rec {
	uint32_t timestamp;
	int16_t gx, gy, gz, ax, ay, az;
};

struct magdata_rec {
	int16_t mx, my, mz;
	uint32_t timestamp;
} __attribute__((packed,aligned(2)));

struct band_output_rec {
	uint8_t effect;
};

struct band_feature_rec {
	uint8_t gmult:4;
	uint8_t gpower:1;
	uint8_t pad1_:3;

	uint8_t amult:3;
	uint8_t pad2_:1;
	uint8_t apower:1;
	uint8_t pad3_:3;
};

extern volatile uint32_t ts;

extern void sensor_enable(void);
extern void sensor_disable(void);

extern void reset_sensor_fifo(void);
extern struct band_input_rec *sensor_next_report(void);
extern void sensor_trigger_read_fifo(void);
extern void sensor_trigger_read_mag(void);
extern void sensor_read_mag_data(struct magdata_rec *data);

extern void hids_trigger_send(void);
extern void bands_trigger_send(void);
extern void bands_notify_magdata(void);

