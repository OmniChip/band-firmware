/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 *
 * Author: Michał Mirosław
 */

#include <stdint.h>
#include <math.h>
#include "sdk_config.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "nrf_log.h"
#include "nrfx_twim.h"
#include "nrfx_swi.h"
#include "nrf_gpio.h"
#include "sensor.h"
#include "motor.h"

#if MOTOR_I2C_ADDR && !ACCEL_TEST

// PWM mode: freq = 10-250 kHz

static const nrfx_twim_t m_twim = NRFX_TWIM_INSTANCE(0);
static const nrfx_twim_config_t twim_config = {
	.scl = TWIM0_SCL_PIN,
	.sda = TWIM0_SDA_PIN,
	.frequency = NRF_TWIM_FREQ_250K,
	.interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
};

static uint8_t play_buf[10] = { 0x04 /* WAV_FRM_SEQ[] */, 0, 0, 0, 0, 0, 0, 0, 0, 0x01 };
static uint8_t rxtx_buf[10];
static volatile uint8_t twi_done, play_pending;
static const nrfx_twim_xfer_desc_t rdb_desc = NRFX_TWIM_XFER_DESC_TXRX(MOTOR_I2C_ADDR, rxtx_buf, 1, rxtx_buf, 1);
static const nrfx_twim_xfer_desc_t wrb_desc = NRFX_TWIM_XFER_DESC_TX(MOTOR_I2C_ADDR, rxtx_buf, 2);
static const nrfx_twim_xfer_desc_t wrten_desc = NRFX_TWIM_XFER_DESC_TX(MOTOR_I2C_ADDR, rxtx_buf, 10);

__attribute__((unused))
static void do_twim_debug(void)
{
	NRF_LOG_INFO("TWIM: %u . %u %u %u %u %u",
		m_twim.p_twim->ENABLE,
		m_twim.p_twim->EVENTS_STOPPED,
		m_twim.p_twim->EVENTS_ERROR,
		m_twim.p_twim->EVENTS_SUSPENDED,
		m_twim.p_twim->EVENTS_RXSTARTED,
		m_twim.p_twim->EVENTS_TXSTARTED)
	NRF_LOG_INFO("TWIMx: %u %u . %x %x %x %x",
		m_twim.p_twim->EVENTS_LASTRX,
		m_twim.p_twim->EVENTS_LASTTX,
		m_twim.p_twim->SHORTS,
		m_twim.p_twim->INTEN,
		m_twim.p_twim->ADDRESS,
		m_twim.p_twim->FREQUENCY);
}

static void do_twim_xfer(const nrfx_twim_xfer_desc_t *desc)
{
	ret_code_t err_code;
	unsigned clks;
	bool ok;

	twi_done = false;
	err_code = nrfx_twim_xfer(&m_twim, desc, 0);
	APP_ERROR_CHECK(err_code);

//	do_twim_debug();

	clks = (2 + desc->primary_length + desc->secondary_length) * 9 + 2;
//	NRF_LOG_INFO("clks: %u", clks);
	//clks = clks * 2 + clks / 2 + 5;
	clks *= 4;
//	NRF_LOG_INFO("usec: %u", clks);
	NRFX_DELAY_US(clks);

//	do_twim_debug();

	NRFX_WAIT_FOR(twi_done, 3, clks/2, ok);
	if (!ok)
		APP_ERROR_CHECK(NRFX_ERROR_TIMEOUT);
}

static void twim_handler(nrfx_twim_evt_t const *p_event, void *p_context)
{
	ret_code_t err_code;

	if (play_pending) {
		memcpy(rxtx_buf, play_buf, 10);
		play_pending = false;

		err_code = nrfx_twim_xfer(&m_twim, &wrten_desc, 0);
		APP_ERROR_CHECK(err_code);
	} else {
		twi_done = true;
	}
}

static void motor_reg_writeb(uint8_t reg, uint8_t value)
{
	rxtx_buf[0] = reg;
	rxtx_buf[1] = value;
	do_twim_xfer(&wrb_desc);
}

static void motor_reg_writeseq(const uint8_t *reg_and_values, size_t n)
{
	nrfx_twim_xfer_desc_t desc = NRFX_TWIM_XFER_DESC_TX(MOTOR_I2C_ADDR, (void *)reg_and_values, n + 1);

	do_twim_xfer(&desc);
}

static uint8_t motor_reg_readb(uint8_t reg)
{
	rxtx_buf[0] = reg;
	do_twim_xfer(&rdb_desc);
	return rxtx_buf[0];
}

#define WAV_END		0x00
#define WAV_EFFECT(i)	((i) & 0x7F)		// 1 .. 123
#define WAV_DELAY(d)	(0x80 | ((d) & 0x7F))	// x10 ms delay

static void fill_play_buf(uint8_t *dest, const uint8_t *seq, size_t len)
{
	memcpy(dest + 1, seq, len);
	memset(dest + 1 + len, 0, 8 - len);
}

void motor_play_lib_effect(const uint8_t *seq, size_t len)
{
	ret_code_t err_code;

	if (len > 8)
		len = 8;

	nrf_gpio_pin_set(MOTOR_EN);	// FIXME: poll for done
	nrfx_twim_enable(&m_twim);

	NRF_LOG_INFO("want to play seq[%u] = { %02x ... }", len, seq[0]);

	play_pending = false;
	__asm__ volatile ("dmb" ::: "memory");

	if (!twi_done) {
		fill_play_buf(play_buf, seq, len);
		// FIXME: just disable TWIM IRQ for a while
		__asm__ volatile ("dmb" ::: "memory");
		play_pending = true;
		__asm__ volatile ("dmb" ::: "memory");
		if (twi_done && play_pending)
			twim_handler(NULL, NULL);
	} else {
		rxtx_buf[0] = 0x04;	// WAV_FRM_SEQ[]
		fill_play_buf(rxtx_buf, seq, len);
		rxtx_buf[9] = 0x01;	// GO

		twi_done = false;
		err_code = nrfx_twim_xfer(&m_twim, &wrten_desc, 0);
		APP_ERROR_CHECK(err_code);
	}
}

// NFP-ELV1030AL-100
// rated-voltage: 2.0 Vrms AC (sine)
// test: 100g fixture
// (overdrive: 25%)a

static const uint8_t sample_time = 3;	// (def) = 300us
static uint8_t cal_settings[] = {
	0x16,	/* starting reg: RATED_VOLTAGE */
	0x50,	/* RATED_VOLTAGE; 2Vrms, f=205 */
	0x9F,	/* OD_CLAMP: 3.49Vpk (2.47Vrms) */
	0x0D,	/* A_CAL_COMP: defaults, ignored */
	0x6D,	/* A_CAL_BEMF: defaults, ignored */
	0xB6,	/* FBC: LRA mode, FB_BRAKE_FACTOR=(def) 4x, LOOP_GAIN=(def) med, BEMF_GAIN=(def,ign) 20x */
	0x93,	/* CTL1: STARTUP_BOOST+, AC_COUPLE-, DRIVE_TIME=19 = 2.4ms (0.5/fRated_LRA = 205Hz) */
	0x45 | (sample_time << 4),	/* CTL2: BIDIR_INPUT-, BRAKE_STAB+, SAMPLE_TIME, BLANKING_TIME=(def) 1, IDISS_TIME=(def) 1 */
	0x94,	/* CTL3: def, LRA_DRIVE_MODE=2/T */
	0x10,	/* CTL4: AUTO_CAL_TIME=1 (250-450 ms) */
};

static void motor_auto_calibrate(void)
{
	uint8_t v;

	motor_reg_writeb(0x01, 0x07);	// MODE = auto-calibration

	motor_reg_writeseq(cal_settings, ARRAY_SIZE(cal_settings) - 1);

	motor_reg_writeb(0x0C, 0x01);	// GO=1
	while (motor_reg_readb(0x0C))
		nrfx_coredep_delay_us(1000);

	v = motor_reg_readb(0x00);	// DIAG_RESULT?
	NRF_LOG_INFO("auto-calibration result: %x", v & 0x1f);
}

static const char *const drvic[8] = {
	"unknown",
	"unknown",
	"unknown",
	"DRV2605",
	"DRV2604",
	"unknown",
	"DRV2604L",
	"DRV2605L",
};

static unsigned bcd_digit(uint32_t bcd, int digit)
{
	return (bcd >> (4 * digit)) & 0x0F;
}

static unsigned from_bcd(uint32_t bcd, int digits)
{
	int i;
	unsigned v = 0, dv = 1;

	for (i = 0; i < digits; ++i) {
		v += bcd_digit(bcd, i) * dv;
		dv *= 10;
	}

	return v;
}

__attribute__((const))
static float clampf(float v, float vmin, float vmax)
{
	if (v > vmax)
		return vmax;
	if (v < vmin)
		return vmin;
	return v;
}

// rated-voltage: rms
// overdriver: peak
// freq: ~resonance
// uint32-BCD: rv[1], od[1], freq[2]
// rv, od: 1.1; freq: 4.0

// default settings: rv=2V, od=3.5V, freq=205Hz
uint32_t motor_conf = __builtin_bswap32(0x20350205);

static const float freq_coeff = (2 * sample_time + 9) * 1.0e-4f;

__attribute__((unused))
static void debug_calib_settings(float freq)
{
	unsigned rvd, odd, freqd;
	float rv, od, dt;

	rv = cal_settings[1];
	od = cal_settings[2];
	dt = cal_settings[6] & 0x1f;

	rv *= 20.71e-3f * 100.0f;
	rv /= sqrtf(1.0f - freq_coeff * freq);

	od *= 21.96e-3f * 100.0f;

	dt = 5000.0f / (dt + 5.0f);

	rvd = rv;
	odd = od;
	freqd = dt;

	NRF_LOG_INFO("calibrating with rated-voltage= %u.%02u Vrms, overdrive-clamp= %u.%02u V, freq-guess= %u Hz",
		rvd / 100, rvd % 100, odd / 100, odd % 100, freqd);
	NRF_LOG_INFO("register settings: rv %u od %u dt %u",
		cal_settings[1], cal_settings[2], cal_settings[6] & 0x1f);
}

__attribute__((unused))
static void debugf(float fp)
{
	unsigned i, f;

	i = floorf(fp);
	f = (fp - floorf(fp)) * 1e9f;

	NRF_LOG_INFO("float: %u.%09u", i, f);
}

static void motor_setup(uint32_t conf)
{
	conf = __builtin_bswap32(conf);

	// freq_coeff = 4 * SAMPLE_TIME + 300us; SAMPLE_TIME = 150us + 50us * sample_time
	float rv = from_bcd(conf >> 24, 2) * 1e-1f;
	float od = from_bcd(conf >> 16, 2) * 1e-1f;
	float freq = from_bcd(conf, 4);
	float dt;

	rv *= sqrtf(1.0f - freq_coeff * freq);
	rv = clampf(roundf(rv * (1.0f/20.71e-3f)), 1, 255);
	cal_settings[1] = (uint8_t)rv;

	od = clampf(roundf(od * (1.0f/21.96e-3f)), 1, 255);
	cal_settings[2] = (uint8_t)od;

	// drive-time = 1/2 * 1/f; DRIVE_TIME = (drive-time - 0.5 ms) / 0.1 ms
	dt = clampf(5000.0f / freq - 5.0f, 0, 31);
	cal_settings[6] &= ~0x1F;
	cal_settings[6] |= (uint8_t)dt;

	debug_calib_settings(freq);
}

static void do_motor_config(void *evdata, uint16_t evsize)
{
	nrfx_twim_enable(&m_twim);
	nrf_gpio_pin_set(MOTOR_EN);

	motor_setup(motor_conf);
	motor_auto_calibrate();
	twim_handler(NULL, NULL);

	if (twi_done) {
		nrf_gpio_pin_clear(MOTOR_EN);
		nrfx_twim_disable(&m_twim);
	}
}

#endif /* MOTOR_I2C_ADDR */

__attribute__((cold))
bool motor_reconfigure(uint32_t conf)
{
#if MOTOR_I2C_ADDR && !ACCEL_TEST
	motor_conf = conf;
	NRF_LOG_INFO("HAPTICS config 0x%08x", __builtin_bswap32(motor_conf));
	app_sched_event_put(NULL, 0, &do_motor_config);
#endif
	return true;
}

__attribute__((cold))
void motor_init(void)
{
#if ACCEL_TEST || !MOTOR_I2C_ADDR
#  if MOTOR_EN
	nrf_gpio_cfg_output(MOTOR_EN);
	nrf_gpio_pin_clear(MOTOR_EN);
#  endif
#elif MOTOR_I2C_ADDR
	ret_code_t err_code;
	uint8_t v, v2;

	nrf_gpio_pin_clear(MOTOR_TRIG);
	nrf_gpio_pin_clear(MOTOR_EN);

	nrf_gpio_cfg_output(MOTOR_TRIG);
	nrf_gpio_cfg_output(MOTOR_EN);

	err_code = nrfx_twim_init(&m_twim, &twim_config, twim_handler, NULL);
	APP_ERROR_CHECK(err_code);

	// DRV2605 init sequence

	nrf_gpio_pin_set(MOTOR_EN);
	NRFX_DELAY_US(1500);

	nrfx_twim_enable(&m_twim);

	v = motor_reg_readb(0x00);	// 00[7:5] = DEVICE_ID
	v2 = motor_reg_readb(0x03);	// 03[2:0] = LIBRARY_SEL
	NRF_LOG_INFO("motor driver ID %u (%s), library %u", v >> 5, drvic[v >> 5], v2 & 0x07);

	if (v != 0) {
		motor_setup(motor_conf);
		motor_auto_calibrate();
		twi_done = true;

		motor_reg_writeb(0x01, 0x00);	// MODE = internal trigger, out of STANDBY
	}

	nrf_gpio_pin_clear(MOTOR_EN);
	nrfx_twim_disable(&m_twim);
#endif /* MOTOR_I2C_ADDR */
}
