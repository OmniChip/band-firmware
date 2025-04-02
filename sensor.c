/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 *
 * Author: Michał Mirosław
 */

#include <stdint.h>
#include "app_error.h"
#include "nrf_log.h"
#include "nrfx_gpiote.h"
#include "nrfx_ppi.h"
#include "nrfx_spim.h"
#include "nrfx_swi.h"
#include "hal/nrf_gpio.h"
#include "bsp.h"
#include "sensor.h"
#include "timebase.h"

#if SENSOR_LSM9DS1
#define SENSOR_ODR_BASE 15
#else
#define SENSOR_ODR_BASE 13
#endif

#if !ACCEL_TEST
#define SENSOR_ODR_HZ (4*SENSOR_ODR_BASE)
#define NOTIFY_BURST 0
#else
#define SENSOR_ODR_HZ (8*SENSOR_ODR_BASE)
#define NOTIFY_BURST 20
#endif

#ifndef SPIM1_FREQ
#define SPIM1_FREQ NRF_SPIM_FREQ_8M
#endif

#if ACCEL_TEST
#undef GYRO_ORIENT
#undef ACCL_ORIENT
#endif

#ifndef GYRO_ORIENT
#define GYRO_ORIENT 0	// def: +X, +Y, +Z
#endif

#ifndef ACCL_ORIENT
#define ACCL_ORIENT 0
#endif

#ifndef MAG_POWER
#define MAG_POWER 0
#endif

#define MAG_ODR_HZ 10

#define REG_WVERIFY 0
#define DEBUG_SENSOR_VALS 0
#define LOG_DECIMATE 512

volatile uint32_t ts;

int16_t sensor_magzero[3];
uint8_t sensor_magcorr[9];

static const nrfx_spim_t m_spim = NRFX_SPIM_INSTANCE(1);
static const nrfx_spim_config_t spim_config = {
	.sck_pin = SPIM1_SCK_PIN,
	.mosi_pin = SPIM1_MOSI_PIN,
	.miso_pin = SPIM1_MISO_PIN,
	.ss_pin = NRFX_SPIM_PIN_NOT_USED, //SPIM1_SS_PIN,
	.ss_active_high = false,
	.irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
	.orc = 0xFF,
	.frequency = SPIM1_FREQ,
	.mode = NRF_SPIM_MODE_0,
	.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST,
};

static volatile uint32_t *spi_start_task, *ss_set_active_task;
static volatile uint32_t *ssm_set_active_task;

static void do_spim_xfer(const nrfx_spim_xfer_desc_t *dma, volatile uint32_t *const ss_task)
{
	nrfx_err_t err_code;
	volatile uint32_t *event;
	bool ok;

	NRF_LOG_DEBUG("SPI xfer [%c] W %u @%p  R %u @%p  (reg 0x%02x)",
		ss_task == ss_set_active_task ? 'A' : 'M',
		dma->tx_length, dma->p_tx_buffer, dma->rx_length, dma->p_rx_buffer,
		*(uint8_t *)dma->p_tx_buffer);

	*ss_task = 1;
	err_code = nrfx_spim_xfer(&m_spim, dma, NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER);
	APP_ERROR_CHECK(err_code);

	event = (void *)nrfx_spim_end_event_get(&m_spim);
	NRFX_WAIT_FOR(*event, 16 * (NRF_SPIM_FREQ_8M / spim_config.frequency), 1, ok);
	if (!ok)
		APP_ERROR_CHECK(NRFX_ERROR_TIMEOUT);
//        nrf_gpio_pin_set(spim_config.ss_pin);
}

struct sensor_spi_buf {
	uint8_t pad0;
	uint8_t c;
	uint16_t d;		// little-endian!
	uint16_t e;		// little-endian!
	uint16_t f;		// little-endian!
};

struct sensor_big_spi_buf {
	union {
		struct sensor_spi_buf reg;
		struct {
			uint8_t c;
			uint8_t si[9];
			uint16_t hi[3];
		} mag;
	};
};

static struct sensor_spi_buf sensor_reg_rbuf;
static struct sensor_big_spi_buf sensor_reg_wbuf;

#define DEFINE_sensor_reg_DMA(name, is_read, vtype)		\
static const nrfx_spim_xfer_desc_t spim_dma_reg_##name = NRFX_SPIM_XFER_TRX( \
	&sensor_reg_wbuf.reg.c, (is_read ? 1 : 1 + sizeof(vtype)),	\
	&sensor_reg_rbuf.c, (is_read ? 1 + sizeof(vtype) : 0))

DEFINE_sensor_reg_DMA(read2w, 1, uint32_t);
static const nrfx_spim_xfer_desc_t spim_dma_write_mag =
	NRFX_SPIM_XFER_TX(&sensor_reg_wbuf.mag, sizeof(sensor_reg_wbuf.mag));
static const nrfx_spim_xfer_desc_t spim_dma_write_mag_hi =
	NRFX_SPIM_XFER_TX(&sensor_reg_wbuf.mag.si[8], 1 + sizeof(sensor_reg_wbuf.mag.hi));

#define DEFINE_sensor_reg_read(name, vtype)		\
DEFINE_sensor_reg_DMA(name, 1, vtype);			\
static vtype sensor_reg_##name##_(uint8_t reg)		\
{							\
	sensor_reg_wbuf.reg.c = 0x80 | reg;		\
							\
	do_spim_xfer(&spim_dma_reg_##name, ss_set_active_task);	\
							\
	return (vtype)sensor_reg_rbuf.d;		\
}							\
__attribute__((unused))					\
static vtype sensor_reg_##name(uint8_t reg)		\
{							\
	vtype ret = sensor_reg_##name##_(reg);		\
	NRF_LOG_DEBUG("reg 0x%x: read 0x%x", reg, ret); \
	return ret;					\
}

#define sensor_reg_verify_writeb sensor_reg_readb_
#define sensor_reg_verify_writew sensor_reg_readw_

#define DEFINE_sensor_reg_write(name, vtype)		\
DEFINE_sensor_reg_DMA(name, 0, vtype);			\
__attribute__((unused))					\
static void sensor_reg_##name(uint8_t reg, vtype val)	\
{							\
	vtype vrfy;					\
							\
	sensor_reg_wbuf.reg.c = reg;			\
	sensor_reg_wbuf.reg.d = val;			\
							\
	do_spim_xfer(&spim_dma_reg_##name, ss_set_active_task);	\
							\
	if (REG_WVERIFY) {				\
		vrfy = sensor_reg_verify_##name(reg);	\
		NRF_LOG_DEBUG("reg 0x%x: wrote 0x%x readback 0x%x", reg, val, vrfy); \
	} else {					\
		NRF_LOG_DEBUG("reg 0x%x: wrote 0x%x", reg, val); \
	}						\
}

DEFINE_sensor_reg_read(readb, uint8_t)
DEFINE_sensor_reg_read(readw, uint16_t)
DEFINE_sensor_reg_write(writeb, uint8_t)
DEFINE_sensor_reg_write(writew, uint16_t)


#define LSM_FIFO_WORDS_MASK 0xfff
#define LSM_FIFO_FTH (1 << 15)
#define LSM_FIFO_OVERRUN (1 << 14)
#define LSM_FIFO_FULL (1 << 13)
#define LSM_FIFO_EMPTY (1 << 12)

#define FIFO_PATTERN_LENGTH 6	// gyro.xyz, acc.xyz

static bool iron_corr_enabled = false;
static uint16_t n_reads;
static uint16_t n_skip;
static volatile bool fifo_reading;
static volatile uint32_t last_irq_timestamp, prev_irq_timestamp;
static volatile uint32_t cur_data_timestamp;
static volatile uint32_t last_mag_timestamp;

static struct band_input_rec dbuf[1024];
static volatile uint16_t head, tail;

static volatile uint32_t mag_ts, mag_data_ts;
static volatile uint32_t next_mag_trigger;
static volatile uint16_t mag_data[3];
static volatile bool mag_read_pending;
static volatile bool mag_read_in_progress;

static uint16_t dbuf_next_i(uint16_t i)
{
	return (i + 1) % ARRAY_SIZE(dbuf);
}

static uint16_t dbuf_distance(uint16_t i, uint16_t j)
{
	return (j - i) % ARRAY_SIZE(dbuf);
}

static void sensor_handle_int(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void sensor_read_done(void);

__attribute__((weak,cold)) void hids_init(void) {}
__attribute__((weak)) void hids_connected(void) {}
__attribute__((weak)) void hids_disconnected(void) {}
__attribute__((weak)) void hids_tx_complete(uint8_t count) {}
__attribute__((weak)) void hids_trigger_send(void) {}
__attribute__((weak)) void hids_handle_key(bsp_event_t event) {}

static void trigger_fifo_read(void)
{
	int16_t *rxd = SENSOR_LSM9DS1 ? &dbuf[tail].ax : &dbuf[tail].gx;
	uint32_t last_irq_ts = last_irq_timestamp;

	++ts;

	cur_data_timestamp = (SENSOR_LSM9DS1 ? last_irq_ts : prev_irq_timestamp);

	if (!SENSOR_LSM9DS1)
		prev_irq_timestamp = last_irq_ts;
	__sync_bool_compare_and_swap(&last_irq_timestamp, last_irq_ts, last_irq_ts + 1000000 / SENSOR_ODR_HZ);

	if (SENSOR_LSM9DS1)
		sensor_reg_wbuf.reg.c = 0x80|0x28;
	m_spim.p_reg->RXD.PTR = (uintptr_t)((uint8_t *)rxd - 1);
	*ss_set_active_task = 1;
	*spi_start_task = 1;
}

static void trigger_fifo_read_gyro(void)
{
	// SENSOR_LSM9DS1
	int16_t *rxd = &dbuf[tail].gx;

	m_spim.p_reg->RXD.PTR = (uintptr_t)((uint8_t *)rxd - 1);
	sensor_reg_wbuf.reg.c = 0x80|0x18;

	*ss_set_active_task = 1;
	*spi_start_task = 1;
}

static void continue_mag_data_read(void)
{
	sensor_reg_wbuf.reg.c = 0x80 | MAG_DATA_REG;
	m_spim.p_reg->RXD.MAXCNT = 7;
	*ssm_set_active_task = 1;
	*spi_start_task = 1;
}

static void handle_mag_data_read(void)
{
	int16_t *data = (void *)&sensor_reg_rbuf.d;
	size_t i;

	if ((mag_ts % LOG_DECIMATE) == 0)
	NRF_LOG_INFO("mag-ts %u data %d %d %d", mag_ts, data[0], data[1], data[2]);
	++mag_ts;

	mag_data_ts = timebase_adjusted_us(last_mag_timestamp);
	for (i = 0; i < 3; ++i)
		mag_data[i] = data[i];

	last_mag_timestamp += 1000000 / MAG_ODR_HZ;
	bands_notify_magdata();

	sensor_read_done();
}

void sensor_read_mag_data(struct magdata_rec *data)
{
	size_t i;
	uint32_t mu;

	do {
		mu = mag_ts;
		for (i = 0; i < 3; ++i)
			(&data->mx)[i] = mag_data[i];
		data->timestamp = mag_data_ts;
	} while (mu != mag_ts);

	// TODO: scale to [0.001 Gauss]
}

volatile bool m_do_input_notify = false;

void reset_sensor_fifo(void)
{
	// XXX zaślepka
	head = tail = 0;
}

static bool sensor_read_start(void)
{
	return !__sync_fetch_and_or((volatile uint8_t *)&fifo_reading, 1);
}

static void sensor_read_done(void)
{
	__sync_fetch_and_and((volatile uint8_t *)&fifo_reading, 0);
//	nrfx_gpiote_in_event_enable(SENSOR_INT1, true);
	if (nrf_gpio_pin_read(SENSOR_INT1)) {
//		sensor_handle_int(SENSOR_INT1, NRF_GPIOTE_POLARITY_LOTOHI);
		sensor_trigger_read_fifo();
	} else if (mag_read_pending) {
		sensor_trigger_read_mag();
	}
}

void sensor_trigger_read_fifo(void)
{
	nrfx_err_t err_code;
	const nrfx_spim_xfer_desc_t *dma;

	if (!sensor_read_start())
		return;

//	NRF_LOG_INFO("XL_OUT: %d, %d, %d", (int16_t)sensor_reg_readw(0x28), (int16_t)sensor_reg_readw(0x2A), (int16_t)sensor_reg_readw(0x2C));

	*ss_set_active_task = 1;

	if (SENSOR_LSM9DS1) {
		sensor_reg_wbuf.reg.c = 0x80 | 0x2F;
		dma = &spim_dma_reg_readb;
	} else {
		sensor_reg_wbuf.reg.c = 0x80 | 0x3A;
		dma = &spim_dma_reg_read2w;
	} 

	err_code = nrfx_spim_xfer(&m_spim, dma, 0);
	APP_ERROR_CHECK(err_code);
}

void sensor_trigger_read_mag(void)
{
	nrfx_err_t err_code;

	if (!MAG_DATA_REG)
		return;

	if (!sensor_read_start()) {
		mag_read_pending = true;
		return;
	}

	mag_read_pending = false;

//	if (mag_ts % (LOG_DECIMATE/8) == 0)
//	NRF_LOG_INFO("MAG_OUT: %d, %d, %d", (int16_t)sensor_reg_readw(0x2E), (int16_t)sensor_reg_readw(0x30), (int16_t)sensor_reg_readw(0x32));

	if (MAG_ON_SPI) {
		mag_read_in_progress = true;
		err_code = nrfx_spim_xfer(&m_spim, &spim_dma_reg_readw, NRFX_SPIM_FLAG_HOLD_XFER);
		APP_ERROR_CHECK(err_code);
		continue_mag_data_read();
	} else {
		sensor_reg_wbuf.reg.c = 0x80 | 0x1E;
		*ssm_set_active_task = 1;
		err_code = nrfx_spim_xfer(&m_spim, &spim_dma_reg_readw, 0);
		APP_ERROR_CHECK(err_code);
		next_mag_trigger = timebase_captured_ts() + 1000000 / MAG_ODR_HZ;
	}
}

static void sensor_handle_int(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint32_t us;

//	nrfx_gpiote_in_event_enable(SENSOR_INT1, false);
	us = timebase_captured_ts();
	last_irq_timestamp = us;

	if (MAG_DATA_REG && !MAG_ON_SPI && (int32_t)(us - next_mag_trigger) >= 0) {
		mag_read_pending = true;
		next_mag_trigger = us + 1000000 / MAG_ODR_HZ;
	}

	sensor_trigger_read_fifo();
}

static void mag_handle_int(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	last_mag_timestamp = timebase2_captured_ts();

	NRF_LOG_DEBUG("mag-rdy trigger");
	sensor_trigger_read_mag();
}

static uint16_t process_read_start(void)
{
	uint16_t count, val;

	val = sensor_reg_rbuf.d;
	count = val & LSM_FIFO_WORDS_MASK;

	if (sensor_reg_rbuf.e != 0) {
		uint16_t n_skip = sensor_reg_rbuf.e;
		n_skip = n_skip < FIFO_PATTERN_LENGTH ? FIFO_PATTERN_LENGTH - n_skip : 1;

		if (ts % LOG_DECIMATE == 0 || ts < 3)
		NRF_LOG_INFO("ts %u: FIFO.PATTERN = 0x%04x, %u samples to skip, %u samples in FIFO",
			ts, sensor_reg_rbuf.e, n_skip, count);

		if (n_skip > count)
			n_skip = count;
		if ((count - n_skip) / FIFO_PATTERN_LENGTH < 1)
			return 0;

		// FIXME: just reset FIFO
		if (n_skip > 3)
			n_skip = 3;
		m_spim.p_reg->RXD.MAXCNT = 1 + n_skip * 2;

		return 1;
	}

	count /= FIFO_PATTERN_LENGTH;
	if (count)
		--count;

	if (ts % LOG_DECIMATE == 0 || ts < 3) { // || (val & LSM_FIFO_WORDS_MASK) % FIFO_PATTERN_LENGTH) {
	NRF_LOG_INFO("%u*R/%u fifo words to read (status 0x%04x%s%s%s)",
		count, val & LSM_FIFO_WORDS_MASK, val,
		(val & LSM_FIFO_OVERRUN) ? " OVERRUN" : "",
		(val & LSM_FIFO_FULL) ? " FULL" : "",
		(val & LSM_FIFO_EMPTY) ? " EMPTY" : "");
	}

	if (count) {
		sensor_reg_wbuf.reg.c = 0x80 | 0x3E;
		m_spim.p_reg->RXD.MAXCNT = 1 + FIFO_PATTERN_LENGTH * 2;
	}

	return count;
}

static bool handle_mag_spi_xfr(void)
{
	if (mag_read_in_progress) {
		mag_read_in_progress = false;

		handle_mag_data_read();
		return true;
	}

	return false;
}

static bool handle_mag_i2c_xfr(void)
{
	if (sensor_reg_wbuf.reg.c == (0x80|0x1E)) {
		if (mag_ts % (LOG_DECIMATE/8) == 0)
		NRF_LOG_INFO("mag-ts %u status 0x%04x iron-corr %u", mag_ts, sensor_reg_rbuf.d, iron_corr_enabled);

		bool mag_avail = (sensor_reg_rbuf.d >> 8 >> iron_corr_enabled) & 1;
		if (mag_avail) {
			continue_mag_data_read();
		} else {
			next_mag_trigger = last_irq_timestamp;
			sensor_read_done();
		}

		return true;
	}

	if (sensor_reg_wbuf.reg.c == (0x80|MAG_DATA_REG)) {
		handle_mag_data_read();
		return true;
	}

	return false;
}

static void sensor_data_complete(void)
{
	uint16_t i = tail, nt = dbuf_next_i(tail);

	dbuf[i].timestamp = timebase_adjusted_us(cur_data_timestamp);
	if (ACCL_ORIENT & 0x20)
		dbuf[i].ax = -dbuf[i].ax;
	if (ACCL_ORIENT & 0x10)
		dbuf[i].ay = -dbuf[i].ay;
	if (ACCL_ORIENT & 0x08)
		dbuf[i].az = -dbuf[i].az;

	if (m_do_input_notify) {
		/* block new if lagging to notify */
		if (nt != head)
			tail = nt;
	} else {
		/* drop old if nobody's watching */
		if (nt == head)
			head = dbuf_next_i(head);
		tail = nt;
	}

	--n_reads;

	if (ts % LOG_DECIMATE == 0)
	NRF_LOG_INFO("ts %u @%u, reads left %u, h/t %u %u, r[%u]=...", ts, dbuf[i].timestamp, n_reads, head, tail, i);
	if (DEBUG_SENSOR_VALS && ts % LOG_DECIMATE == 0)
	NRF_LOG_INFO("... R(%d,%d,%d) A(%d,%d,%d)",
		(int)dbuf[i].gx, (int)dbuf[i].gy, (int)dbuf[i].gz,
		(int)dbuf[i].ax, (int)dbuf[i].ay, (int)dbuf[i].az);

	hids_trigger_send();
	bands_trigger_send();
}

static void handle_lsm6_xfer(void)
{
	if (sensor_reg_wbuf.reg.c != (0x80|0x3E))
		n_reads = process_read_start();
	else
		sensor_data_complete();

	if (n_reads)
		trigger_fifo_read();
	else
		sensor_read_done();
}

static uint16_t process_read_start_lsm9(void)
{
	uint8_t val = sensor_reg_rbuf.d;

	if (ts % LOG_DECIMATE == 0 || ts < 3 || (val & 1))
	NRF_LOG_INFO("%u fifo sets to read (status 0x%02x%s%s)",
		val & 0x3F, val,
		(val & 0x40) ? " OVERRUN" : "",
		(val & 0x80) ? " FTH" : "");

	val &= 0x3F;
	val /= 2;

	if (val)
		m_spim.p_reg->RXD.MAXCNT = 7;

	return val;
}

static void handle_lsm9_xfer(void)
{
	if (sensor_reg_wbuf.reg.c == (0x80|0x28)) {
		trigger_fifo_read_gyro();
		return;
	}

	if (sensor_reg_wbuf.reg.c == (0x80|0x2F))
		n_reads = process_read_start_lsm9();
	else
		sensor_data_complete();

	if (n_reads)
		trigger_fifo_read();
	else
		sensor_read_done();
}

static void spim_handler(nrfx_spim_evt_t const *p_event, void *p_context)
{
	if (p_event->type != NRFX_SPIM_EVENT_DONE)
		return;

	if (MAG_DATA_REG)
		if (MAG_ON_SPI ? handle_mag_spi_xfr() : handle_mag_i2c_xfr())
			return;

	if (SENSOR_LSM9DS1)
		handle_lsm9_xfer();
	else
		handle_lsm6_xfer();
}

static bool in_burst = false;

struct band_input_rec *sensor_next_report(void)
{
	uint16_t i = head, j = tail;

	if (i == j) {
		if (NOTIFY_BURST)
			in_burst = false;
		return NULL;
	}

	if (NOTIFY_BURST && !in_burst) {
		if (dbuf_distance(i, j) < NOTIFY_BURST)
			return NULL;
		in_burst = true;
	}

	head = dbuf_next_i(i);

//	NRF_LOG_INFO("-+- R(%d,%d,%d) A(%d,%d,%d)",
//		(int)dbuf[i].gx, (int)dbuf[i].gy, (int)dbuf[i].gz,
//		(int)dbuf[i].ax, (int)dbuf[i].ay, (int)dbuf[i].az);
	return &dbuf[i];
}

static void spim_ss_setup(uint32_t pin, uint32_t *act_task, uint32_t *deact_task)
{
	static const nrfx_gpiote_out_config_t ss_pin_config = NRFX_GPIOTE_CONFIG_OUT_TASK_LOW;
	ret_code_t err_code;

	err_code = nrfx_gpiote_out_init(pin, &ss_pin_config);
	APP_ERROR_CHECK(err_code);

	nrfx_gpiote_out_task_enable(pin);

	*deact_task = nrfx_gpiote_set_task_addr_get(pin);
	*act_task = nrfx_gpiote_out_task_addr_get(pin);
}

static void sensor_irq_setup(uint32_t pin, nrfx_gpiote_evt_handler_t irq_handler, uint32_t capture_task)
{
	static const nrfx_gpiote_in_config_t int_pin_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	nrf_ppi_channel_t m_irq_timestamp_ppich;
	ret_code_t err_code;
	uint32_t irq_event;

	nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_NOPULL);

	err_code = nrfx_gpiote_in_init(pin, &int_pin_config, irq_handler);
	APP_ERROR_CHECK(err_code);

	if (!capture_task)
		return;

	irq_event = nrfx_gpiote_in_event_addr_get(pin);
	err_code = nrfx_ppi_channel_alloc(&m_irq_timestamp_ppich);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_channel_assign(m_irq_timestamp_ppich, irq_event, capture_task);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_channel_enable(m_irq_timestamp_ppich);
	APP_ERROR_CHECK(err_code);
}

static void spim_pins_init(void)
{
	nrf_ppi_channel_t m_spi_stop_ppich;
	ret_code_t err_code;
	uint32_t spi_end_event, ss_deact_task;

	spi_start_task = (void *)nrfx_spim_start_task_get(&m_spim);
	spi_end_event = nrfx_spim_end_event_get(&m_spim);

	nrfx_gpiote_init();

	spim_ss_setup(SPIM1_SS_PIN, (void *)&ss_set_active_task, &ss_deact_task);
	err_code = nrfx_ppi_channel_alloc(&m_spi_stop_ppich);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_channel_assign(m_spi_stop_ppich, spi_end_event, ss_deact_task);
	APP_ERROR_CHECK(err_code);
	if (MAG_ON_SPI) {
		spim_ss_setup(SPIM1_SSM_PIN, (void *)&ssm_set_active_task, &ss_deact_task);
		err_code = nrfx_ppi_channel_fork_assign(m_spi_stop_ppich, ss_deact_task);
		APP_ERROR_CHECK(err_code);
	} else if (MAG_DATA_REG) {
		ssm_set_active_task = ss_set_active_task;
	}
	err_code = nrfx_ppi_channel_enable(m_spi_stop_ppich);
	APP_ERROR_CHECK(err_code);

	sensor_irq_setup(SENSOR_INT1, sensor_handle_int, timebase_capture_task());
	if (MAG_DATA_REG)
		sensor_irq_setup(SENSOR_INT2, mag_handle_int, timebase2_capture_task());
}

static void slave_reg_writeb(uint8_t reg, uint8_t val)
{
	if (MAG_ON_SPI) {
		sensor_reg_wbuf.reg.c = reg;
		sensor_reg_wbuf.reg.d = val;

		do_spim_xfer(&spim_dma_reg_writeb, ssm_set_active_task);

		if (!REG_WVERIFY) {
			NRF_LOG_DEBUG("S: reg 0x%x: wrote 0x%x", reg, val);
		} else {
			sensor_reg_wbuf.reg.c = reg | 0x80;
			do_spim_xfer(&spim_dma_reg_readb, ssm_set_active_task);

			NRF_LOG_INFO("S: reg 0x%x: wrote 0x%x readback 0x%x",
				reg, val, sensor_reg_rbuf.d & 0xFF);
		}
	} else if (MAG_DATA_REG) {
		uint16_t slv_addr = (reg << 8) | (MAG_I2C_ADDR << 1);
		bool ok;

		sensor_reg_writeb(0x01, 0x80);		// FUNC_CFG_ACCESS: 0x02..0x32 are FUNC_CFG regs now
		sensor_reg_writew(0x02, slv_addr);	// SLV0_SUBADD: reg; SLV0_ADD: magnetometer, write
		sensor_reg_writeb(0x0E, val);		// DATAWRITE_SRC_MODE_SUB_SLV0: value
		sensor_reg_writeb(0x01, 0x00);		// FUNC_CFG_ACCESS: 0x02..0x32 are normal regs now

		sensor_reg_writew(0x19, 0x093C);	// NASTER_CONFIG: START=XL-DR, PU_EN=1, MASTER_ON=1; CTRL10_C: gyro=XYZ, FUNC_EN=1
		sensor_reg_writeb(0x10, 0x80);		// CTRL1_XL: ODR=1.66kHz [trigger master]
		NRFX_WAIT_FOR(sensor_reg_readb(0x53) & 1, 100, 100, ok);
		if (!ok)
			NRF_LOG_INFO("sensor slave write timeout");
		sensor_reg_writew(0x19, 0x0838);	// MASTER_CONF: +MASTER_ON=0; CTRL10_C: +FUNC_EN=0
		sensor_reg_writeb(0x10, 0x00);		// CTRL1_XL: ODR=power-down
	} else {
		APP_ERROR_CHECK(NRFX_ERROR_NOT_SUPPORTED);
	}
}

static uint8_t slave_reg_readb(uint8_t reg)
{
	if (MAG_ON_SPI) {
		sensor_reg_wbuf.reg.c = reg | 0x80;
		do_spim_xfer(&spim_dma_reg_readb, ssm_set_active_task);
		return sensor_reg_rbuf.d;
	} else {
		APP_ERROR_CHECK(NRFX_ERROR_NOT_SUPPORTED);
	}
}

static void setup_mag_ironc(int16_t *hard, uint8_t *soft)
{
	const nrfx_spim_xfer_desc_t *dma;
	size_t i;

	if (!MAG_I2C_ADDR)
		return;

	if (!hard) {
		iron_corr_enabled = false;
		return;
	}

	sensor_reg_writeb(0x01, 0x80);		// FUNC_CFG_ACCESS: 0x02..0x32 are FUNC_CFG regs now

	for (i = 0; i < 3; ++i)
		sensor_reg_wbuf.mag.hi[i] = hard[i];

	if (soft) {
		for (i = 0; i < 9; ++i)
			sensor_reg_wbuf.mag.si[i] = soft[i];
		sensor_reg_wbuf.mag.c = 0x24;
		dma = &spim_dma_write_mag;
	} else {
		sensor_reg_wbuf.mag.si[8] = 0x2D;
		dma = &spim_dma_write_mag_hi;
	}

	do_spim_xfer(dma, ss_set_active_task);
	sensor_reg_writeb(0x01, 0x00);		// FUNC_CFG_ACCESS: 0x02..0x32 are normal regs now

	iron_corr_enabled = true;
}

static void sensor_setup_mag(void)
{
	uint8_t v;

	if (MAG_POWER) {
		nrf_gpio_cfg_output(MAG_POWER);
		nrf_gpio_pin_set(MAG_POWER);
		nrfx_coredep_delay_us(200000);
	}

	// whoami@0x0F = 0x3D
	if (MAG_ON_SPI) {
		v = slave_reg_readb(0x0F);
		if (v != 0x3D) {
			NRF_LOG_ERROR("mag read ID @0x%02x: 0x%02x",
				sensor_reg_wbuf.reg.c & 0x7f, v);
			APP_ERROR_CHECK(NRFX_ERROR_NOT_SUPPORTED);
		}

		slave_reg_writeb(0x21, 0x04);
		NRFX_WAIT_FOR(~slave_reg_readb(0x21) & 4, 100, 50, v);
		if (!v) {
			NRF_LOG_INFO("mag reset timeout");
			APP_ERROR_CHECK(NRFX_ERROR_TIMEOUT);
		}
	}

	NRF_LOG_DEBUG("configuring magnetometer");

	// hard-iron offset: XYZ @ 0x05..0x0A

	// CTRL1@0x20 = 0x10: ODR=10Hz, TEMP_EN=0, XY_MODE=low-power
	slave_reg_writeb(0x20, 0xf0);		// CTRL1: ODR=10Hz, TEMP_EN=1, XY_MODE=UHP
	// CTRL2@0x21 = 0: FS=4gs
	slave_reg_writeb(0x21, 0x00);
	// CTRL4@0x23 = 0: Z_MODE=low-power, BLE=little-endian
	slave_reg_writeb(0x23, 0x0C);		// CTRL4: Z_MODE=UHP, BLE=le
	slave_reg_writeb(0x24, 0x40);		// CTRL5: BDU=1
//	slave_reg_writeb(0x31, 0x02);		// INT_SRC: MROI=1
//	slave_reg_writeb(0x30, 0x0F);		// INT_CFG: XYZ=0, IEA=active-high, LIR=no-latch, IEN=1
	v = 0x00; 				// CTRL3: mode=cont-conv, LP=0
//	v = 0x01;				// CTRL3: mode=single-conv, LP=0
	if (0 && SENSOR_LSM9DS1)	// XXX: breaks RDY output
		v |= 0x80;			// CTRL3: +I2C_disable
	slave_reg_writeb(0x22, v);
}

static void slave_setup_poll(uint8_t reg)
{
	if (MAG_DATA_REG && !MAG_ON_SPI) {
		uint16_t slv_addr = 0x8000 | (reg << 8) | (MAG_I2C_ADDR << 1) | 1;

		setup_mag_ironc(sensor_magzero, sensor_magcorr);
		sensor_reg_writeb(0x01, 0x80);		// FUNC_CFG_ACCESS: 0x02..0x32 are FUNC_CFG regs now
		sensor_reg_writew(0x02, slv_addr);	// SLV0_SUBADD: reg; SLV0_ADD: magnetometer, read
		sensor_reg_writeb(0x04, 6);		// SLAVE0_CONFIG: every-1, sensors=1, src_cond_mode=0, slave0_numop=6
		sensor_reg_writeb(0x01, 0x00);		// FUNC_CFG_ACCESS: 0x02..0x32 are normal regs now
		sensor_reg_writeb(0x1A, 0x1B);		// MASTER_CONF: START_CONFIG=ext-INT2, AUX_I2C_PU_EN=1, IRON_EN=1, MASTER_ON=1
	} else {
		APP_ERROR_CHECK(NRFX_ERROR_NOT_SUPPORTED);
	}
}

static unsigned sensor_calc_odr(unsigned hz)
{
	unsigned odr;

	if (SENSOR_LSM9DS1) {
		odr = (hz + 8) / SENSOR_ODR_BASE;
		odr = odr ? 8 * sizeof(unsigned int) - __builtin_clz(odr) : 1;
		if (odr > 2)
			--odr;
		if (odr > 6)
			odr = 6;
	} else {
		odr = hz / SENSOR_ODR_BASE;
		odr = odr ? 8 * sizeof(unsigned int) - __builtin_clz(odr) : 1;
		if (odr > 8)
			odr = 8;
	}

	return odr;
}

static void sensor_set_odr(unsigned odr)
{
	odr = odr ? sensor_calc_odr(odr) : 0;

	if (SENSOR_LSM9DS1) {
		odr <<= 5;
		sensor_reg_writeb(0x10, 0x1A | odr);	// CTRL2_G: INT_SEL=LPF1, OUT_SEL=LPF1; CTRL1_G: ODR=15Hz<<odr[2=>1], FS=2k, BW_LPF2=max
		sensor_reg_writeb(0x20, 0x08 | odr);	// CTRL7_XL: filters off; CTRL6_XL: ODR=gyro, FS=16g, BW_SCAL_ODR=y
	} else {
		odr = (odr << 12) | (odr << 4);
		sensor_reg_writew(0x10, 0x0C04 | odr);	// CTRL2_G: ODR=13/2Hz<<odr, FS=2k; CTRL1_XL: ODR=13/2Hz<<odr, FS=16g
	}
}

static void sensor_setup_lsm6(void)
{
	uint16_t v, axes;

	NRF_LOG_DEBUG("IMU: LSM6DS3+LISM3MDL");

	sensor_reg_writeb(0x12, 0x01);	// reset
	NRFX_WAIT_FOR(~sensor_reg_readb(0x12) & 1, 100, 1, v);
	if (!v) {
		NRF_LOG_INFO("sensor reset timeout");
		APP_ERROR_CHECK(NRFX_ERROR_TIMEOUT);
		return;
	}

	n_skip = 0;
	sensor_reg_writeb(0x0A,   0x00);	// FIFO disable
	sensor_reg_writeb(0x12,   0x44);	// CTRL3_C: BDU,IF_INC

	axes = 0x3838;				// CTRL10_C: GYRO=XYZ; CTRL9_C: XL=XYZ
	if (MAG_DATA_REG) {
		sensor_setup_mag();
		slave_setup_poll(0x28); 	// out@0x28...: X,Y,Z
		axes |= 0x0400;			// CTRL10_C: +FUNC_EN=1
		if (iron_corr_enabled)
			axes |= 0x04;		// CTRL9_XL: +SOFT_EN=1
	}

	sensor_reg_writew(0x13, 0x8024);	// CTRL5_C: reg-rounding=mag, CTRL4_C: I2C_disable, INT2_on_INT1=1
	sensor_reg_writeb(0x58,   0x01);	// TAP_CFG: LIR=1 (FUNC_SRC bits are latched and cleared on read)
#if !SENSOR_DISABLE
	sensor_set_odr(SENSOR_ODR_HZ);
#endif
	sensor_reg_writew(0x16, 0x0004);	// CTRL8_XL: LPF2_XL_EN=0; CTRL7_G: HP_G_EN=0, HPCF=0 (0.0081 Hz), STATUS_ROUNDING=1
	sensor_reg_writew(0x18, axes);		// CTRL10_C: GYRO=XYZ, FUNC_EN=x; CTRL9_C: XL=XYZ
	sensor_reg_writew(0x0D, 0x0008);	// INT2: none; INT1: FTH
	sensor_reg_writew(0x06, 0x0007);	// FIFO.FTH=7
	sensor_reg_writew(0x08, 0x0009);	// FIFO.DS: 1st = GYRO /1, 2nd = XL /1; 3rd = none, 4th = none
	v = sensor_calc_odr(SENSOR_ODR_HZ);
	v = (v << 3) | 0x01;			// FIFO.ODR=13/2Hz<<odr, FIFO.MODE=FIFO (stop when full)
	v |= GYRO_ORIENT << 8;			// ORIENT_CFG
	sensor_reg_writew(0x0A, v);
//	sensor_reg_writeb(0x0A,   0x51);	// FIFO.ODR=max, FIFO.MODE=FIFO (stop when full)
//	sensor_reg_writeb(0x0A,   0x36);	// FIFO.ODR=416Hz, FIFO.MODE=continuous
}

static void sensor_setup_lsm9(void)
{
	bool ok;

	NRF_LOG_DEBUG("IMU: LSM9DS1");

	sensor_reg_writeb(0x22, 0x01);	// reset
	NRFX_WAIT_FOR(~sensor_reg_readb(0x17) & 8, 100, 1, ok);
	if (!ok) {
		NRF_LOG_ERROR("sensor reset timeout");
		APP_ERROR_CHECK(NRFX_ERROR_TIMEOUT);
	}

	n_skip = 0;
	sensor_reg_writeb(0x2E,   0x00);	// FIFO disable
	sensor_reg_writeb(0x22,   0x44);	// CTRL8: BDU,IF_INC
	sensor_reg_writeb(0x23,   0x04);	// CTRL9: I2C_disable
	sensor_reg_writew(0x1E, 0x3838);	// CTRL5_XL: XL=XYZ; CTRL4: GYRO=XYZ, LIR_XL=no-latch

	if (MAG_DATA_REG)
		sensor_setup_mag();
#if !SENSOR_DISABLE
	sensor_set_odr(SENSOR_ODR_HZ);
#endif
	sensor_reg_writew(0x0C, 0x0008);	// INT2: none; INT1: FTH
	sensor_reg_writeb(0x2E, 0x22);		// FIFO.FTH=2, MODE=FIFO (stop when full)
	sensor_reg_writeb(0x23, 0x06);		// CTRL9: I2C_disable, FIFO_EN=1
}

void sensor_enable(void)
{
	//FIXME: sync z INT
	sensor_set_odr(SENSOR_ODR_HZ);
	if (MAG_DATA_REG)
		slave_reg_writeb(0x22, 0x00);
}

void sensor_disable(void)
{
	//FIXME: sync z INT
	sensor_set_odr(0);
	if (MAG_DATA_REG)
		slave_reg_writeb(0x22, 0x03);
}

__attribute__((cold))
void sensor_init(void)
{
	ret_code_t err_code;

	err_code = nrfx_spim_init(&m_spim, &spim_config, spim_handler, NULL);
	APP_ERROR_CHECK(err_code);

	spim_pins_init();

	sensor_reg_writeb(0x01, 0x00);		// FUNC_CFG_ACCESS: 0x02..0x32 are normal regs now
	if (sensor_reg_readb(0x0F) == SENSOR_ID) {
		if (SENSOR_LSM9DS1)
			sensor_setup_lsm9();
		else
			sensor_setup_lsm6();
	} else {
		NRF_LOG_ERROR("sensor read ID @0x%02x: 0x%02x",
			sensor_reg_wbuf.reg.c & 0x7f, sensor_reg_rbuf.d & 0xff);
		APP_ERROR_CHECK(NRFX_ERROR_NOT_SUPPORTED);
	}

	nrfx_gpiote_in_event_enable(SENSOR_INT1, true);
	sensor_trigger_read_fifo();
	if (MAG_ON_SPI) {
		nrfx_gpiote_in_event_enable(SENSOR_INT2, true);
		sensor_trigger_read_mag();
	}
}
