/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 *
 * Author: Michał Mirosław
 */

#include "nrfx_clock.h"
#include "nrfx_timer.h"
#include "nrfx_pwm.h"

#include "nrf_log.h"
#include "nrf_soc.h"

#include "timebase.h"
#include "pstore.h"

#include "boards.h"
#ifdef BOARD_PCA10040
#include "board-band-devb.h"
#endif

#ifndef CLK_OUT_PIN
#define CLK_OUT_PIN NRFX_PWM_PIN_NOT_USED
#endif

#define PSKEY_US_CLK_FREQ (0x00010001)

static const nrfx_timer_config_t timer_config = {
	.frequency = NRF_TIMER_FREQ_1MHz,
	.mode = NRF_TIMER_MODE_TIMER,
	.bit_width = NRF_TIMER_BIT_WIDTH_32,
	.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
};

static const nrfx_pwm_t m_pwm = NRFX_PWM_INSTANCE(0);

static const nrfx_pwm_config_t pwm_config = {
	.output_pins = { CLK_OUT_PIN | NRFX_PWM_PIN_INVERTED, NRFX_PWM_PIN_NOT_USED, NRFX_PWM_PIN_NOT_USED, NRFX_PWM_PIN_NOT_USED },
	.irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
	.base_clock = NRF_PWM_CLK_16MHz,
	.count_mode = NRF_PWM_MODE_UP,
	.top_value = 16,
	.load_mode = NRF_PWM_LOAD_COMMON,
	.step_mode = NRF_PWM_STEP_AUTO,
};

static nrf_pwm_values_common_t pwm_samples[] = { 8, 8 };

static const nrf_pwm_sequence_t pwm_seq = {
	.values.p_common = pwm_samples,
	.length = NRF_PWM_VALUES_LENGTH(pwm_samples),
	.repeats = 0,
	.end_delay = 0,
};

static volatile uint32_t timebase_mul;
static volatile uint32_t timebase_ofs[2];
uint32_t timebase_freq = 1000000;	// 1 MHz
bool timebase_step = true;
static bool step_held;

/** @brief: Function for handling the TIMER1 interrupts.
 */
static void timer_handler(nrf_timer_event_t ev_type, void *ctx)
{
	switch (ev_type) {
	case NRF_TIMER_EVENT_COMPARE2:
		timebase_ofs[1] = timebase_ofs[0];
		break;
	case NRF_TIMER_EVENT_COMPARE3:
		timebase_ofs[0] += timebase_mul;
		break;
	default:
		break;
	}
}

void timebase_reset(void)
{
	nrfx_timer_clear(&m_timer);
	timebase_ofs[0] = timebase_ofs[1] = 0;
}

uint32_t timebase_adjusted_us(uint32_t val)
{
	uint32_t mul = timebase_mul;
	uint32_t ofs;
	bool i = val >> 31;

	ofs = ((uint64_t)val * timebase_mul) >> 32;
	val = (mul >> 31) ? ofs : val + ofs;
	val += timebase_ofs[i];
	return val;
}

static uint32_t timebase_calc_adjmul(uint32_t freq /* Hz */)
{
	uint64_t mul = 1000000ull << 32;
	mul /= freq;
	return (uint32_t)mul;
}

void timebase_update_freq(uint32_t freq)
{
	uint64_t mul = timebase_calc_adjmul(freq);
	uint32_t v;

	// FIXME: zablokowanie przerwań lub użycie PPI
	m_timer.p_reg->TASKS_CAPTURE[4] = 1;
	m_timer.p_reg->TASKS_CLEAR = 1;
	v = m_timer.p_reg->CC[4];

	timebase_ofs[0] = timebase_adjusted_us(v);
	timebase_mul = mul;

	timebase_freq = freq;
	pstore_write(PSKEY_US_CLK_FREQ, &freq, sizeof(freq));
}

__attribute__((weak))
void timebase_freq_changed(void)
{
}

void timebase_reset_offset(int32_t adj)
{
	bool signal = timebase_step || adj < -1000 || 1000 < adj;
	timebase_ofs[1] += adj;
	timebase_ofs[0] += adj;
	step_held = timebase_step = false;
	if (signal)
		timebase_freq_changed();
}

void timebase_update(uint32_t ts, uint32_t ref_ts, uint32_t prev_ref_offs)
{
	uint32_t adj_ts = timebase_adjusted_us(ts);
	int32_t adj = ref_ts + prev_ref_offs - adj_ts;

	NRF_LOG_INFO("time-update: local (%u) %u ref %u prev_offs %u diff %d",
		ts, adj_ts, ref_ts, prev_ref_offs, -adj);

	if (timebase_step) {
		if (prev_ref_offs || step_held)
			timebase_reset_offset(adj);
		else
			step_held = true;
	} else if (-100 <= adj && adj <= 100) {
		timebase_reset_offset(adj);
	} else {
		// TODO: update frequency + offset
	}
}

__attribute__((cold))
void timebase_configure(void)
{
	uint32_t freq;

	if (pstore_read(PSKEY_US_CLK_FREQ, &freq, sizeof(freq))) {
		timebase_freq = freq;
		timebase_mul = timebase_calc_adjmul(freq);
		NRF_LOG_INFO("us-clock freq: %d Hz, adj.mul = 0x%x", freq, timebase_mul);
	} else {
		NRF_LOG_INFO("us-clock freq: 1 MHz, adj.mul = 0 (default)");
	}
}

__attribute__((cold))
void timebase_init(void)
{
	ret_code_t err_code;

	err_code = nrfx_timer_init(&m_timer, &timer_config, &timer_handler);
	APP_ERROR_CHECK(err_code);

	nrfx_timer_enable(&m_timer);
	nrfx_timer_clear(&m_timer);

	nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL2, 0x70000000, true);
	nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL3, 0xF0000000, true);

	err_code = sd_clock_hfclk_request();
	APP_ERROR_CHECK(err_code);

	err_code = sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);
	APP_ERROR_CHECK(err_code);

	/* usec_clk_out for testing */

	if (CLK_OUT_PIN == NRFX_PWM_PIN_NOT_USED)
		return;

	err_code = nrfx_pwm_init(&m_pwm, &pwm_config, NULL);
	APP_ERROR_CHECK(err_code);

	nrfx_pwm_simple_playback(&m_pwm, &pwm_seq, 0x1000,
		NRFX_PWM_FLAG_LOOP|NRFX_PWM_FLAG_NO_EVT_FINISHED);
}
