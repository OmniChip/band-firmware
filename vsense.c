/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 *
 * Author: Michał Mirosław
 */

#include <stdint.h>
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "hal/nrf_saadc.h"
#include "nrfx_ppi.h"
#include "vsense.h"


#define ADC_TRIGGER_INTERVAL	APP_TIMER_TICKS(1000 /* ms */)	// TODO: longer interval
#define ADC_BITS		10
#define ADC_RES			NRF_SAADC_RESOLUTION_10BIT
#define ADC_GAIN		NRF_SAADC_GAIN1_4

#if defined(ADC_VBAT_IN) || defined(ADC_VBUS_IN)
STATIC_ASSERT(ADC_NUM <= 1 << (32 - ADC_BITS), "ADC value will overflow");
#endif

static const nrf_saadc_channel_config_t adc_ch_config[] = {
#ifdef ADC_VBAT_IN
	{ /* [0] VBAT */
		NRF_SAADC_RESISTOR_DISABLED,
		NRF_SAADC_RESISTOR_DISABLED,
		ADC_GAIN,
		NRF_SAADC_REFERENCE_INTERNAL,
		ADC_ACQTIME,
		NRF_SAADC_MODE_SINGLE_ENDED,
		NRF_SAADC_BURST_DISABLED,
		ADC_VBAT_IN,
		NRF_SAADC_INPUT_DISABLED,
	},
#endif
#ifdef ADC_VBUS_IN
	{ /* [1] VBUS */
		NRF_SAADC_RESISTOR_DISABLED,
		NRF_SAADC_RESISTOR_DISABLED,
		ADC_GAIN,
		NRF_SAADC_REFERENCE_INTERNAL,
		ADC_ACQTIME,
		NRF_SAADC_MODE_SINGLE_ENDED,
		NRF_SAADC_BURST_DISABLED,
		ADC_VBUS_IN,
		NRF_SAADC_INPUT_DISABLED,
	},
#endif
	{ /* [2] VCC */
		NRF_SAADC_RESISTOR_DISABLED,
		NRF_SAADC_RESISTOR_DISABLED,
		NRF_SAADC_GAIN1_4,
		NRF_SAADC_REFERENCE_INTERNAL,
		NRF_SAADC_ACQTIME_10US,	// min. b/c errata
		NRF_SAADC_MODE_SINGLE_ENDED,
		NRF_SAADC_BURST_DISABLED,
		NRF_SAADC_INPUT_VDD,
		NRF_SAADC_INPUT_DISABLED,
	},
};

static nrf_saadc_value_t adc_buf[(ARRAY_SIZE(adc_ch_config) + 1) & ~1];
volatile uint16_t vbat_mv, vbus_mv, vdd_mv;

APP_TIMER_DEF(m_trigger_timer);

static inline uint16_t calc_mv(int16_t adcv, uint32_t mul, uint32_t div)
{
	return adcv * mul / div >> ADC_BITS;
}

static void vsense_finish_measurement(void *evdata, uint16_t evsize)
{
	int i = 0;

#ifdef ADC_VBAT_IN
	vbat_mv = calc_mv(adc_buf[i++], ADC_NUM, ADC_DENOM);
#endif
#ifdef ADC_VBUS_IN
	vbus_mv = calc_mv(adc_buf[i++], ADC_NUM, ADC_DENOM);
#endif
	vdd_mv = calc_mv(adc_buf[i++], 4 * 1000 * 3 / 5, 1);

	NRF_LOG_DEBUG("Vsense: VBAT %d mV, VBUS %u mV, VDD %u mV",
		vbat_mv, vbus_mv, vdd_mv);

	bands_send_voltdata();

	// TODO: calc battery level
	// TODO: notify user?
}

void nrfx_saadc_irq_handler(void)
{
	NRF_LOG_DEBUG("Vsense: IRQ %d, %d, %d; %p #%u",
		adc_buf[0], adc_buf[1], adc_buf[2],
		NRF_SAADC->RESULT.PTR, NRF_SAADC->RESULT.MAXCNT
	);

	nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
	nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
	app_sched_event_put(NULL, 0, &vsense_finish_measurement);
}

static void vsense_trigger(void *p_context)
{
	NRF_LOG_DEBUG("Vsense: trigger");
	nrf_saadc_task_trigger(NRF_SAADC_TASK_CALIBRATEOFFSET);
}

__attribute__((cold))
void vsense_start_timer(void)
{
	ret_code_t err_code;

	vsense_trigger(NULL);

	err_code = app_timer_start(m_trigger_timer, ADC_TRIGGER_INTERVAL, NULL);
	APP_ERROR_CHECK(err_code);
}

__attribute__((cold))
void vsense_init(void)
{
	nrf_ppi_channel_t ppich __attribute__((unused));
	nrf_ppi_channel_group_t ppigrp __attribute__((unused));
	ret_code_t err_code;
	int i;

	nrf_saadc_resolution_set(ADC_RES);
	nrf_saadc_oversample_set(NRF_SAADC_OVERSAMPLE_DISABLED);

	nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
	nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
	nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
	nrf_saadc_event_clear(NRF_SAADC_EVENT_CALIBRATEDONE);
	nrf_saadc_int_enable(NRF_SAADC_INT_END);
	nrf_saadc_enable();
	nrf_saadc_buffer_init(adc_buf, ARRAY_SIZE(adc_ch_config));
	NVIC_EnableIRQ(SAADC_IRQn);
#if 1
	err_code = nrfx_ppi_channel_alloc(&ppich);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_channel_assign(ppich,
		nrf_saadc_event_address_get(NRF_SAADC_EVENT_STARTED),
		nrf_saadc_task_address_get(NRF_SAADC_TASK_SAMPLE));
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_channel_enable(ppich);
	APP_ERROR_CHECK(err_code);
#endif
	// errata WA
	err_code = nrfx_ppi_group_alloc(&ppigrp);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_group_disable(ppigrp);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_ppi_channel_alloc(&ppich);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_channel_assign(ppich,
		nrf_saadc_event_address_get(NRF_SAADC_EVENT_CALIBRATEDONE),
		nrf_saadc_task_address_get(NRF_SAADC_TASK_STOP));
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_channel_fork_assign(ppich,
		nrfx_ppi_task_addr_group_enable_get(ppigrp));
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_channel_enable(ppich);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_ppi_channel_alloc(&ppich);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_channel_assign(ppich,
		nrf_saadc_event_address_get(NRF_SAADC_EVENT_STOPPED),
		nrf_saadc_task_address_get(NRF_SAADC_TASK_START));
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_channel_fork_assign(ppich,
		nrfx_ppi_task_addr_group_disable_get(ppigrp));
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_channels_include_in_group(1 << ppich, ppigrp);
	APP_ERROR_CHECK(err_code);

	for (i = 0; i < ARRAY_SIZE(adc_ch_config); ++i) {
		nrf_saadc_channel_init(i, &adc_ch_config[i]);
		nrf_saadc_channel_limits_set(i, 0, 0x7fff);
	}

	err_code = app_timer_create(&m_trigger_timer, APP_TIMER_MODE_REPEATED,
				    vsense_trigger);
	APP_ERROR_CHECK(err_code);
}
