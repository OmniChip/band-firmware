/*
 * SPDX-License-Identifier: GPL-3.0-or-later	// TBD
 *
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 *
 * Author: Michał Mirosław
 */
/**
 * Portions Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


// TODO: włączanie RX dopiero, gdy przewidujemy większy rozjazd ("sleep" do szacowanego |ts-ref| > próg)

#include "timesync.h"
#include "timebase.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_error.h"
#include "app_util_platform.h"
#include "app_scheduler.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_soc.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdm.h"
#include "nrf_radio.h"
#include "nrfx_ppi.h"

#define NRF_LOG_MODULE_NAME time_sync
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#define TS_SOC_OBSERVER_PRIO 0
#define PKT_WHITENING RADIO_PCNF1_WHITEEN_Enabled
#define CAPTURE_RSSI 0

#define TS_INTERVAL_US		(1000000UL)
#define PKT_END_US		(sizeof(struct sync_pkt) * 4 + 12)
#define PKT_TIME_US		(6 * 4 + PKT_END_US)
#define TS_LEN_US		(1000UL)
#define TS_LEN_EXTENSION_US	(1000UL)
#define TS_SAFETY_MARGIN_US	(200UL+PKT_END_US)	/**< The timeslot activity should be finished with this much to spare. */
#define TS_EXTEND_MARGIN_US	(400UL+PKT_END_US)	/**< The timeslot activity should request an extension this long before end of timeslot. */
#define TS_TX_LEN_US		(100UL+PKT_TIME_US)
#define TX_BOOST_BLOCKED_BEACONS	64

#define MAIN_DEBUG                           0x12345678UL

static nrf_ppi_channel_t m_ppich[2];
static const uint8_t m_rf_chn = 80;
uint8_t timesync_tag[5] __attribute__((aligned(4))) = { 1, 2, 3, 4, 5 };	// len = 5 [TODO: len = 3..5?]

struct sync_pkt {
	uint32_t clock;
	uint16_t prev_offs;
} __attribute__((packed,aligned(2)));

static volatile bool timesync_enabled;
static volatile struct sync_pkt pktbuf __attribute__((aligned(4)));
static volatile unsigned pkt_count;

static uint32_t m_total_timeslot_length;
static uint32_t m_blocked_cancelled_count;
static const nrf_radio_request_t *req_on_blocked;

/**< This will be used when requesting the first timeslot or any time a timeslot is blocked or cancelled. */
static nrf_radio_request_t m_timeslot_req_earliest_tx = {
        NRF_RADIO_REQ_TYPE_EARLIEST,
        .params.earliest = {
            NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
            NRF_RADIO_PRIORITY_NORMAL,
            TS_TX_LEN_US,
            NRF_RADIO_EARLIEST_TIMEOUT_MAX_US
        }};

static nrf_radio_request_t m_timeslot_req_earliest_rx = {
        NRF_RADIO_REQ_TYPE_EARLIEST,
        .params.earliest = {
            NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
            NRF_RADIO_PRIORITY_NORMAL,
            TS_LEN_US,
            NRF_RADIO_EARLIEST_TIMEOUT_MAX_US
        }};

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_request_t m_timeslot_req_normal_tx = {
        NRF_RADIO_REQ_TYPE_NORMAL,
        .params.normal = {
            NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
            NRF_RADIO_PRIORITY_NORMAL,
            TS_INTERVAL_US,
            TS_TX_LEN_US
        }};

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_return_sched_next_tx = {
        NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END,
        .params.request = { &m_timeslot_req_normal_tx }
};

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_return_sched_next_rx = {
        NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END,
        .params.request = { &m_timeslot_req_earliest_rx }
};

/**< This will be used at the end of each timeslot to request an extension of the timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_extend = {
        NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND,
        .params.extend = { TS_LEN_EXTENSION_US }
};

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_return_no_action = {
        NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE,
        .params.request = { NULL }
};

static bool handling_event(volatile uint32_t *event)
{
	if (*event == 0)
		return false;

	*event = 0;
	(void)*event;
	return true;
}

static void update_radio_parameters(bool tx)
{
    uint32_t v;

    // enable
    NRF_RADIO->POWER    = RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos;
    *((volatile uint32_t *)NRF_RADIO + 0x72C/4) |= 1 << 10;	// WA errata 182

    // TX power
    NRF_RADIO->TXPOWER  = RADIO_TXPOWER_TXPOWER_0dBm   << RADIO_TXPOWER_TXPOWER_Pos;

    // RF bitrate
    NRF_RADIO->MODE     = RADIO_MODE_MODE_Nrf_2Mbit       << RADIO_MODE_MODE_Pos;

    // Fast startup mode
    NRF_RADIO->MODECNF0 = RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;

    // CRC configuration
    NRF_RADIO->CRCCNF  = RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos;
    NRF_RADIO->CRCINIT = 0xFFFFFFUL;      // Initial value
    NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1

    // Packet format
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_LFLEN_Pos) | (0 << RADIO_PCNF0_S1LEN_Pos);
    NRF_RADIO->PCNF1 = (PKT_WHITENING                    << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big           << RADIO_PCNF1_ENDIAN_Pos)  |
                       (4                                << RADIO_PCNF1_BALEN_Pos)   |
                       (sizeof(pktbuf)                   << RADIO_PCNF1_STATLEN_Pos) |
                       (sizeof(pktbuf)                   << RADIO_PCNF1_MAXLEN_Pos);
    NRF_RADIO->PACKETPTR = (uint32_t)&pktbuf;

    // Radio address config
    NRF_RADIO->PREFIX0 = timesync_tag[4];
    NRF_RADIO->BASE0   = *(uint32_t *)timesync_tag;
    NRF_RADIO->BASE1   = ~*(uint32_t *)timesync_tag;	// WA errata 143

    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = (1 << 0);

    NRF_RADIO->FREQUENCY = m_rf_chn;
    NRF_RADIO->TXPOWER   = RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos;

    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;

    v = RADIO_SHORTS_READY_START_Msk;
    if (tx)
        v |= RADIO_SHORTS_END_DISABLE_Msk;
    else if (CAPTURE_RSSI)
        v |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_DISABLED_RSSISTOP_Msk;
    NRF_RADIO->SHORTS = v;

    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    v = RADIO_INTENSET_DISABLED_Msk;
    if (!tx)
        v |= RADIO_INTENSET_END_Msk;
    NRF_RADIO->INTENSET = v;
    NVIC_EnableIRQ(RADIO_IRQn);
}

/**@brief IRQHandler used for execution context management.
  *        Any available handler can be used as we're not using the associated hardware.
  *        This handler is used to stop and disable UESB
  */
static void timeslot_end_handler(void)
{
	NRF_RADIO->INTENCLR      = 0xFFFFFFFF;

	nrfx_ppi_channel_disable(m_ppich[1]);
	nrfx_ppi_channel_disable(m_ppich[0]);
}

static void timesync_start_tx(void)
{
	update_radio_parameters(true);

	nrfx_ppi_channel_enable(m_ppich[0]);
	nrfx_ppi_channel_enable(m_ppich[1]);

	NRF_RADIO->TASKS_TXEN = 1;

	while (NRF_RADIO->EVENTS_READY == 0)
		// PPI is used to trigger sync timer capture when radio is ready
		// Radio will automatically start transmitting once ready, so the
		// captured timer value must be copied into radio packet buffer ASAP
		__NOP();

	pktbuf.clock = timebase_adjusted_us(timesync_captured_ts());
}

/**@brief   Function for handling timeslot events (TX).
 */
static nrf_radio_signal_callback_return_param_t *tx_radio_callback(uint8_t signal_type)
{
	// NOTE: This callback runs at lower-stack priority (the highest priority possible).
	switch (signal_type) {
	case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
		NRF_TIMER0->TASKS_CAPTURE[2] = 1;
		req_on_blocked = &m_timeslot_req_normal_tx;
		m_timeslot_req_normal_tx.params.normal.priority = NRF_RADIO_PRIORITY_NORMAL;
		m_timeslot_req_normal_tx.params.normal.distance_us = TS_INTERVAL_US;
		timesync_start_tx();
		break;

	case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
		if (handling_event(&NRF_RADIO->EVENTS_DISABLED)) {
			timeslot_end_handler();
			pktbuf.prev_offs = timebase_adjusted_us(timesync_captured_ts()) - pktbuf.clock;
			NRF_TIMER0->TASKS_CAPTURE[3] = 1;
			NRF_LOG_INFO("timesync: TX %u offs %u; T0: %u .. %u",
				pktbuf.clock, pktbuf.prev_offs,
				NRF_TIMER0->CC[2], NRF_TIMER0->CC[3]);
			// Schedule next timeslot
			return &m_rsc_return_sched_next_tx;
		}
		break;

	default:
		app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
		break;
	};

	return &m_rsc_return_no_action;
}

static void timesync_start_rx(void)
{
	update_radio_parameters(false);
	nrfx_ppi_channel_enable(m_ppich[0]);
	nrfx_ppi_channel_enable(m_ppich[1]);

	NRF_RADIO->TASKS_RXEN = 1;
}

static void timesync_rx_trampoline(void *evdata, uint16_t evsize)
{
	uint32_t *ts = evdata;

	timebase_update(*ts, pktbuf.clock, pktbuf.prev_offs);
}

void RADIO_IRQHandler(void)
{
	uint32_t ts;

	if (!handling_event(&NRF_RADIO->EVENTS_END))
		return;

	// receiving

	if (!nrf_radio_crc_status_check()) {
		NRF_RADIO->TASKS_START = 1;
		return;
	}

	ts = timesync_captured_ts();
	++pkt_count;
	app_sched_event_put(&ts, sizeof(ts), &timesync_rx_trampoline);

	NRF_RADIO->TASKS_DISABLE = 1;
}


/**@brief   Function for handling timeslot events (RX).
 */
static nrf_radio_signal_callback_return_param_t *rx_radio_callback(uint8_t signal_type)
{
    // NOTE: This callback runs at lower-stack priority (the highest priority possible).
    switch (signal_type) {
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
        // TIMER0 is pre-configured for 1Mhz and started with the timeslot.
	NRF_TIMER0->TASKS_CAPTURE[2]    = 1;
        NRF_TIMER0->CC[0]               = TS_LEN_US - TS_SAFETY_MARGIN_US;
        NRF_TIMER0->CC[1]               = TS_LEN_US - TS_EXTEND_MARGIN_US;
        NRF_TIMER0->EVENTS_COMPARE[0]   = 0;
        NRF_TIMER0->EVENTS_COMPARE[1]   = 0;
        NRF_TIMER0->INTENSET = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos) |
                               (TIMER_INTENSET_COMPARE1_Set << TIMER_INTENSET_COMPARE1_Pos);
        NVIC_EnableIRQ(TIMER0_IRQn);

        m_total_timeslot_length = TS_LEN_US;
	timesync_start_rx();
        break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
	if (handling_event(&NRF_TIMER0->EVENTS_COMPARE[0])) {
            // This is the "timeslot is about to end" timeout
            uint32_t last_rxd = NRF_TIMER0->CC[0] - NRF_TIMER0->CC[2];
            if (last_rxd > PKT_END_US)
                NRF_RADIO->TASKS_DISABLE = 1;
            break;
        }

        if (handling_event(&NRF_TIMER0->EVENTS_COMPARE[1])) {
            // This is the "try to extend timeslot" timeout

            if (m_total_timeslot_length < (NRF_RADIO_DISTANCE_MAX_US - TS_LEN_EXTENSION_US))
                // Request timeslot extension if total length does not exceed 128 seconds
                return &m_rsc_extend;

            // Don't do anything. Timeslot will end and new one requested upon the next timer0 compare.
        }
	break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
        RADIO_IRQHandler();
	if (handling_event(&NRF_RADIO->EVENTS_DISABLED)) {
		timeslot_end_handler();
		return &m_rsc_return_sched_next_rx;
	}
        break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
        // Don't do anything. Our timer will expire before timeslot ends
        break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
        // Extension succeeded: update timer
        NRF_TIMER0->CC[0]               += TS_LEN_EXTENSION_US;
        NRF_TIMER0->CC[1]               += TS_LEN_EXTENSION_US;
        NRF_TIMER0->EVENTS_COMPARE[0]   = 0;
        NRF_TIMER0->EVENTS_COMPARE[1]   = 0;

        // Keep track of total length
        m_total_timeslot_length += TS_LEN_EXTENSION_US;
        break;

    default:
        app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
        break;
    };

    // Fall-through return: return with no action request
    return &m_rsc_return_no_action;
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void timesync_sys_evt(uint32_t sys_evt, void * p_context)
{
    uint32_t err_code;

    switch(sys_evt)
    {
        case NRF_EVT_RADIO_BLOCKED:
        case NRF_EVT_RADIO_CANCELED:
        {
            // Blocked events are rescheduled with normal priority. They could also
            // be rescheduled with high priority if necessary.
            if (req_on_blocked == &m_timeslot_req_normal_tx) {
                uint32_t v = m_timeslot_req_normal_tx.params.normal.distance_us += TS_INTERVAL_US;
                NRF_LOG_INFO("timesync: rescheduling TX to %u", v);
                if (v >= TX_BOOST_BLOCKED_BEACONS * TS_INTERVAL_US)
                    m_timeslot_req_normal_tx.params.normal.priority = NRF_RADIO_PRIORITY_HIGH;
                else if (v > NRF_RADIO_DISTANCE_MAX_US)
                    req_on_blocked = &m_timeslot_req_earliest_tx;
            }

            err_code = sd_radio_request(req_on_blocked);
            APP_ERROR_CHECK(err_code);

            m_blocked_cancelled_count++;
            break;
        }

        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            NRF_LOG_ERROR("NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN\r\n");
            app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
            break;

        case NRF_EVT_RADIO_SESSION_CLOSED:
		NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_CLOSED");
		timesync_enabled = false;
		break;

        case NRF_EVT_RADIO_SESSION_IDLE:
            NRF_LOG_ERROR("NRF_EVT_RADIO_SESSION_IDLE\r\n");
            app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
            break;

        default:
            // No implementation needed.
            NRF_LOG_DEBUG("Event: 0x%08x\r\n", sys_evt);
            break;
    }
}

NRF_SDH_SOC_OBSERVER(timesync_soc_obs,
                     TS_SOC_OBSERVER_PRIO,
                     timesync_sys_evt, 0);

__attribute__((cold))
void timesync_init(void)
{
	nrf_ppi_channel_t ppich;
	ret_code_t err_code;

	/* TX:
	 *   RADIO.READY -> RADIO.START (short)
	 *   RADIO.READY -> PPI -> TIMERx.CAPTUREn (timebase)
	 *   RADIO.READY -> (poll) pktbuf = TIMERx.CCn (timebase)
	 *   RADIO.END -> RADIO.DISABLE (short)
	 *   RADIO.DISABLED -> IRQ: finish
	 */
	err_code = nrfx_ppi_channel_alloc(&ppich);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_ppi_channel_assign(ppich,
		nrf_radio_event_address_get(NRF_RADIO_EVENT_READY),
		timesync_capture_task());
	APP_ERROR_CHECK(err_code);
	m_ppich[0] = ppich;

	/* RX:
	 *   RADIO.READY -> RADIO.START (short)
	 *   RADIO.ADDRESS -> PPI -> TIMERx.CAPTUREn (timebase)
	 *   RADIO.END -> RADIO.DISABLE (short)
	 *   RADIO.END -> IRQ: handle pktbuf + TIMERx.CCn (timebase)
	 *   RADIO.DISABLED -> IRQ: finish
	 */
	err_code = nrfx_ppi_channel_alloc(&ppich);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_ppi_channel_assign(ppich,
		nrf_radio_event_address_get(NRF_RADIO_EVENT_ADDRESS),
		timesync_capture_task());
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_ppi_channel_fork_assign(ppich,
		(uint32_t)nrf_timer_task_address_get(NRF_TIMER0, NRF_TIMER_TASK_CAPTURE2));
	APP_ERROR_CHECK(err_code);
	m_ppich[1] = ppich;
}

uint32_t timesync_enable_master(void)
{
	ret_code_t err_code;

	err_code = sd_radio_session_open(tx_radio_callback);
	if (err_code != NRF_SUCCESS)
		return err_code;

	timesync_enabled = true;

	req_on_blocked = &m_timeslot_req_earliest_tx;
	m_timeslot_req_normal_tx.params.normal.priority = NRF_RADIO_PRIORITY_NORMAL;
	m_timeslot_req_normal_tx.params.normal.distance_us = TS_INTERVAL_US;

	err_code = sd_radio_request(req_on_blocked);
	APP_ERROR_CHECK(err_code);

	return NRF_SUCCESS;
}

uint32_t timesync_enable_slave(void)
{
	ret_code_t err_code;

	err_code = sd_radio_session_open(rx_radio_callback);
	if (err_code != NRF_SUCCESS)
		return err_code;

	timesync_enabled = true;

	req_on_blocked = &m_timeslot_req_earliest_rx;

	err_code = sd_radio_request(req_on_blocked);
	APP_ERROR_CHECK(err_code);

	return NRF_SUCCESS;
}

void timesync_disable(void)
{
	extern void idle_state_handle(void);

	if (sd_radio_session_close() == NRF_ERROR_FORBIDDEN)
		return;

	do idle_state_handle();
	while (timesync_enabled);
}
