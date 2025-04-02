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
#include "bsp.h"
#include "ble_hids.h"
#include "nrf_log.h"

#include "sensor.h"

/* from main.c */
extern uint16_t m_conn_handle;

#define BASE_USB_HID_SPEC_VERSION           0x0111                                     /**< Version number of base USB HID Specification implemented by this application. */

union hids_input_report {
	struct band_input_rec in0;
};

union hids_output_report {
	struct band_output_rec out0;
};

union hids_feature_report {
	struct band_feature_rec feat0;
};

BLE_HIDS_DEF(m_hids,                              /**< Structure used to identify the HID service. */
             NRF_SDH_BLE_TOTAL_LINK_COUNT,
             sizeof(union hids_input_report),
             sizeof(union hids_output_report),
             sizeof(union hids_feature_report));

static volatile uint8_t m_sent = 0;
extern volatile bool m_do_input_notify;

static bool send_sensor_report(void);
       void hids_trigger_send(void);

/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
        ret_code_t err_code;
        uint8_t report_index, report_type;
	bool is_write = false, is_read = false, notify_on = false;

	switch (p_evt->evt_type) {
	case BLE_HIDS_EVT_NOTIF_ENABLED:
		notify_on = true;
		/* fall-through */
	case BLE_HIDS_EVT_NOTIF_DISABLED:
		report_type = p_evt->params.notification.char_id.rep_type;
        	report_index = p_evt->params.notification.char_id.rep_index;
		break;

        case BLE_HIDS_EVT_REPORT_READ:
		is_read = true;
		report_type = p_evt->params.char_auth_read.char_id.rep_type;
        	report_index = p_evt->params.char_auth_read.char_id.rep_index;
		break;

	case BLE_HIDS_EVT_REP_CHAR_WRITE:
		is_write = true;
		report_type = p_evt->params.char_write.char_id.rep_type;
        	report_index = p_evt->params.char_write.char_id.rep_index;
		break;

	default:
		return;
	}

	if (!is_write) {
		if (report_type != BLE_HIDS_REP_TYPE_INPUT || report_index != 0)
			return;

		if (!is_read) {
			m_do_input_notify = notify_on;
			NRF_LOG_INFO("INrep notify %sabled", notify_on ? "en" : "dis");
			if (notify_on)
				reset_sensor_fifo();
		} else {
			NRF_LOG_INFO("INrep read");
		}

		if (is_read || notify_on)
			send_sensor_report();
	} else {
		union hids_output_report out;
	
		if (report_type != BLE_HIDS_REP_TYPE_OUTPUT || report_index != 0)
			return;

		err_code = ble_hids_outp_rep_get(&m_hids, report_index, sizeof(out), 0, m_conn_handle, (void *)&out);
		APP_ERROR_CHECK(err_code);

		/* TODO: trigger event */
	}
}


static ble_hids_inp_rep_init_t input_report_array[] = {
	{
		.max_len = sizeof(struct band_input_rec),
		.rep_ref.report_id = 0,
		.rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT,
		.sec.cccd_wr = SEC_JUST_WORKS,
//		.sec.wr      = SEC_JUST_WORKS,
		.sec.rd      = SEC_JUST_WORKS,
	},
};

static ble_hids_outp_rep_init_t output_report_array[] = {
	{
		.max_len = sizeof(struct band_output_rec),
		.rep_ref.report_id = 0,
		.rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT,
		.sec.wr      = SEC_JUST_WORKS,
//		.sec.rd      = SEC_JUST_WORKS,
	},
};

static ble_hids_feature_rep_init_t feature_report_array[] = {
	{
		.max_len = sizeof(struct band_feature_rec),
		.rep_ref.report_id = 0,
		.rep_ref.report_type = BLE_HIDS_REP_TYPE_FEATURE,
		.sec.wr      = SEC_JUST_WORKS,
		.sec.rd      = SEC_JUST_WORKS,
	},
};

static uint8_t hid_desc[] = {
#include "band.hid.cdata"
};

static void service_error_handler(uint32_t nrf_error);

static const ble_hids_init_t hids_init_obj = {
	.evt_handler                    = on_hids_evt,
	.error_handler                  = service_error_handler,
	.inp_rep_count                  = ARRAY_SIZE(input_report_array),
	.p_inp_rep_array                = input_report_array,
	.outp_rep_count                 = ARRAY_SIZE(output_report_array),
	.p_outp_rep_array               = output_report_array,
	.feature_rep_count              = ARRAY_SIZE(feature_report_array),
	.p_feature_rep_array            = feature_report_array,
	.rep_map.data_len               = sizeof(hid_desc),
	.rep_map.p_data                 = hid_desc,
	.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION,
	.hid_information.b_country_code = 0,
	.hid_information.flags          =
		HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK,
	.included_services_count        = 0,
	.p_included_services_array      = NULL,

	.rep_map.rd_sec         = SEC_JUST_WORKS,
	.hid_information.rd_sec = SEC_JUST_WORKS,
	.ctrl_point_wr_sec    = SEC_JUST_WORKS,

//	.is_mouse                       = true,
	.protocol_mode_rd_sec = SEC_JUST_WORKS,
	.protocol_mode_wr_sec = SEC_JUST_WORKS,
	.boot_mouse_inp_rep_sec.cccd_wr = SEC_JUST_WORKS,
	.boot_mouse_inp_rep_sec.wr      = SEC_JUST_WORKS,
	.boot_mouse_inp_rep_sec.rd      = SEC_JUST_WORKS,
};

/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}

static bool send_sensor_report(void)
{
	static struct band_input_rec *data;
        ret_code_t err_code;

	if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
		return false;

	if (m_sent > 3)
		return false;

	data = sensor_next_report();
	if (!data)
		return false;

	if (data->timestamp % 512 == 0)
		NRF_LOG_INFO("INrep mark: (h 0x%x)", m_conn_handle);

	++m_sent;
	err_code = ble_hids_inp_rep_send(&m_hids, 0, sizeof(*data), (void *)data, m_conn_handle);
	if (err_code != NRF_SUCCESS) {
		--m_sent;
		NRF_LOG_INFO("INrep send failed: 0x%x (h 0x%x)", err_code, m_conn_handle);
	}

	if ( (err_code != NRF_SUCCESS)
	  && (err_code != NRF_ERROR_INVALID_STATE)
	  && (err_code != NRF_ERROR_RESOURCES)
	  && (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING )
	) {
		APP_ERROR_CHECK(err_code);
	}

	return true;
}

void hids_handle_key(bsp_event_t event)
{
	extern void sensor_trigger_read_fifo(void);

	switch (event) {
        case BSP_EVENT_KEY_0:
        case BSP_EVENT_KEY_1:
		break;

        case BSP_EVENT_KEY_2:
		send_sensor_report();
		break;

        case BSP_EVENT_KEY_3:
		sensor_trigger_read_fifo();
		break;

	default:
		__builtin_unreachable();
	}
}

static volatile bool m_connected;

void hids_connected(void)
{
	m_connected = true;
	NRF_LOG_INFO("INrep: connected (sw %u, h 0x%x)", m_sent, m_conn_handle);
	if (m_do_input_notify)
		while (send_sensor_report())
			/* redo */;
}

void hids_disconnected(void)
{
	m_connected = false;
	NRF_LOG_INFO("INrep: disconnected (sw %u, h 0x%x)", m_sent, m_conn_handle);
}

void hids_tx_complete(uint8_t count)
{
	m_sent -= count;
	if (m_do_input_notify && m_connected)
		while (send_sensor_report())
			/* redo */;
}

static void hids_tx_trampoline(void *evdata, uint16_t evsize)
{
	hids_tx_complete(0);
}

void hids_trigger_send(void)
{
	if (m_do_input_notify)
		app_sched_event_put(NULL, 0, &hids_tx_trampoline);
}

/**@brief Function for initializing HID Service.
 */
__attribute__((cold))
void hids_init(void)
{
    ret_code_t                    err_code;

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}
