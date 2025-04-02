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
#include "ble_srv_common.h"
#include "bsp.h"
#include "nrf_log.h"
#include "nrf_sdh_ble.h"

#include "sensor.h"
#include "motor.h"
#include "vsense.h"
#include "pstore.h"
#include "timebase.h"
#include "timesync.h"

#define SENSOR_SERVICE_UUID	0x1833
#define TIME_SERVICE_UUID	0x1805	// Current Time
#define HAPTIC_SERVICE_UUID	0x1844
#define MAG_SERVICE_UUID	0x181A	// Environmental Sensing

#define INCLUDE_PF_DESCRIPTORS 0
#define BAND_SEC_MODE SEC_OPEN
//#define BAND_SEC_MODE SEC_JUST_WORKS
#define MOTOR_DEVEL 1

#if MOTOR_DEVEL && (!MOTOR_I2C_ADDR || ACCEL_TEST)
#undef MOTOR_DEVEL
#define MOTOR_DEVEL 0
#endif

/* from main.c */
extern uint16_t m_conn_handle;

static uint16_t m_bands_service_handle;
static ble_gatts_char_handles_t m_bands_input_char_handles;
static ble_gatts_char_handles_t m_bands_input_zero_char_handles;
static ble_gatts_char_handles_t m_bands_enable_char_handles;
static ble_gatts_char_handles_t m_bands_decimation_char_handles;
static ble_gatts_char_handles_t m_bands_volt_char_handles;

static uint16_t m_time_service_handle;
static ble_gatts_char_handles_t m_time_timebase_char_handles;
static ble_gatts_char_handles_t m_time_timesync_tag_char_handles;
static ble_gatts_char_handles_t m_time_timesync_enable_char_handles;
static ble_gatts_char_handles_t m_time_timesync_value_char_handles;

static uint16_t m_mags_service_handle;
static ble_gatts_char_handles_t m_mags_data_char_handles;
static ble_gatts_char_handles_t m_mags_hard_char_handles;
static ble_gatts_char_handles_t m_mags_soft_char_handles;

static uint16_t m_haptics_service_handle;
static ble_gatts_char_handles_t m_haptics_libplay_char_handles;
static ble_gatts_char_handles_t m_haptics_config_char_handles;

extern uint8_t timesync_tag[5];
static uint8_t timesync_val;
static uint16_t tsv_len = sizeof(timesync_val);
static uint8_t timesync_enable;

static bool do_notify, do_retry, do_md_notify, do_volt_notify;
static struct band_input_rec sendbuf;
static uint16_t sb_len = sizeof(sendbuf);
static int16_t sensor_zero[6];
static uint8_t sensor_enabled = true;
static uint16_t sensor_decimate[2];	// num, denom; sum >= 1 odrzuca
static uint16_t sensor_dec_counter;

static struct magdata_rec mag_data;
static uint16_t md_len = sizeof(mag_data);
extern int16_t sensor_magzero[3];
extern uint8_t sensor_magcorr[9];

static uint16_t volt_data[3];
static uint16_t volt_len = sizeof(volt_data);

static ble_gatts_hvx_params_t input_notify = {
	.type = BLE_GATT_HVX_NOTIFICATION,
	.offset = 0,
	.p_len = &sb_len,
	.p_data = (void *)&sendbuf,
};

static ble_gatts_hvx_params_t tsync_notify = {
	.type = BLE_GATT_HVX_NOTIFICATION,
	.offset = 0,
	.p_len = &tsv_len,
	.p_data = (void *)&timesync_val,
};

__attribute__((unused))
static ble_gatts_hvx_params_t magdata_notify = {
	.type = BLE_GATT_HVX_NOTIFICATION,
	.offset = 0,
	.p_len = &md_len,
	.p_data = (void *)&mag_data,
};

static ble_gatts_hvx_params_t volt_notify = {
	.type = BLE_GATT_HVX_NOTIFICATION,
	.offset = 0,
	.p_len = &volt_len,
	.p_data = (void *)&volt_data,
};

static const ble_add_char_params_t sensor_char_params = {
    .uuid              = 0x29FE,
    .max_len           = sizeof(struct band_input_rec),
    .init_len          = sizeof(struct band_input_rec),
    .p_init_value      = (void *)&sendbuf,
    .char_props.notify = 1,
    .char_props.read   = 1,
    .cccd_write_access = BAND_SEC_MODE,
    .read_access       = BAND_SEC_MODE,
    .is_defered_read   = 1,
    .is_value_user     = 1,
};

#define PSKEY_SENSOR_ZERO ((SENSOR_SERVICE_UUID << 16) | 0x29FD)
static const ble_add_char_params_t sensor_zero_char_params = {
    .uuid              = 0x29FD,
    .max_len           = sizeof(sensor_zero),
    .init_len          = sizeof(sensor_zero),
    .p_init_value      = (void *)&sensor_zero,
    .char_props.read   = 1,
    .char_props.write  = 1,
    .read_access       = BAND_SEC_MODE,
    .write_access      = BAND_SEC_MODE,
    .is_defered_write  = 1,
    .is_value_user     = 1,
};

#define PSKEY_SENSOR_DECIMATE ((SENSOR_SERVICE_UUID << 16) | 0x29F8)
static const ble_add_char_params_t sensor_dec_char_params = {
    .uuid              = 0x29F8,
    .max_len           = sizeof(sensor_decimate),
    .init_len          = sizeof(sensor_decimate),
    .p_init_value      = (void *)&sensor_decimate,
    .char_props.read   = 1,
    .char_props.write  = 1,
    .read_access       = BAND_SEC_MODE,
    .write_access      = BAND_SEC_MODE,
    .is_defered_write  = 1,
    .is_value_user     = 1,
};

static const ble_add_char_params_t timebase_char_params = {
    .uuid              = 0x29F9,
    .max_len           = sizeof(timebase_freq),
    .init_len          = sizeof(timebase_freq),
    .p_init_value      = (void *)&timebase_freq,
    .char_props.read   = 1,
    .char_props.write  = 1,
    .read_access       = BAND_SEC_MODE,
    .write_access      = BAND_SEC_MODE,
    .is_defered_write  = 1,
    .is_value_user     = 1,
};

__attribute__((unused))
static const ble_add_char_params_t sensor_enable_char_params = {
    .uuid              = 0x29F7,
    .max_len           = sizeof(sensor_enabled),
    .init_len          = sizeof(sensor_enabled),
    .p_init_value      = (void *)&sensor_enabled,
    .char_props.read   = 1,
    .char_props.write  = 1,
    .read_access       = BAND_SEC_MODE,
    .write_access      = BAND_SEC_MODE,
    .is_defered_write  = 1,
    .is_value_user     = 1,
};

static const ble_add_char_params_t timesync_tag_char_params = {
    .uuid              = 0x29FA,
    .max_len           = sizeof(timesync_tag),
    .init_len          = sizeof(timesync_tag),
    .p_init_value      = (void *)&timesync_tag,
    .char_props.read   = 1,
    .char_props.write  = 1,
    .read_access       = BAND_SEC_MODE,
    .write_access      = BAND_SEC_MODE,
    .is_defered_write  = 1,
    .is_value_user     = 1,
};

static const ble_add_char_params_t timesync_value_char_params = {
    .uuid              = 0x29FB,
    .max_len           = sizeof(timesync_val),
    .init_len          = sizeof(timesync_val),
    .p_init_value      = (void *)&timesync_val,
    .char_props.notify = 1,
    .char_props.read   = 1,
    .cccd_write_access = BAND_SEC_MODE,
    .read_access       = BAND_SEC_MODE,
    .is_defered_read   = 1,
    .is_value_user     = 1,
};

static const ble_add_char_params_t timesync_enable_char_params = {
    .uuid              = 0x29FC,
    .max_len           = sizeof(timesync_enable),
    .init_len          = sizeof(timesync_enable),
    .p_init_value      = (void *)&timesync_enable,
    .char_props.read   = 1,
    .char_props.write  = 1,
    .read_access       = BAND_SEC_MODE,
    .write_access      = BAND_SEC_MODE,
    .is_defered_write  = 1,
    .is_value_user     = 1,
};

static const ble_add_char_params_t vsense_char_params = {
    .uuid              = 0x29F6,
    .max_len           = sizeof(volt_data),
    .init_len          = sizeof(volt_data),
    .p_init_value      = (void *)&volt_data,
    .char_props.notify = 1,
    .char_props.read   = 1,
    .cccd_write_access = BAND_SEC_MODE,
    .read_access       = BAND_SEC_MODE,
    .is_defered_read   = 1,
    .is_value_user     = 1,
};

static const ble_add_char_params_t magdata_char_params = {
    .uuid              = 0x2AA1,
    .max_len           = sizeof(mag_data),
    .init_len          = sizeof(mag_data),
    .p_init_value      = (void *)&mag_data,
    .char_props.notify = 1,
    .char_props.read   = 1,
    .cccd_write_access = BAND_SEC_MODE,
    .read_access       = BAND_SEC_MODE,
    .is_defered_read   = 1,
    .is_value_user     = 1,
};

#define PSKEY_SENSOR_MAGZERO ((MAG_SERVICE_UUID << 16) | 0x29F2)
static const ble_add_char_params_t maghard_char_params = {
    .uuid              = 0x29F2,
    .max_len           = sizeof(sensor_magzero),
    .init_len          = sizeof(sensor_magzero),
    .p_init_value      = (void *)&sensor_magzero,
    .char_props.read   = 1,
    .char_props.write  = 1,
    .read_access       = BAND_SEC_MODE,
    .write_access      = BAND_SEC_MODE,
    .is_defered_write  = 1,
    .is_value_user     = 1,
};

#define PSKEY_SENSOR_MAGCORR ((MAG_SERVICE_UUID << 16) | 0x29F3)
static const ble_add_char_params_t magsoft_char_params = {
    .uuid              = 0x29F3,
    .max_len           = sizeof(sensor_magcorr),
    .init_len          = sizeof(sensor_magcorr),
    .p_init_value      = (void *)&sensor_magcorr,
    .char_props.read   = 1,
    .char_props.write  = 1,
    .read_access       = BAND_SEC_MODE,
    .write_access      = BAND_SEC_MODE,
    .is_defered_write  = 1,
    .is_value_user     = 1,
};

static const ble_add_char_params_t haptic_char_params = {
    .uuid              = 0x29F0,
    .max_len           = 12,
    .init_len          = 0,
    .p_init_value      = NULL,
    .char_props.write_wo_resp = 1,
    .write_access      = BAND_SEC_MODE,
    .is_defered_write  = 1,
    .is_value_user     = 1,
};

static const ble_add_char_params_t haptic_conf_char_params = {
    .uuid              = 0x29F1,
    .max_len           = sizeof(motor_conf),
    .init_len          = sizeof(motor_conf),
    .p_init_value      = (void *)&motor_conf,
    .char_props.read   = 1,
    .char_props.write_wo_resp = 1,
    .read_access       = BAND_SEC_MODE,
    .write_access      = BAND_SEC_MODE,
    .is_defered_write  = 1,
    .is_value_user     = 1,
};

static bool update_sensor_data(void)
{
	struct band_input_rec *data;
	int i;

	data = sensor_next_report();
	if (!data)
		return false;

	memcpy(&sendbuf, data, sizeof(sendbuf));

	for (i = 0; i < sizeof(sensor_zero)/sizeof(*sensor_zero); ++i) {
		int16_t *pv = &sendbuf.gx + i;
		int16_t v = *pv;
		int16_t dz = sensor_zero[i];

		if (__builtin_sub_overflow(v, dz, &v))
			v = dz < 0 ? 0x7FFF : 0x8000;

		*pv = v;
	}

	return true;
}

static void update_voltdata(void)
{
	volt_data[0] = vbat_mv;
	volt_data[1] = vbus_mv;
	volt_data[2] = vdd_mv;
}

static void send_notify(void)
{
	if (!do_notify)
		return;

	if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
		return;

	for (;;) {
		ret_code_t err_code;

		if (!do_retry) {
			if (!update_sensor_data())
				break;

			if (sendbuf.timestamp % 512 == 0)
				NRF_LOG_INFO("BANDS mark: ts %u (h 0x%x) dc %u",
					sendbuf.timestamp, m_conn_handle, sensor_dec_counter);

			if (sensor_decimate[0]) {
				sensor_dec_counter += sensor_decimate[0];
				if (sensor_dec_counter >= sensor_decimate[1]) {
					sensor_dec_counter -= sensor_decimate[1];
					continue;
				}
			}
		}

		err_code = sd_ble_gatts_hvx(m_conn_handle, &input_notify);
		if (err_code != NRF_SUCCESS) {
			static unsigned fail_count;

			do_retry = true;
			if (++fail_count % 256 == 0)
				NRF_LOG_INFO("BANDS notify failed: 0x%x (h 0x%x)", err_code, m_conn_handle);
			break;
		} else {
			do_retry = false;
		}
	}
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_evt_t const *p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (    (p_evt_write->handle == m_bands_input_char_handles.cccd_handle)
        &&  (p_evt_write->len == BLE_CCCD_VALUE_LEN))
    {
        do_notify = ble_srv_is_notification_enabled(p_evt_write->data);

	NRF_LOG_INFO("BANDS notify %sabled", do_notify ? "en" : "dis");
	if (do_notify)
		reset_sensor_fifo();
    }
    else if (MAG_DATA_REG
        && (p_evt_write->handle == m_mags_data_char_handles.cccd_handle)
        &&  (p_evt_write->len == BLE_CCCD_VALUE_LEN))
    {
        do_md_notify = ble_srv_is_notification_enabled(p_evt_write->data);

	NRF_LOG_INFO("BANDS magdata notify %sabled", do_md_notify ? "en" : "dis");
    }
    else if ((p_evt_write->handle == m_bands_volt_char_handles.cccd_handle)
        &&  (p_evt_write->len == BLE_CCCD_VALUE_LEN))
    {
        do_volt_notify = ble_srv_is_notification_enabled(p_evt_write->data);

	NRF_LOG_INFO("BANDS voltage notify %sabled", do_volt_notify ? "en" : "dis");
    }
    else if (p_evt_write->handle == m_bands_enable_char_handles.value_handle) {
#if 0
	const uint8_t *p = p_evt_write->data;
	if (*p && !sensor_enabled)
		sensor_enable();
	else if (!*p && sensor_enabled)
		sensor_disable();
	sensor_enabled = *p;
#endif
	timebase_reset();
    }
    else if (MOTOR_I2C_ADDR && !ACCEL_TEST && p_evt_write->handle == m_haptics_libplay_char_handles.value_handle) {
	NRF_LOG_INFO("HAPTICS write (len %u)", p_evt_write->len);
	if (p_evt_write->len > 0 && p_evt_write->len <= 8)
		motor_play_lib_effect(p_evt_write->data, p_evt_write->len);
    }
    else if (MOTOR_DEVEL && p_evt_write->handle == m_haptics_config_char_handles.value_handle
	&&  (p_evt_write->len == sizeof(motor_conf))) {
	uint32_t oldcf = motor_conf;
	memcpy(&motor_conf, p_evt_write->data, sizeof(motor_conf));
	if (!motor_reconfigure(motor_conf)) {
		motor_conf = oldcf;
		NRF_LOG_INFO("HAPTICS config ignored - busy");
	}
    }
}

static void on_rw_auth(ble_evt_t const *p_ble_evt)
{
	ble_gatts_evt_t const *gatts_ev = &p_ble_evt->evt.gatts_evt;
	ble_gatts_evt_rw_authorize_request_t const *ar = &gatts_ev->params.authorize_request;
	ble_gatts_rw_authorize_reply_params_t reply = { 0, };
	uint16_t handle;
	bool do_wr = false;

	switch (ar->type) {
	case BLE_GATTS_AUTHORIZE_TYPE_READ:
		handle = ar->request.read.handle;
		break;

	case BLE_GATTS_AUTHORIZE_TYPE_WRITE:
		handle = ar->request.write.handle;
		if (ar->request.write.offset == 0)
			do_wr = true;
		break;

	default:
		return;
	}

	if (handle == m_bands_input_char_handles.value_handle) {
		// sensor value
		reply.params.write.len = sizeof(sendbuf);
		reply.params.write.p_data = (void *)&sendbuf;
		update_sensor_data();
		do_wr = false;
	} else if (handle == m_time_timesync_tag_char_handles.value_handle) {
		// timesync_tag
		if (ar->request.write.len != sizeof(timesync_tag))
			do_wr = false;
		if (do_wr)
			memcpy(timesync_tag, ar->request.write.data, sizeof(timesync_tag));

		reply.params.write.len = sizeof(timesync_tag);
		reply.params.write.p_data = timesync_tag;
	} else if (handle == m_time_timesync_enable_char_handles.value_handle) {
		// timesync enable: 0 = off, 1 = rx, 2 = tx
		if (ar->request.write.len != sizeof(timesync_enable))
			do_wr = false;
		if (do_wr) {
			uint8_t en = *(uint8_t *)ar->request.write.data;
			if (en > 2)
				en = 2;

			if (en != timesync_enable) {
				ret_code_t err_code = NRF_SUCCESS;

				NRF_LOG_INFO("timesync enable: %u -> %u", timesync_enable, en);

				if (timesync_enable)
					timesync_disable();

				if (en > 1)
					err_code = timesync_enable_master();
				else if (en)
					err_code = timesync_enable_slave();

				APP_ERROR_CHECK(err_code);
				timesync_enable = en;
			}

			timebase_step = true;
		}

		reply.params.write.len = sizeof(timesync_enable);
		reply.params.write.p_data = (void *)&timesync_enable;
	} else if (handle == m_time_timesync_value_char_handles.value_handle) {
		reply.params.write.len = sizeof(timesync_val);
		reply.params.write.p_data = (void *)&timesync_val;
		do_wr = false;
	} else if (MAG_DATA_REG && handle == m_mags_data_char_handles.value_handle) {
		sensor_read_mag_data(&mag_data);
		reply.params.write.len = sizeof(mag_data);
		reply.params.write.p_data = (void *)&mag_data;
		do_wr = false;
	} else if (handle == m_bands_volt_char_handles.value_handle) {
		update_voltdata();
		reply.params.write.len = sizeof(volt_data);
		reply.params.write.p_data = (void *)&volt_data;
		do_wr = false;
	} else if (handle == m_bands_input_zero_char_handles.value_handle) {
		if (do_wr && ar->request.write.len != sizeof(sensor_zero))
			do_wr = false;
		if (do_wr) {
			memcpy(sensor_zero, ar->request.write.data, sizeof(sensor_zero));
			pstore_write(PSKEY_SENSOR_ZERO, &sensor_zero, sizeof(sensor_zero));
		}
	} else if (handle == m_bands_decimation_char_handles.value_handle) {
		if (do_wr && ar->request.write.len != sizeof(sensor_decimate))
			do_wr = false;
		if (do_wr) {
			memcpy(sensor_decimate, ar->request.write.data, sizeof(sensor_decimate));
			pstore_write(PSKEY_SENSOR_DECIMATE, &sensor_decimate, sizeof(sensor_decimate));
		}
	} else if (handle == m_mags_hard_char_handles.value_handle) {
		if (do_wr && ar->request.write.len != sizeof(sensor_magzero))
			do_wr = false;
		if (do_wr) {
			memcpy(sensor_magzero, ar->request.write.data, sizeof(sensor_magzero));
			pstore_write(PSKEY_SENSOR_MAGZERO, &sensor_magzero, sizeof(sensor_magzero));
		}
	} else if (handle == m_mags_soft_char_handles.value_handle) {
		if (do_wr && ar->request.write.len != sizeof(sensor_magcorr))
			do_wr = false;
		if (do_wr) {
			memcpy(sensor_magcorr, ar->request.write.data, sizeof(sensor_magcorr));
			pstore_write(PSKEY_SENSOR_MAGCORR, &sensor_magcorr, sizeof(sensor_magcorr));
		}
	} else if (handle == m_time_timebase_char_handles.value_handle) {
		uint32_t v;
		if (do_wr && ar->request.write.len != sizeof(timebase_freq))
			do_wr = false;
		if (do_wr) {
			memcpy(&v, ar->request.write.data, sizeof(v));
			if (v < 900000 || v > 1100000)
				do_wr = false;
		}
		if (do_wr)
			timebase_update_freq(v);
	} else {
		return;
	}

	NRF_LOG_DEBUG("BANDS %ca h=%u len=%u",
		ar->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE ? 'W' : 'R',
		handle, reply.params.write.len);

	reply.type = ar->type;
	if (!do_wr && ar->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
		reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
	else
		reply.params.write.update = true;

        sd_ble_gatts_rw_authorize_reply(gatts_ev->conn_handle, &reply);
}

static void bands_tx_trampoline(void *evdata, uint16_t evsize)
{
	send_notify();
}

void bands_trigger_send(void)
{
	app_sched_event_put(NULL, 0, &bands_tx_trampoline);
}

void timebase_freq_changed(void)
{
	reset_sensor_fifo();
	++timesync_val;
	sd_ble_gatts_hvx(m_conn_handle, &tsync_notify);
}

static void bands_send_magdata(void *evdata, uint16_t evsize)
{
	if (!do_md_notify)
		return;

	sensor_read_mag_data(&mag_data);
	sd_ble_gatts_hvx(m_conn_handle, &magdata_notify);
}

void bands_notify_magdata(void)
{
	if (do_md_notify)
		app_sched_event_put(NULL, 0, &bands_send_magdata);
}

void bands_send_voltdata(void)
{
	if (!do_volt_notify)
		return;

	update_voltdata();
	sd_ble_gatts_hvx(m_conn_handle, &volt_notify);
}

static void bands_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
	    on_rw_auth(p_ble_evt);
	    break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
	    send_notify();
            break;

        default:
            // No implementation needed.
            break;
    }
}

__attribute__((cold))
void bands_configure(void)
{
	pstore_read(PSKEY_SENSOR_ZERO, &sensor_zero, sizeof(sensor_zero));
	NRF_LOG_INFO("sensor zero: %d,%d,%d %d,%d,%d",
		sensor_zero[0], sensor_zero[1], sensor_zero[2],
		sensor_zero[3], sensor_zero[4], sensor_zero[5]);
	pstore_read(PSKEY_SENSOR_DECIMATE, &sensor_decimate, sizeof(sensor_decimate));
	NRF_LOG_INFO("pkt decimate: %u/%u", sensor_decimate[0], sensor_decimate[1]);

	if (!MAG_DATA_REG)
		return;

	pstore_read(PSKEY_SENSOR_MAGZERO, &sensor_magzero, sizeof(sensor_magzero));
	NRF_LOG_INFO("mag zero: %d,%d,%d",
		sensor_magzero[0], sensor_magzero[1], sensor_magzero[2]);

	#define MAGCORR_MAG(i) \
		((sensor_magcorr[i] & 0x78) >> 3)
	#define MAGCORR_VAL(i) \
		(sensor_magcorr[i] & 0x80 ? -MAGCORR_MAG(i) : MAGCORR_MAG(i)),  \
		(125 * (sensor_magcorr[i] & 0x07))
	sensor_magcorr[0] = sensor_magcorr[4] = sensor_magcorr[8] = 0x08;
	pstore_read(PSKEY_SENSOR_MAGCORR, &sensor_magcorr, sizeof(sensor_magcorr));
	NRF_LOG_INFO("mag corr Xi: %+d.%03u %+d.%03u %+d.%03u",
		MAGCORR_VAL(0), MAGCORR_VAL(1), MAGCORR_VAL(2));
	NRF_LOG_INFO("mag corr Yi: %+d.%03u %+d.%03u %+d.%03u",
		MAGCORR_VAL(3), MAGCORR_VAL(4), MAGCORR_VAL(5));
	NRF_LOG_INFO("mag corr Zi: %+d.%03u %+d.%03u %+d.%03u",
		MAGCORR_VAL(6), MAGCORR_VAL(7), MAGCORR_VAL(8));
}

#define BANDS_BLE_OBSERVER_PRIO  3
NRF_SDH_BLE_OBSERVER(m_bands_obs, BANDS_BLE_OBSERVER_PRIO, bands_on_ble_evt, NULL);

/**@brief Function for initializing Band Service.
 */
static void sensors_init(void)
{
    ret_code_t err_code;
    ble_uuid_t ble_uuid;

    // Add service

    BLE_UUID_BLE_ASSIGN(ble_uuid, SENSOR_SERVICE_UUID);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &m_bands_service_handle);
    APP_ERROR_CHECK(err_code);

    // Add inertial sensor characteristics

    err_code = characteristic_add(m_bands_service_handle,
                                  (void *)&sensor_char_params,
                                  &m_bands_input_char_handles);
    APP_ERROR_CHECK(err_code);

    input_notify.handle = m_bands_input_char_handles.value_handle;

    err_code = characteristic_add(m_bands_service_handle,
                                  (void *)&sensor_zero_char_params,
                                  &m_bands_input_zero_char_handles);
    APP_ERROR_CHECK(err_code);

    err_code = characteristic_add(m_bands_service_handle,
                                  (void *)&sensor_enable_char_params,
                                  &m_bands_enable_char_handles);
    APP_ERROR_CHECK(err_code);

    err_code = characteristic_add(m_bands_service_handle,
                                  (void *)&sensor_dec_char_params,
                                  &m_bands_decimation_char_handles);
    APP_ERROR_CHECK(err_code);

    // Add voltage-sense characteristics

    err_code = characteristic_add(m_bands_service_handle,
                                  (void *)&vsense_char_params,
                                  &m_bands_volt_char_handles);
    APP_ERROR_CHECK(err_code);
}

static void times_init(void)
{
    ret_code_t err_code;
    ble_uuid_t ble_uuid;

    // Add service for clock

    BLE_UUID_BLE_ASSIGN(ble_uuid, TIME_SERVICE_UUID);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &m_time_service_handle);
    APP_ERROR_CHECK(err_code);

    // Add time clock characteristics

    err_code = characteristic_add(m_time_service_handle,
                                  (void *)&timebase_char_params,
                                  &m_time_timebase_char_handles);
    APP_ERROR_CHECK(err_code);

    // Add time-sync characteristics

    err_code = characteristic_add(m_time_service_handle,
                                  (void *)&timesync_tag_char_params,
                                  &m_time_timesync_tag_char_handles);
    APP_ERROR_CHECK(err_code);

    err_code = characteristic_add(m_time_service_handle,
                                  (void *)&timesync_value_char_params,
                                  &m_time_timesync_value_char_handles);
    APP_ERROR_CHECK(err_code);

    tsync_notify.handle = m_time_timesync_value_char_handles.value_handle;

    err_code = characteristic_add(m_time_service_handle,
                                  (void *)&timesync_enable_char_params,
                                  &m_time_timesync_enable_char_handles);
    APP_ERROR_CHECK(err_code);
}

static void mags_init(void)
{
    ret_code_t err_code;
    ble_uuid_t ble_uuid;

    if (!MAG_DATA_REG)
	return;

    // Add service for magnetometer

    BLE_UUID_BLE_ASSIGN(ble_uuid, MAG_SERVICE_UUID);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &m_mags_service_handle);
    APP_ERROR_CHECK(err_code);

    // Add magnetometer characteristics

    err_code = characteristic_add(m_mags_service_handle,
				  (void *)&magdata_char_params,
				  &m_mags_data_char_handles);
    APP_ERROR_CHECK(err_code);
 
    if (!MAG_I2C_ADDR)
        return;

    magdata_notify.handle = m_mags_data_char_handles.value_handle;

    err_code = characteristic_add(m_mags_service_handle,
				  (void *)&maghard_char_params,
				  &m_mags_hard_char_handles);
    APP_ERROR_CHECK(err_code);

    err_code = characteristic_add(m_mags_service_handle,
				  (void *)&magsoft_char_params,
				  &m_mags_soft_char_handles);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing Haptic Feedback Service.
 */
static void haptics_init(void)
{
    ret_code_t err_code;
    ble_uuid_t ble_uuid;

    if (ACCEL_TEST || !MOTOR_I2C_ADDR)
	return;

    // Add service

    BLE_UUID_BLE_ASSIGN(ble_uuid, HAPTIC_SERVICE_UUID);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &m_haptics_service_handle);
    APP_ERROR_CHECK(err_code);

    // Add motor characteristic

    err_code = characteristic_add(m_haptics_service_handle,
                                  (void *)&haptic_char_params,
                                  &m_haptics_libplay_char_handles);
    APP_ERROR_CHECK(err_code);

    if (!MOTOR_DEVEL)
	return;

    err_code = characteristic_add(m_haptics_service_handle,
                                  (void *)&haptic_conf_char_params,
                                  &m_haptics_config_char_handles);
    APP_ERROR_CHECK(err_code);
}

__attribute__((cold))
void bands_init(void)
{
	sensors_init();
	times_init();
	haptics_init();
	mags_init();
}
