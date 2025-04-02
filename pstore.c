/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 *
 * Author: Michał Mirosław
 */

#include "app_error.h"
#include "app_scheduler.h"
#include "fds.h"
#include "nrf_log.h"

#include "pstore.h"

#define DEBUG 1
#define MAX_RECORD_SIZE 16

/* Flag to check fds initialization. */
static volatile bool m_fds_initialized;
static volatile bool m_fds_writing;
static volatile unsigned m_fds_gc;
static volatile unsigned m_fds_ops;

void idle_state_handle(void);

#if DEBUG

/* Array to map FDS return values to strings. */
char const * fds_err_str[] =
{
    "FDS_SUCCESS",
    "FDS_ERR_OPERATION_TIMEOUT",
    "FDS_ERR_NOT_INITIALIZED",
    "FDS_ERR_UNALIGNED_ADDR",
    "FDS_ERR_INVALID_ARG",
    "FDS_ERR_NULL_ARG",
    "FDS_ERR_NO_OPEN_RECORDS",
    "FDS_ERR_NO_SPACE_IN_FLASH",
    "FDS_ERR_NO_SPACE_IN_QUEUES",
    "FDS_ERR_RECORD_TOO_LARGE",
    "FDS_ERR_NOT_FOUND",
    "FDS_ERR_NO_PAGES",
    "FDS_ERR_USER_LIMIT_REACHED",
    "FDS_ERR_CRC_CHECK_FAILED",
    "FDS_ERR_BUSY",
    "FDS_ERR_INTERNAL",
};

/* Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};
#endif

static bool pstore_find_valid(uint32_t key, size_t len, fds_record_desc_t *desc)
{
	fds_flash_record_t flash_rec;
	fds_record_desc_t read_desc;
	fds_find_token_t ftok;
	ret_code_t err_code;
	uint16_t file_id = key >> 16;
	uint16_t key_id = key & 0xFFFF;
	uint32_t record_id = 0;
	unsigned count = 0;
	bool ok = false;

	memset(&ftok, 0, sizeof(ftok));
	len = (len + 3) / 4;

	while (fds_record_find(file_id, key_id, &read_desc, &ftok) == FDS_SUCCESS) {
		++count;
		if (fds_record_open(&read_desc, &flash_rec) != FDS_SUCCESS)
			continue;

		if (flash_rec.p_header->length_words == len) {
			if (!ok || record_id < flash_rec.p_header->record_id) {
				ok = true;
				record_id = flash_rec.p_header->record_id;
				memcpy(desc, &read_desc, sizeof(*desc));
			}
		}

		err_code = fds_record_close(&read_desc);
		if (DEBUG)
			APP_ERROR_CHECK(err_code);
	}

	if (count > ok) {
		memset(&ftok, 0, sizeof(ftok));
		while (fds_record_find(file_id, key_id, &read_desc, &ftok) == FDS_SUCCESS) {
			if (ok && read_desc.record_id == record_id)
				continue;

			err_code = fds_record_delete(&read_desc);
			if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
				break;
			if (DEBUG)
				APP_ERROR_CHECK(err_code);
		}
	}

	NRF_LOG_INFO("pstore_find_valid: %08X for %u: %u/%u; id 0x%X", key, len, ok, count, record_id);

	return count > 0;
}

static bool pstore_wait(ret_code_t err_code, unsigned ops)
{
	if (err_code == FDS_ERR_BUSY)
		return true;
	if (err_code != FDS_ERR_NO_SPACE_IN_QUEUES)
		return false;

	while (ops == m_fds_ops)
		idle_state_handle();

	return true;
}

static void pstore_gc(void)
{
	ret_code_t err_code;
	unsigned ops, gc;

	NRF_LOG_INFO("pstore: need gc");

	gc = m_fds_gc;
	for (;;) {
		ops = m_fds_ops;
		err_code = fds_gc();
		if (!pstore_wait(err_code, ops)) {
			APP_ERROR_CHECK(err_code);
			break;
		}
	}

	while (gc == m_fds_gc)
		idle_state_handle();
}

static ret_code_t pstore_delete_desc(fds_record_desc_t *del_desc)
{
	ret_code_t err_code;

	for (;;) {
		unsigned ops = m_fds_ops;

		err_code = fds_record_delete(del_desc);
		if (!pstore_wait(err_code, ops))
			break;
	}

	return err_code;
}

void pstore_write(uint32_t key, const void *data, size_t len)
{
	uint32_t write_buf[(MAX_RECORD_SIZE + 3) / 4];
	uint16_t file_id = key >> 16;
	uint16_t key_id = key & 0xFFFF;
	fds_record_desc_t write_desc;
	fds_record_t write_rec;
	ret_code_t err_code;
	bool write_update;
	bool did_gc;

	NRF_LOG_INFO("pstore_write: %04X.%04X for %u", file_id, key_id, len);

	if (len > MAX_RECORD_SIZE)
		APP_ERROR_CHECK(NRF_ERROR_INVALID_LENGTH);
	if (file_id >= 0xC000 || key_id == 0 || key_id >= 0xC000)
		APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);

	write_update = pstore_find_valid(key, len, &write_desc);

	memcpy(write_buf, data, len);

	write_rec.file_id = file_id;
	write_rec.key = key_id;
	write_rec.data.p_data = &write_buf;
	write_rec.data.length_words = (len + 3) / 4;

	did_gc = false;
	m_fds_writing = true;
	for (;;) {
		unsigned ops = m_fds_ops;

		if (write_update)
			err_code = fds_record_update(&write_desc, &write_rec);
		else
			err_code = fds_record_write(NULL, &write_rec);

		if (pstore_wait(err_code, ops))
			continue;
		if (err_code == FDS_ERR_NO_SPACE_IN_FLASH) {
			if (did_gc) {
				if (write_update) {
					pstore_delete_desc(&write_desc);
					write_update = false;
				} else {
					APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
				}
			}
			pstore_gc();
			did_gc = true;
			continue;
		}

		while (m_fds_writing)
			idle_state_handle();
		return;
	}
}

bool pstore_read(uint32_t key, void *data, size_t len)
{
	fds_flash_record_t flash_rec;
	fds_record_desc_t read_desc;
	ret_code_t err_code;

	NRF_LOG_INFO("pstore_read: %08X for %u", key, len);

	if (!pstore_find_valid(key, len, &read_desc))
		return false;

	if (fds_record_open(&read_desc, &flash_rec) != FDS_SUCCESS) {
		err_code = fds_record_delete(&read_desc);
		NRF_LOG_INFO("pstore_read failed: %x", err_code);
		if (DEBUG)
			APP_ERROR_CHECK(err_code);
		return false;
	}

	memcpy(data, flash_rec.p_data, len);

	return true;
}

static void fds_evt_handler(fds_evt_t const * p_evt)
{
#if DEBUG
	NRF_LOG_INFO("Event: %s received (%s)",
				 fds_evt_str[p_evt->id],
				 fds_err_str[p_evt->result]);
#else
	NRF_LOG_INFO("Event: %d received (%d)", p_evt->id, p_evt->result);
#endif

	++m_fds_ops;

	switch (p_evt->id)
	{
		case FDS_EVT_INIT:
			if (p_evt->result == FDS_SUCCESS)
				m_fds_initialized = true;
			break;

		case FDS_EVT_WRITE:
		case FDS_EVT_UPDATE:
			if (p_evt->result == FDS_SUCCESS)
			{
				NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
				NRF_LOG_INFO("File ID:\t0x%04x",	p_evt->write.file_id);
				NRF_LOG_INFO("Record ID:\t0x%08x",  p_evt->write.record_id);
			}

			if (p_evt->write.file_id < 0xC000)
				m_fds_writing = false;
		break;

		case FDS_EVT_DEL_RECORD:
			if (p_evt->result == FDS_SUCCESS)
			{
				NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
				NRF_LOG_INFO("File ID:\t0x%04x",	p_evt->del.file_id);
				NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
			}
		break;

		case FDS_EVT_GC:
			++m_fds_gc;

		default:
			break;
	}
}

__attribute__((cold))
void pstore_init(void)
{
	ret_code_t err_code;

	err_code = fds_register(fds_evt_handler);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("Initializing pstore");
	err_code = fds_init();
	APP_ERROR_CHECK(err_code);
}

bool pstore_initialized(void)
{
	return m_fds_initialized;
}
