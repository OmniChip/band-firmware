/*
 * SPDX-License-Identifier: GPL-3.0-or-later	// TBD
 *
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 *
 * Author: Michał Mirosław
 */
/**
 * Portions Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
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

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "ble.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "sensorsim.h"
#include "app_timer.h"
#include "nrf_log.h"

#include "boards.h"
#ifdef BOARD_PCA10040
#include "board-band-devb.h"
#endif
#include "fw-version.h"

#define UICR_BOARD_SN (*(volatile uint32_t *)0x100010FC)

#ifndef HW_VERSION
#define HW_VERSION ""
#endif

#if ACCEL_TEST
#define BASE_SW_VERSION "b"
#else
#define BASE_SW_VERSION "a"
#endif

#if DEBUG
#define DEBUG_SW_VERSION "d"
#else
#define DEBUG_SW_VERSION
#endif

#define SW_VERSION (BASE_SW_VERSION DEBUG_SW_VERSION)

#ifndef DEVICE_NAME
#define DEVICE_NAME                         "Band_On_DEVB"                             /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#endif

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(20000)                     /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                   85                                         /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   95                                         /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT             1                                          /**< Increment between each simulated battery level measurement. */

#define PNP_ID_VENDOR_ID_SOURCE             0x02                                       /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                    0x1915                                     /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID                   0xEEEE                                     /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION              0x0001                                     /**< Product Version. */


APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */
BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */

static sensorsim_cfg_t   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                       /**< Battery Level sensor simulator state. */


/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_FORBIDDEN) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Device Information Service.
 */

static const ble_dis_pnp_id_t dis_pnp_id = {
    .vendor_id_source = PNP_ID_VENDOR_ID_SOURCE,
    .vendor_id        = PNP_ID_VENDOR_ID,
    .product_id       = PNP_ID_PRODUCT_ID,
    .product_version  = PNP_ID_PRODUCT_VERSION,
};

static const ble_dis_init_t dis_init_obj = {
    .manufact_name_str.length = sizeof(MANUFACTURER_NAME)-1,
    .manufact_name_str.p_str = (uint8_t *)MANUFACTURER_NAME,
    .serial_num_str.length = 8,
    .serial_num_str.p_str = NULL /* runtime */,
    .hw_rev_str.length = sizeof(HW_VERSION)-1,
    .hw_rev_str.p_str = (uint8_t *)HW_VERSION,
    .fw_rev_str.length = sizeof(FW_VERSION)-1,
    .fw_rev_str.p_str = (uint8_t *)FW_VERSION,
    .sw_rev_str.length = sizeof(SW_VERSION)-1,
    .sw_rev_str.p_str = (uint8_t *)SW_VERSION,
    .p_pnp_id = (ble_dis_pnp_id_t *)&dis_pnp_id,

    .dis_char_rd_sec = SEC_OPEN,
};

static void dis_init(void)
{
    ble_dis_init_t   init_obj;
    ret_code_t       err_code;
    char             sn[10];

    memcpy(&init_obj, &dis_init_obj, sizeof(init_obj));
    init_obj.serial_num_str.p_str = (uint8_t *)sn;
    sprintf(sn, "%08lX", UICR_BOARD_SN);

    err_code = ble_dis_init(&init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Battery Service.
 */

static const ble_bas_init_t bas_init_obj = {
    .evt_handler          = NULL,
    .support_notification = true,
    .p_report_ref         = NULL,
    .initial_batt_level   = 100,

    .bl_rd_sec        = SEC_OPEN,
    .bl_cccd_wr_sec   = SEC_OPEN,
    .bl_report_rd_sec = SEC_OPEN,
};

static void bas_init(void)
{
    ret_code_t     err_code;

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the battery sensor simulator.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}


/**@brief Function for starting timers.
 */
static void timers_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
__attribute__((cold))
void basics_init(void)
{
    timers_init();
    dis_init();
    bas_init();
    sensor_simulator_init();
}

__attribute__((cold))
void basics_start(void)
{
    timers_start();
}
