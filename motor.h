/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 *
 * Author: Michał Mirosław
 */

#include "boards.h"
#ifdef BOARD_PCA10040
#include "board-band-devb.h"
#endif

#ifndef MOTOR_I2C_ADDR
#define MOTOR_I2C_ADDR 0
#endif

extern void motor_init(void);
extern void motor_play_lib_effect(const uint8_t *seq, size_t len);
extern bool motor_reconfigure(uint32_t conf);

extern uint32_t motor_conf;
