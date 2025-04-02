/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 *
 * Author: Michał Mirosław
 */

extern void pstore_write(uint32_t key, const void *data, size_t len);
extern bool pstore_read(uint32_t key, void *data, size_t len);

extern void pstore_init(void);
extern bool pstore_initialized(void);
