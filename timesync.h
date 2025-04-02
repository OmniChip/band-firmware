/**
 * Copyright (c) 2020, OmniChip sp. z o.o.
 */
#ifndef TIMESYNC_H_
#define TIMESYNC_H_

#include <stdint.h>

void timesync_init(void);
uint32_t timesync_enable_master(void);
uint32_t timesync_enable_slave(void);
void timesync_disable(void);

#endif /* TIMESYNC_H_ */
