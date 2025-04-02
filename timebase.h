/**
 * Copyright (c) 2019-2020, OmniChip sp. z o.o.
 */
#ifndef TIMEBASE_H_
#define TIMEBASE_H_

#include "nrfx_timer.h"

extern uint32_t timebase_freq;
extern bool timebase_step;

void timebase_reset(void);
void timebase_update_freq(uint32_t freq);
void timebase_update(uint32_t ts, uint32_t ref_ts, uint32_t prev_ref_offs);
void timebase_freq_changed(void);

static const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(3);

static inline uint32_t timebase_capture_task(void)
{
	return nrfx_timer_task_address_get(&m_timer, NRF_TIMER_TASK_CAPTURE0);
}

static inline uint32_t timebase2_capture_task(void)
{
	return nrfx_timer_task_address_get(&m_timer, NRF_TIMER_TASK_CAPTURE1);
}

static inline uint32_t timesync_capture_task(void)
{
	return nrfx_timer_task_address_get(&m_timer, NRF_TIMER_TASK_CAPTURE5);
}

static inline uint32_t timebase_captured_ts(void)
{
	return nrfx_timer_capture_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
}

static inline uint32_t timebase2_captured_ts(void)
{
	return nrfx_timer_capture_get(&m_timer, NRF_TIMER_CC_CHANNEL1);
}

static inline uint32_t timesync_captured_ts(void)
{
	return nrfx_timer_capture_get(&m_timer, NRF_TIMER_CC_CHANNEL5);
}

uint32_t timebase_adjusted_us(uint32_t ts);

#endif /* TIMEBASE_H_ */
