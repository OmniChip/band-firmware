/**
 * Copyright (c) 2019, OmniChip sp. z o.o.
 */

#include "boards.h"
#ifdef BOARD_PCA10040
#include "board-band-devb.h"
#endif

extern volatile uint16_t vbat_mv, vbus_mv, vdd_mv;

extern void bands_send_voltdata(void);

extern void vsense_init(void);
extern void vsense_start_timer(void);
