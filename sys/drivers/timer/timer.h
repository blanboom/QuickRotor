//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu.
//

#ifndef TIMER_H_
#define TIMER_H_

#include "cmsis_device.h"

// ----------------------------------------------------------------------------

#define TIMER_FREQUENCY_HZ (1000u)

typedef uint32_t timer_ticks_t;

extern volatile timer_ticks_t systemTime;

extern volatile timer_ticks_t timer_delayCount;

extern void
timer_start (void);

extern void
timer_sleep (timer_ticks_t ticks);

inline u32 timer_get_us(void);


inline u32 __attribute__((always_inline))
timer_get_us(void) {
	return systemTime * 1000 + (SysTick -> VAL) / 72;
}

// ----------------------------------------------------------------------------

#endif // TIMER_H_
