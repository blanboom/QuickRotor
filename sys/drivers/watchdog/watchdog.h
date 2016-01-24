#ifndef DRIVERS_WATCHDOG_WATCHDOG_H_
#define DRIVERS_WATCHDOG_WATCHDOG_H_

void Watchdog_Init(u8 prer,u16 rlr);
inline void Watchdog_Feed(void);

inline void __attribute__((always_inline))
Watchdog_Feed(void)
{
	IWDG->KR=0XAAAA;  // reload
}

#endif /* DRIVERS_WATCHDOG_WATCHDOG_H_ */
