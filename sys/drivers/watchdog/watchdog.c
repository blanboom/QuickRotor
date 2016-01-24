#include "stm32f10x.h"
#include "watchdog.h"

// Tout=((4×2^prer) ×rlr)/40 (ms)
// prer: 0~7
// rlr: 12bit
void Watchdog_Init(u8 prer,u16 rlr)
{
	IWDG->KR=0X5555;
  	IWDG->PR=prer;
  	IWDG->RLR=rlr;
	IWDG->KR=0XAAAA;
  	IWDG->KR=0XCCCC;
}
