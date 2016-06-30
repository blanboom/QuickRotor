#include "MirfSpiDriver.h"
#include <stdint.h>
#include "stm32f10x.h"

#ifndef __MIRF_HARDWARE_SPI_DRIVER
#define __MIRF_HARDWARE_SPI_DRIVER 


class MirfHardwareSpiDriver : public MirfSpiDriver {

	public: 
		virtual uint8_t transfer(uint8_t data);
		virtual void begin();
		virtual void end();
};

extern MirfHardwareSpiDriver MirfHardwareSpi;

#endif
