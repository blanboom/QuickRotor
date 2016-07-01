#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "drivers/timer/timer.h"
#include "drivers/watchdog/watchdog.h"
#include "drivers/nrf24l01/nrf24l01.h"
#include "remote_control.h"
#include "flight_control.h"

// ----- main() ---------------------------------------------------------------
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int main(int argc, char* argv[]){
	timer_ticks_t currentTime, currentTime_RC;

	trace_printf("System clock: %uHz\n", SystemCoreClock);
	timer_start();

//	timer_sleep(5000);

	initIMU();

	Watchdog_Init(5, 500); // 1600ms
	currentTime = 0;
	currentTime_RC = 0;
	for (;;) {
		if (systemTime - currentTime > 10) {
			currentTime = systemTime;
			updateYPR();
			computePID();
			calculateVelocities();
			updateMotors();
			Watchdog_Feed();
		}

		if (systemTime - currentTime_RC > 100) {
			currentTime_RC = systemTime;
			RC_Receive();
			Watchdog_Feed();
		}
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
