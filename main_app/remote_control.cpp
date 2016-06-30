/*****************************************
 * 接收数据格式：
 * 		 * 一共 4 个字节
 * 		 * 字节 0:
 * 		 	* 最 0 位：油门 +
 * 		 	* 第 1 位: 油门 -
 * 		 	* 第 2 位：yaw +
 * 		 	* 第 3 位: yaw -
 * 		 	* 第 4 位: roll +
 * 		 	* 第 5 位: roll-
 * 		 	* 第 6 位: pitch +
 * 		 	* 第 7 位: pitch -
 * 		 * 字节 1：（确定设置哪个值）
 * 		 	* a: Yaw Kp
 * 		 	* b: Yaw Ki
 * 		 	* c: Yaw Kd
 * 		 	* d: Roll Kp
 * 		 	* e: Roll Ki
 * 		 	* f: Roll Kd
 * 		 	* g: Pitch Kp
 * 		 	* h: Pitch Ki
 * 		 	* i: Pitch Kd
 * 		 * 字节 2: PID 参数 整数部分 （ASCII 字符 0 ~ 9）
 * 		 * 字节 3: PID 参数 小数部分 （ASCII 字符 0 ~ 9）
 */

extern "C"{
#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "remote_control.h"
}
#include "drivers/Mirf/Mirf.h"
#include "drivers/Mirf/MirfHardwareSpiDriver.h"


#define RX_PAYLOAD 8

uint8_t buf[RX_PAYLOAD];

void RC_Init(void) {
//	nRF24_init();
//	trace_printf("nRF24L01 Check(0: Online): %d\n", nRF24_Check());
//	nRF24_RXMode(RX_PAYLOAD);
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((uint8_t *)"deytr");
  Mirf.payload = RX_PAYLOAD * sizeof(uint8_t); // 一次发送四字节
  Mirf.channel = 57; // 发送通道，可以填 0~128，收发必须一致。
  Mirf.config();
}

void RC_Receive(void) {
	if(Mirf.dataReady()) {
		Mirf.getData(buf);
		trace_printf("%x, ", buf[0]);
		trace_printf("%x, ", buf[1]);
		trace_printf("%x, ", buf[2]);
		trace_printf("%x, ", buf[3]);
		trace_printf("%x, ", buf[4]);
		trace_printf("%x, ", buf[5]);
		trace_printf("%x, ", buf[6]);
		trace_printf("%x\n", buf[7]);
	}
}
