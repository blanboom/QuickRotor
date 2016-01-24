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
 * 		 * 字节 1：
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
 * 		 * 字节 2: PID 参数 小数部分 （ASCII 字符 0 ~ 9）
 */

#include "drivers/nrf24l01/nrf24l01.h"
#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "remote_control.h"

static nrf_reg_buf addr;

void RC_Init(void) {
	nRF24_init();
	nRF24_RXMode(RC_DATA_SIZE);
}

void RC_Receive(void) {
	if (nRF24_DataReady()) {
		nRF24_RXPacket(tmp, 9);
		for (int i = 0; i < 9; i++) {
			trace_printf("%c", tmp[i]);
		}
		trace_printf("\n");
	}
}
