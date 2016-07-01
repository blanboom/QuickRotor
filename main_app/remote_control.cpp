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
#include "flight_control.h"

#define RX_PAYLOAD 4

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
		trace_printf("%x, %x, %c.%c", buf[0], buf[1], buf[2], buf[3]);
		// 设置油门或 YPR
		if(buf[0] != 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0) {
			// TODO: 判断数值是否超过范围
			switch(buf[1]) {
			case 0x01:
				velocity_set += 10;
				break;
			case 0x02:
				velocity_set -= 10;
				break;
			case 0x04:
				yaw_set += 1;
				break;
			case 0x08:
				yaw_set -= 1;
				break;
			case 0x10:
				pitch_set += 1;
				break;
			case 0x20:
				pitch_set -= 1;
				break;
			case 0x40:
				roll_set += 1;
				break;
			case 0x80:
				roll_set -= 1;
				break;
			default:
				break;
			}
		}
		// 设置 PID
		if(buf[0] == 0 && buf[1] != 0) {
			if(buf[3] >= '0' && buf[3] <= '9' && buf[4] >= '0' && buf[4] <= '9') {
				float result = (buf[3] - '0') + (float)(buf[4] - '0') / (float)10 ;
				switch(buf[0]) {
				case 'a':
					yaw_Kp = result;
					break;
				case 'b':
					yaw_Ki = result;
					break;
				case 'c':
					yaw_Kd = result;
					break;
				case 'd':
					roll_Kp = result;
					break;
				case 'e':
					roll_Ki = result;
					break;
				case 'f':
					roll_Kd = result;
					break;
				case 'g':
					pitch_Kp = result;
					break;
				case 'h':
					pitch_Ki = result;
					break;
				case 'i':
					pitch_Kd = result;
					break;
				default:
					break;
				}
			}
		}
	}
}
