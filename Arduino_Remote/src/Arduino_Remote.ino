/* 引脚：
 *     nRF24L01   ---   Arduino UNO
 *     VCC       <--->  3.3V
 *     GND       <--->  GND
 *     CE        <--->  D8
 *     CSN       <--->  D7
 *     MOSI      <--->  MOSI
 *     MISO      <--->  MISO
 *     SCK       <--->  SCK
 */

/* 按键：
 * 控制增减：
 *     q: 油门 +
 *     a: 油门 -
 *     w: yaw +
 *     s: yaw -
 *     e: roll +
 *     d: roll -
 *     r: pitch +
 *     f: pitch -
 * 设置 PID:
 *     z: Yaw Kp
 *     x: Yaw Ki
 *     c: Yaw Kd
 *     v: Roll Kp
 *     b: Roll Ki
 *     n: Roll Kd
 *     m: Pitch Kp
 *     ,: Pitch Ki
 *     .: Pitch Kd
 *     后面跟上 PID 数值，整数与小数部分各一位，例如 5.7, 以 $ 结束
 */

#include <Mirf.h>
#include <MirfHardwareSpiDriver.h>
#include <SPI.h>
#include <nRF24L01.h>

uint8_t data[4];

void setup()
{
    Serial.begin(115200);
    Serial.print("Init...");

    /* 初始化 nRF24L01 */
    pinMode(7, OUTPUT);
    digitalWrite(7, 1);
    Mirf.cePin  = 8; // 设置 CE 引脚
    Mirf.csnPin = 7; // 设置 CSN 引脚
    Mirf.spi    = &MirfHardwareSpi;
    Mirf.init();
    Mirf.setRADDR((byte *)"63472");
    Mirf.payload = 4 * sizeof(uint8_t); // 一次发送四字节
    Mirf.channel = 57; // 发送通道，可以填 0~128，收发必须一致。
    Mirf.config();

    Serial.println("Init OK!");
}

void loop()
{
    if (Serial.available() > 0) {
        char tmp = Serial.read();
        char tmp2;
        if (tmp == 'q' || tmp == 'a' || tmp == 'w' || tmp == 's' || tmp == 'e' ||
            tmp == 'd' || tmp == 'r' || tmp == 'f') {
            data[1] = 0;
            data[2] = 0;
            data[3] = 0;
            switch (tmp) {
            case 'q': data[0] = 0x01; break;
            case 'a': data[0] = 0x02; break;
            case 'w': data[0] = 0x04; break;
            case 's': data[0] = 0x08; break;
            case 'e': data[0] = 0x10; break;
            case 'd': data[0] = 0x20; break;
            case 'r': data[0] = 0x40; break;
            case 'f': data[0] = 0x80; break;
            default: data[0] = 0; break;
            }
            sendData();
        }
        if (tmp == 'z' || tmp == 'x' || tmp == 'c' || tmp == 'v' || tmp == 'b' ||
            tmp == 'n' || tmp == 'm' || tmp == ',' || tmp == '.') {
            data[0] = 0;
            while (Serial.available() == 0) ;
            data[2] = Serial.read(); if (data[2] < '0' || data[2] > '9') return;
            while (Serial.available() == 0) ;
            tmp2 = Serial.read(); if (tmp2 != '.') return;
            while (Serial.available() == 0) ;
            data[3] = Serial.read(); if (data[3] < '0' || data[3] > '9') return;
            while (Serial.available() == 0) ;
            tmp2 = Serial.read(); if (tmp2 != tmp) return;
            switch (tmp) {
            case 'z': data[1] = 'a'; break;
            case 'x': data[1] = 'b'; break;
            case 'c': data[1] = 'c'; break;
            case 'v': data[1] = 'd'; break;
            case 'b': data[1] = 'e'; break;
            case 'n': data[1] = 'f'; break;
            case 'm': data[1] = 'g'; break;
            case ',': data[1] = 'h'; break;
            case '.': data[1] = 'i'; break;
            default: break;
            }
            sendData();
        }
    }
}

void sendData()
{
    Serial.print("Sending");
    Mirf.setTADDR((byte *)"deytr"); // 设置发送地址
    Mirf.send(data);
    while (Mirf.isSending()) {
        Serial.print('.');
    }
    Serial.println("Sent!");
}
