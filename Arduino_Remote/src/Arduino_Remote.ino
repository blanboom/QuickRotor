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

#include <Mirf.h>
#include <MirfHardwareSpiDriver.h>
#include <SPI.h>
#include <nRF24L01.h>

void setup() {
  Serial.begin(115200);
  Serial.print("Init...");

  /* 初始化 nRF24L01 */
  pinMode(7, OUTPUT);
  digitalWrite(7, 1);
  Mirf.cePin = 8;  // 设置 CE 引脚
  Mirf.csnPin = 7; // 设置 CSN 引脚
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"63472");
  Mirf.payload = 8 * sizeof(uint8_t); // 一次发送四字节
  Mirf.channel = 57; // 发送通道，可以填 0~128，收发必须一致。
  Mirf.config();

  Serial.println("Init OK!");
}

void loop() {
  /* 发送数据 */
  uint8_t data[] = {0x18, 0x25, 0x36, 0x74, 0x18, 0x25, 0x36, 0x74};
  Serial.print("Sending");
  Mirf.setTADDR((byte *)"deytr"); // 设置发送地址
  Mirf.send(data);
  while (Mirf.isSending()) {
      Serial.print('.');
  }

  Serial.println("Sent!");

  delay(1000);
}
