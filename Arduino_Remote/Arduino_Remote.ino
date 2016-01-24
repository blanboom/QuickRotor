#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>


/* 暂存 ADC 读取值 */
uint16_t adata = 0;
uint16_t datatmp[9];
uint8_t data[9];

void setup() {
  Serial.begin(115200);
  Serial.print("Init...");

  /* 初始化 nRF24L01 */
  //pinMode(7, OUTPUT);
  //digitalWrite(7, 1);
  Mirf.cePin = 8;     // 设置 CE 引脚
  Mirf.csnPin = 7;    // 设置 CSN 引脚
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"63472"); // 设置接收标识符"Sen87"
  Mirf.payload = 9 * sizeof(uint8_t);  // 设置一次收发的字节数, 9 个传感器，9 字节
  Mirf.channel = 0x12;   // 发送通道，可以填0~128，收发必须一致。
  Mirf.config();

  Serial.println("OK!");

  delay(500);

  byte reg = 0;
  uint8_t reg1[6];

  Mirf.readRegister( RX_ADDR_P1, reg1, 5 );
  Serial.print( "nRF24_REG_RX_ADDR_P1 = " );
  reg1[5] = 0;
  Serial.println((char*)reg1);


  Mirf.readRegister( EN_AA, &reg, sizeof(reg) );
  Serial.print( "EN_AA = " );
  Serial.println( reg, HEX );


  Mirf.readRegister( EN_RXADDR, &reg, sizeof(reg) );
  Serial.print( "EN_RXADDR = " );
  Serial.println( reg, HEX );

  Mirf.readRegister( RF_CH, &reg, sizeof(reg) );
  Serial.print( "RF_CH = " );
  Serial.println( reg, HEX );

  Mirf.readRegister( RX_PW_P0, &reg, sizeof(reg) );
  Serial.print( "RX_PW_P0 = " );
  Serial.println( reg, HEX );  

  Mirf.readRegister( RX_PW_P1, &reg, sizeof(reg) );
  Serial.print( "RX_PW_P1 = " );
  Serial.println( reg, HEX );  

  Mirf.readRegister( RF_SETUP, &reg, sizeof(reg) );
  Serial.print( "RF_SETUP = " );
  Serial.println( reg, HEX );  

  Mirf.readRegister( CONFIG, &reg, sizeof(reg) );
  Serial.print( "CONFIG = " );
  Serial.println( reg, HEX );    
}

void loop() {
  uint8_t i;

  for(i = 0; i < 9; i++) {
    data[i] = 'a' + i;
  }

  /* 发送数据 */
  Mirf.setTADDR((byte *)"63472"); // 设置发送地址
  Mirf.send(data);
  while(Mirf.isSending()) {}

  for(i = 0; i < 9; i++) {
    Serial.print((char)data[i]);
  }
  Serial.println(" Sent!");
}
