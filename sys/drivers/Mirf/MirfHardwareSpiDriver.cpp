#include "MirfHardwareSpiDriver.h"

#define SPI_PORT      SPI1
#define SPI_SCK_PIN   GPIO_Pin_5     // PA5
#define SPI_MISO_PIN  GPIO_Pin_6     // PA6
#define SPI_MOSI_PIN  GPIO_Pin_7     // PA7
#define SPI_CS_PIN    GPIO_Pin_4     // PA4
#define SPI_GPIO_PORT GPIOA


uint8_t MirfHardwareSpiDriver::transfer(uint8_t data){
//	return SPI.transfer(data);

	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_TXE) == RESET); // Wait while DR register is not empty
	SPI_I2S_SendData(SPI_PORT,data); // Send byte to SPI
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_RXNE) == RESET); // Wait to receive byte
	return SPI_I2S_ReceiveData(SPI_PORT); // Read byte from SPI bus
}

void MirfHardwareSpiDriver::begin(){
//	SPI.begin();
//	SPI.setDataMode(SPI_MODE0);
//	SPI.setClockDivider(SPI_2XCLOCK_MASK);

	// Clock Init
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA,ENABLE);


	// IO Init
	GPIO_InitTypeDef PORT;
	// Configure SPI pins
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Pin = SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN;
	PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_GPIO_PORT,&PORT);


	// SPI Init
	SPI_InitTypeDef SPI;
	SPI.SPI_Mode = SPI_Mode_Master;
	SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI.SPI_CPOL = SPI_CPOL_Low;
	SPI.SPI_CPHA = SPI_CPHA_1Edge;
	SPI.SPI_CRCPolynomial = 7;
	SPI.SPI_DataSize = SPI_DataSize_8b;
	SPI.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI_PORT,&SPI);

	// NSS must be set to '1' due to NSS_Soft settings (otherwise it will be Multimaster mode).
	SPI_NSSInternalSoftwareConfig(SPI_PORT,SPI_NSSInternalSoft_Set);
	SPI_Cmd(SPI_PORT,ENABLE);
}

void MirfHardwareSpiDriver::end(){
}

MirfHardwareSpiDriver MirfHardwareSpi;
