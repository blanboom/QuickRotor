/*
 * This file is part of the libemb project.
 *
 * Copyright (C) 2011 Stefan Wendler <sw@kaltpost.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "nrf24l01_hw.h"
#include "stm32f10x.h"

#define NRF24L01_SPI		SPI1
#define NRF24L01_PORT		GPIOA
#define NRF24L01_CS			GPIO_Pin_4
#define NRF24L01_MISO		GPIO_Pin_6
#define NRF24L01_MOSI		GPIO_Pin_7
#define NRF24L01_SCK		GPIO_Pin_5

#define NRF24L01_CE_PORT	GPIOC
#define NRF24L01_CE			GPIO_Pin_5

#define SPI_CS_HIGH		GPIO_SetBits(NRF24L01_PORT, NRF24L01_CS)
#define SPI_CS_LOW		GPIO_ResetBits(NRF24L01_PORT, NRF24L01_CS)
#define CE_HIGH		GPIO_SetBits(NRF24L01_CE_PORT, NRF24L01_CE)
#define CE_LOW		GPIO_ResetBits(NRF24L01_CE_PORT, NRF24L01_CE)

void nrf_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 |
	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = NRF24L01_MISO | NRF24L01_MOSI | NRF24L01_SCK;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(NRF24L01_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = NRF24L01_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(NRF24L01_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = NRF24L01_CE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStructure);

	CE_HIGH;

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 84000kHz/256=328kHz < 400kHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(NRF24L01_SPI, &SPI_InitStructure);
	SPI_CalculateCRC(NRF24L01_SPI, DISABLE);
	SPI_Cmd(NRF24L01_SPI, ENABLE);

    SPI_CS_HIGH;
}

void nrf_spi_csh(void)
{
     SPI_CS_HIGH;
}

void nrf_spi_csl(void)
{
     SPI_CS_LOW;
}

unsigned char nrf_spi_xfer_byte(unsigned char data) {
	while (SPI_I2S_GetFlagStatus(NRF24L01_SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(NRF24L01_SPI, data);
	while (SPI_I2S_GetFlagStatus(NRF24L01_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(NRF24L01_SPI);
}

