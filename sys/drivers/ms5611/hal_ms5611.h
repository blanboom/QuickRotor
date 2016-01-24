// TODO: 注明来源

#ifndef _HAL_MS5611_H_
#define _HAL_MS5611_H_

#include <drivers/mpu9250/mpu9250.h>
#include "stm32f10x.h"
#include "drivers/timer/timer.h"

#define MS5611_I2C                  I2C2
#define MS5611_I2C_RCC_Periph       RCC_APB1Periph_I2C2
#define MS5611_I2C_Port             GPIOB
#define MS5611_I2C_SCL_Pin          GPIO_Pin_10
#define MS5611_I2C_SDA_Pin          GPIO_Pin_11
#define MS5611_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define MS5611_I2C_Speed            100000 //100000

void MS5611_I2C_Init(void);
void MS5611_I2C_ByteWrite(u8, u8, u8);
void MS5611_I2C_CommandWrite(u8, u8);
void MS5611_I2C_BufferRead(u8, u8, u16, u8*);
char MS5611_I2C_ByteRead(u8, u8);
inline void MS5611_Delay(u32);


inline void __attribute__((always_inline))
MS5611_Delay(u32 ms)
{
	timer_sleep(ms);
}

#endif
