// TODO: 注明来源

#ifndef _HAL_MPU9250_H_
#define _HAL_MPU9250_H_

#include <drivers/mpu9250/mpu9250.h>
#include "stm32f10x.h"
#include "drivers/timer/timer.h"
#include <stdbool.h>

#define MPU9250_I2C                  I2C2
#define MPU9250_I2C_RCC_Periph       RCC_APB1Periph_I2C2
#define MPU9250_I2C_Port             GPIOB
#define MPU9250_I2C_SCL_Pin          GPIO_Pin_10
#define MPU9250_I2C_SDA_Pin          GPIO_Pin_11
#define MPU9250_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define MPU9250_I2C_Speed            100000 //100000
#define MPU9250_INT_PIN              GPIO_Pin_0
#define MPU9250_INT_PORT             GPIOB

// Using the MSENSR-9250 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
//mbed uses the eight-bit device address, so shift seven-bit addresses left by one!
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68<<1  // Device address when ADO = 0
#endif

void MPU9250_I2C_Init(void);
void MPU9250_I2C_ByteWrite(u8, u8, u8);
void MPU9250_I2C_BufferRead(u8, u8, u16, u8*);
char MPU9250_I2C_ByteRead(u8, u8);
inline void MPU9250_Delay(u32);
inline bool MPU9250_GetIntStatus(void);


inline void __attribute__((always_inline))
MPU9250_Delay(u32 ms)
{
	timer_sleep(ms);
}

inline bool __attribute__((always_inline))
MPU9250_GetIntStatus(void)
{
	return MPU9250_INT_PORT->IDR & MPU9250_INT_PIN;
}

#endif
