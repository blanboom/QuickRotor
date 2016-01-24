#ifndef DRIVERS_MOTOR_MOTOR_H_
#define DRIVERS_MOTOR_MOTOR_H_

#include "stm32f10x.h"

#define MOTOR_PORT      GPIOB
#define MOTOR_TIM       TIM4
#define MOTOR_TIM_RCC   RCC_APB1Periph_TIM4
#define MOTOR_PORT_RCC  RCC_APB2Periph_GPIOB

#define MOTOR1_PIN      GPIO_Pin_6
#define MOTOR2_PIN      GPIO_Pin_7
#define MOTOR3_PIN      GPIO_Pin_8
#define MOTOR4_PIN      GPIO_Pin_9

void Motor_Init(void);
inline void Motor1_SetSpeed(uint16_t speed);
inline void Motor2_SetSpeed(uint16_t speed);
inline void Motor3_SetSpeed(uint16_t speed);
inline void Motor4_SetSpeed(uint16_t speed);

inline void __attribute__((always_inline))
Motor1_SetSpeed(uint16_t speed) {
	MOTOR_TIM->CCR1 = speed;
}

inline void __attribute__((always_inline))
Motor2_SetSpeed(uint16_t speed) {
	MOTOR_TIM->CCR4 = speed;
}

inline void __attribute__((always_inline))
Motor3_SetSpeed(uint16_t speed) {
	MOTOR_TIM->CCR3 = speed;
}

inline void __attribute__((always_inline))
Motor4_SetSpeed(uint16_t speed) {
	MOTOR_TIM->CCR2 = speed;
}


#endif /* DRIVERS_MOTOR_MOTOR_H_ */
