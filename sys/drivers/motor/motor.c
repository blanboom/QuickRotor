#include "motor.h"

void Motor_Init(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(MOTOR_TIM_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(MOTOR_PORT_RCC, ENABLE);

	/* Initialize GPIO */
	GPIO_InitStructure.GPIO_Pin = MOTOR1_PIN | MOTOR2_PIN | MOTOR3_PIN
			| MOTOR4_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(MOTOR_PORT, &GPIO_InitStructure);

	//GPIO_ResetBits(MOTOR_PORT, MOTOR1_PIN | MOTOR2_PIN | MOTOR3_PIN | MOTOR4_PIN);

	/* Initialize Timer */
	TIM_TimeBaseStructure.TIM_Period = 999;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(MOTOR_TIM, &TIM_TimeBaseStructure);

	/* Initialize PWM */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;  // PWM ducy cycle = thisValue / 1000
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(MOTOR_TIM, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);
	TIM_OC2Init(MOTOR_TIM, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);
	TIM_OC3Init(MOTOR_TIM, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);
	TIM_OC4Init(MOTOR_TIM, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(MOTOR_TIM, ENABLE);

	TIM_Cmd(MOTOR_TIM, ENABLE);
}
