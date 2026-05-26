/*
 * motor.h
 *
 *  Created on: Dec 12, 2025
 *      Author: kth59
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "tim.h"
#include "main.h"

#define MOTOR_TIM (&htim1)
#define MAX_DUTY (MOTOR_TIM->Instance->ARR)
#define LIMIT_DUTY (MAX_DUTY * 0.75)
#define DELTA_DUTY (MAX_DUTY * 0.05)
#define DEGREE_DIF 10

#define GAIN_P 10.f
#define GAIN_I 0.f
#define I_TERM_MAX 10

void Motor_Start();

void Motor_Stop();

void Motor_Test();

void Motor_IRQ();

extern uint32_t center_duty;
extern int32_t left_gain;
extern int32_t right_gain;
extern int32_t front_gain;
extern int32_t back_gain;


#endif /* INC_MOTOR_H_ */
