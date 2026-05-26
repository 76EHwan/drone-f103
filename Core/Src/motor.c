/*
 * motor.c
 *
 *  Created on: Dec 12, 2025
 *      Author: kth59
 */

#include "motor.h"
#include "math.h"
#include "LSM6DS3TR.h"

uint32_t center_duty;
int32_t left_gain;
int32_t right_gain;
int32_t front_gain;
int32_t back_gain;

uint32_t duty[4];

void Motor_Start() {
	center_duty = 0;
	HAL_TIM_PWM_Start(MOTOR_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(MOTOR_TIM, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(MOTOR_TIM, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(MOTOR_TIM, TIM_CHANNEL_4);

	__HAL_TIM_MOE_ENABLE(MOTOR_TIM);

	MOTOR_TIM->Instance->CCR1 = 0;
	MOTOR_TIM->Instance->CCR2 = 0;
	MOTOR_TIM->Instance->CCR3 = 0;
	MOTOR_TIM->Instance->CCR4 = 0;

	HAL_TIM_Base_Start_IT(&htim4);
}

void Motor_Stop() {
	HAL_TIM_Base_Stop_IT(&htim4);

	MOTOR_TIM->Instance->CCR1 = 0;
	MOTOR_TIM->Instance->CCR2 = 0;
	MOTOR_TIM->Instance->CCR3 = 0;
	MOTOR_TIM->Instance->CCR4 = 0;

	HAL_TIM_PWM_Stop(MOTOR_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(MOTOR_TIM, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(MOTOR_TIM, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(MOTOR_TIM, TIM_CHANNEL_4);
}

void Motor_Test() {
	uint32_t test_duty = MAX_DUTY / 10;
	MOTOR_TIM->Instance->CCR1 = test_duty;
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	MOTOR_TIM->Instance->CCR1 = 0;
	MOTOR_TIM->Instance->CCR2 = test_duty;
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	MOTOR_TIM->Instance->CCR2 = 0;
	MOTOR_TIM->Instance->CCR3 = test_duty;
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	MOTOR_TIM->Instance->CCR3 = 0;
	MOTOR_TIM->Instance->CCR4 = test_duty;
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	MOTOR_TIM->Instance->CCR4 = 0;
}

void Motor_IRQ() {
	static float error_i[4] = { 0 };
	static float deg_ref[4] = { 0 };
	static float deg_pres[4] = { 0 };
	deg_ref[0] = left_gain + back_gain;
	deg_ref[1] = right_gain + back_gain;
	deg_ref[2] = left_gain + front_gain;
	deg_ref[3] = right_gain + front_gain;

	deg_pres[0] = -pitch_deg - roll_deg;
	deg_pres[1] = -pitch_deg + roll_deg;
	deg_pres[2] = pitch_deg - roll_deg;
	deg_pres[3] = pitch_deg + roll_deg;

	for (uint8_t i = 0; i < 4; i++) {
		int32_t error_p = deg_ref[i] - deg_pres[i];
		error_i[i] += error_p;
		if (error_i[i] > I_TERM_MAX)
			error_i[i] = I_TERM_MAX;
		else if (error_i[i] < -I_TERM_MAX)
			error_i[i] = -I_TERM_MAX;
		duty[i] = center_duty + GAIN_P * error_p + GAIN_I * error_i[i];
		duty[i] = fmax(0, duty[i]);
		duty[i] = fmin(duty[i], LIMIT_DUTY);
	}

	MOTOR_TIM->Instance->CCR1 = duty[0];
	MOTOR_TIM->Instance->CCR2 = duty[1];
	MOTOR_TIM->Instance->CCR3 = duty[2];
	MOTOR_TIM->Instance->CCR4 = duty[3];

}
