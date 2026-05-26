/*
 * LSM6DS3TR.c
 *
 * Integrated Fixes:
 * 1. Added missing bias variables (bias_gx_rad, etc.)
 * 2. Fixed Complementary Filter discontinuity issue (wrapping logic)
 * 3. Uses -180 ~ +180 degree range for robust calculation
 */

#include "LSM6DS3TR.h"
#include "main.h"
#include "spi.h"
//#include "lcd.h"
#include "tim.h"
#include <stdlib.h>
#include <math.h> // 필수: atan2, sqrt, fmod

// ---------------------- 설정 상수 ----------------------
#define COMPLEMENTARY_ALPHA 0.96f // 신뢰도 계수 (자이로 96%, 가속도 4%)

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ---------------------- 전역 변수 정의 ----------------------
int16_t GyroRaw[3] = { 0 };
int16_t ACCRaw[3] = { 0 };

// 최종 계산된 각도 (사용자가 참조하는 변수)
volatile float yaw_deg = 0.f;
volatile float pitch_deg = 0.f;
volatile float roll_deg = 0.f;

volatile uint32_t prevTick = 0;

// [누락되었던 바이어스 변수 추가]
static float bias_gx_rad = 0.0f;
static float bias_gy_rad = 0.0f;
static float bias_gz_rad = 0.0f;

// 자이로 감도 (기본값)
float gyro_sens_dps_per_lsb = 0.0175f;

// ---------------------- 함수 프로토타입 ----------------------
void IMU_CalcGyroBias_All_rad(uint16_t samples, uint16_t delay_ms_each);
void IMU_GetGyroDps_Corrected(float *g_dps);
void LSM6_update_gyro_sens_from_device(void);
float gyro_raw_to_rads(int16_t raw);
float gyro_raw_to_dps(int16_t raw);
int16_t LSM6_Merge16(uint8_t lo, uint8_t hi);

// ---------------------- 기본 SPI Read/Write ----------------------

__STATIC_INLINE void LSM6DS3TR_C_ReadReg(uint8_t reg_addr, uint8_t *rx_byte,
		uint8_t size) {
	uint8_t tx_byte = reg_addr | 0x80; // 읽기 플래그(MSB=1)

	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(LSM6DS3TR_SPI_PORT, &tx_byte, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(LSM6DS3TR_SPI_PORT, rx_byte, size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

__STATIC_INLINE void LSM6DS3TR_C_WriteReg(uint8_t reg_addr, uint8_t *setting,
		uint8_t size) {
	uint8_t tx_byte[16]; // 배열 사용 (malloc 제거)
	if (size > 15)
		return;
	tx_byte[0] = reg_addr;
	for (uint8_t i = 0; i < size; i++) {
		tx_byte[i + 1] = setting[i];
	}
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(LSM6DS3TR_SPI_PORT, tx_byte, size + 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

// ---------------------- 수학 유틸리티 (중요) ----------------------

// 각도를 -180 ~ +180 범위로 정규화 (필터 연산용)
// 이 함수가 있어야 0도 근처에서 +179도와 -179도의 차이를 2도로 올바르게 계산함
static inline float wrap_deg_180(float deg) {
	while (deg > 180.0f)
		deg -= 360.0f;
	while (deg < -180.0f)
		deg += 360.0f;
	return deg;
}

// 각도를 0 ~ 360 범위로 정규화 (최종 출력용, 선택 사항)
static inline float wrap_deg_0_360(float deg) {
	deg = fmodf(deg, 360.0f);
	if (deg < 0.0f)
		deg += 360.0f;
	if (deg >= 359.999f)
		deg = 0.0f;
	return deg;
}

// ---------------------- 편의 함수 ----------------------
uint8_t LSM6DS3TR_C_ReadU8(uint8_t reg_addr) {
	uint8_t v;
	LSM6DS3TR_C_ReadReg(reg_addr, &v, 1);
	return v;
}

void LSM6DS3TR_C_WriteU8(uint8_t reg_addr, uint8_t value) {
	LSM6DS3TR_C_WriteReg(reg_addr, &value, 1);
}

// ---------------------- 초기화 및 설정 ----------------------

void LSM6DS3TR_C_ConfigCTRL(void) {
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL3_C, CTRL3_C);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL1_XL, CTRL1_XL);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL2_G, CTRL2_G);
	// 3->1->2 순서 권장사항 준수 후 나머지 설정
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL4_C, CTRL4_C);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL5_C, CTRL5_C);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL6_C, CTRL6_C);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL7_G, CTRL7_G);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL8_XL, CTRL8_XL);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL9_XL, 0x38);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL10_C, CTRL10_C);
}

void LSM6DS3TR_C_CheckCTRL() {
	uint8_t read_ctrl3 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL3_C);
	uint8_t read_ctrl1 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL1_XL);
	uint8_t read_ctrl2 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL2_G);
	uint8_t read_ctrl4 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL4_C);
	uint8_t read_ctrl5 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL5_C);
	uint8_t read_ctrl6 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL6_C);
	uint8_t read_ctrl7 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL7_G);
	uint8_t read_ctrl8 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL8_XL);
	uint8_t read_ctrl9 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL9_XL);
	uint8_t read_ctrl10 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL10_C);

	if (read_ctrl3 != CTRL3_C) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(200);
	if (read_ctrl1 != CTRL1_XL) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(200);
	if (read_ctrl2 != CTRL2_G) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(200);
	if (read_ctrl4 != CTRL4_C) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(200);
	if (read_ctrl5 != CTRL5_C) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(200);
	if (read_ctrl6 != CTRL6_C) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(200);
	if (read_ctrl7 != CTRL7_G) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(200);
	if (read_ctrl8 != CTRL8_XL) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(200);
	if (read_ctrl9 != 0x38) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
	HAL_Delay(2000);
	if (read_ctrl10 != CTRL10_C) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}

}

uint8_t LSM6DS3TR_data_ready() {
	uint8_t s = LSM6DS3TR_C_ReadU8(LSM6DS3_STATUS_REG);
	return (s & 0x03) != 0;
}

// ---------------------- 데이터 읽기 ----------------------

int16_t LSM6_Merge16(uint8_t lo, uint8_t hi) {
	return (int16_t) (((uint16_t) hi << 8) | (uint16_t) lo);
}

HAL_StatusTypeDef LSM6_ReadGyroRaw(int16_t g[3]) {
	uint8_t b[6];
	LSM6DS3TR_C_ReadReg(LSM6DS3_OUTX_L_G, b, 6);
	g[0] = LSM6_Merge16(b[0], b[1]);
	g[1] = LSM6_Merge16(b[2], b[3]);
	g[2] = LSM6_Merge16(b[4], b[5]);
	return HAL_OK;
}

HAL_StatusTypeDef LSM6_ReadAccelRaw(int16_t a[3]) {
	uint8_t b[6];
	LSM6DS3TR_C_ReadReg(LSM6DS3_OUTX_L_XL, b, 6);
	a[0] = LSM6_Merge16(b[0], b[1]);
	a[1] = LSM6_Merge16(b[2], b[3]);
	a[2] = LSM6_Merge16(b[4], b[5]);
	return HAL_OK;
}

// ---------------------- 각도 계산 로직 ----------------------

// 가속도 벡터를 이용해 Roll, Pitch 계산 (-180 ~ +180)
void IMU_GetAccelAngles(float *roll_acc, float *pitch_acc) {
	float ax = (float) ACCRaw[0];
	float ay = (float) ACCRaw[1];
	float az = (float) ACCRaw[2];

	// atan2를 사용하면 -180 ~ 180 도 범위가 나옵니다.
	*roll_acc = atan2f(ay, az) * RAD2DEG;
	*pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
}

// ---------------------- IRQ (핵심 알고리즘) ----------------------

void LSM6DS3TR_C_IRQ() {
	static float g_dps[3];
	float acc_roll_deg, acc_pitch_deg;

	if (!LSM6DS3TR_data_ready()) {
		return;
	}

	LSM6_ReadGyroRaw(GyroRaw);
	LSM6_ReadAccelRaw(ACCRaw);

	IMU_GetGyroDps_Corrected(g_dps);

	uint32_t nowTick = HAL_GetTick();
	uint32_t dt_ms = nowTick - prevTick;
	prevTick = nowTick;
	float dt_sec = dt_ms * 0.001f;

	if (dt_sec > 0.05f)
		dt_sec = 0.05f;
	if (dt_sec <= 0.0f)
		dt_sec = 0.001f;

	float gyro_roll_step = ROLL_DIR * g_dps[0] * dt_sec;
	float gyro_pitch_step = PITCH_DIR * g_dps[1] * dt_sec;
	float gyro_yaw_step = YAW_DIR * g_dps[2] * dt_sec;

	IMU_GetAccelAngles(&acc_roll_deg, &acc_pitch_deg);

	float next_roll = roll_deg + gyro_roll_step;
	float roll_err = acc_roll_deg - next_roll;
	roll_err = wrap_deg_180(roll_err);

	roll_deg = next_roll + ((1.0f - COMPLEMENTARY_ALPHA) * roll_err);
	roll_deg = wrap_deg_180(roll_deg);

	float next_pitch = pitch_deg + gyro_pitch_step;
	float pitch_err = acc_pitch_deg - next_pitch;
	pitch_err = wrap_deg_180(pitch_err);

	pitch_deg = next_pitch + ((1.0f - COMPLEMENTARY_ALPHA) * pitch_err);
	pitch_deg = wrap_deg_180(pitch_deg);

	yaw_deg += gyro_yaw_step;
	yaw_deg = wrap_deg_180(yaw_deg);
}

void LSM6DS3TR_C_Init() {
	LSM6DS3TR_C_ConfigCTRL();
	LSM6DS3TR_C_CheckCTRL();
	LSM6_update_gyro_sens_from_device();

	HAL_Delay(100);

	if (LSM6DS3TR_data_ready()) {
		LSM6_ReadGyroRaw(GyroRaw);
		LSM6_ReadAccelRaw(ACCRaw);

		// 초기 각도를 가속도 값으로 셋팅 (0도 시작 문제 방지)
		float init_roll, init_pitch;
		IMU_GetAccelAngles(&init_roll, &init_pitch);

		// init_roll *= ROLL_DIR; // 필요 시 주석 해제
		// init_pitch *= PITCH_DIR;

		roll_deg = wrap_deg_180(init_roll);
		pitch_deg = wrap_deg_180(init_pitch);
		yaw_deg = 0.0f;
	}
}

void IMU_Start() {
	uint16_t who_am_i = LSM6DS3TR_C_ReadU8(LSM6DS3TR_C_WHO_AM_I_REG);
	IMU_CalcGyroBias_All_rad(300, 1); // 300샘플 평균으로 0점 잡기

	prevTick = HAL_GetTick(); // 시작 직전 시간 초기화
	HAL_TIM_Base_Start_IT(LSM6DS3TR_TIM);
}

void IMU_Stop() {
	HAL_TIM_Base_Stop_IT(LSM6DS3TR_TIM);
}

void IMU_CalcGyroBias_All_rad(uint16_t samples, uint16_t delay_ms_each) {
	int64_t sx = 0, sy = 0, sz = 0;

	// 워밍업
	LSM6_ReadGyroRaw(GyroRaw);
	HAL_Delay(10);

	for (uint16_t i = 0; i < samples; i++) {
		LSM6_ReadGyroRaw(GyroRaw);
		sx += GyroRaw[0];
		sy += GyroRaw[1];
		sz += GyroRaw[2];
		HAL_Delay(delay_ms_each);
	}

	float mx = (float) (sx / (double) samples);
	float my = (float) (sy / (double) samples);
	float mz = (float) (sz / (double) samples);

	// 평균값을 바이어스 변수에 저장
	bias_gx_rad = gyro_raw_to_rads(mx);
	bias_gy_rad = gyro_raw_to_rads(my);
	bias_gz_rad = gyro_raw_to_rads(mz);
}

// ---------------------- 감도 및 변환 함수 ----------------------

float LSM6_get_gyro_sens_dps_per_lsb_from_CTRL2(uint8_t ctrl2) {
	uint8_t fs125 = (ctrl2 >> 1) & 0x1;
	if (fs125) {
		return 0.004375f;
	}
	uint8_t fs = (ctrl2 >> 2) & 0x3;
	switch (fs) {
	case 0:
		return 0.00875f;  // 245 dps
	case 1:
		return 0.0175f;   // 500 dps
	case 2:
		return 0.035f;    // 1000 dps
	case 3:
		return 0.07f;     // 2000 dps
	}
	return 0.0175f;
}

void LSM6_update_gyro_sens_from_device(void) {
	uint8_t ctrl2 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL2_G);
	gyro_sens_dps_per_lsb = LSM6_get_gyro_sens_dps_per_lsb_from_CTRL2(ctrl2);
}

// Raw -> dps
float gyro_raw_to_dps(int16_t raw) {
	return raw * gyro_sens_dps_per_lsb;
}

// Raw -> rad/s
float gyro_raw_to_rads(int16_t raw) {
	return gyro_raw_to_dps(raw) * DEG2RAD;
}

// 바이어스가 보정된 dps 값 가져오기
void IMU_GetGyroDps_Corrected(float *g_dps) {
	float g_radps[3];

	// 현재 Raw값 -> rad/s 변환 후 bias 빼기
	g_radps[0] = gyro_raw_to_rads(GyroRaw[0]) - bias_gx_rad;
	g_radps[1] = gyro_raw_to_rads(GyroRaw[1]) - bias_gy_rad;
	g_radps[2] = gyro_raw_to_rads(GyroRaw[2]) - bias_gz_rad;

	*(g_dps + 0) = g_radps[0] * RAD2DEG;
	*(g_dps + 1) = g_radps[1] * RAD2DEG;
	*(g_dps + 2) = g_radps[2] * RAD2DEG;
}
