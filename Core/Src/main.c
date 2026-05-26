/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"
#include "main.h"
#include "tools.h"
#include "usart.h"
#include "hm19.h"
#include "LSM6DS3TR.h"
#include "motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		if (rx_data == '0') { // Toggle LD2
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}
		else if(rx_data == '1') { // Test
			IMU_Start();
			HAL_Delay(10);
			Motor_Start();
			Motor_Test();
			Motor_Stop();
		}
		else if(rx_data == '2') { // Start
			IMU_Start();
			Motor_Start();
//			center_duty = MOTOR_TIM->Instance->ARR / 2;
		}
		else if(rx_data == '3') { // Up
			center_duty += (LIMIT_DUTY > center_duty) ? (DELTA_DUTY) : 0;
			if(center_duty > LIMIT_DUTY) center_duty = LIMIT_DUTY;
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}
		else if(rx_data == '4') { // Down
			center_duty -= (DELTA_DUTY < center_duty) ? (DELTA_DUTY) : 0;
			if(center_duty < DELTA_DUTY) center_duty = 0;
		}
		else if(rx_data == '5') { // Left
			left_gain = -DEGREE_DIF;
			right_gain = DEGREE_DIF;
		}
		else if(rx_data == '6') { // Right
			left_gain = DEGREE_DIF;
			right_gain = -DEGREE_DIF;
		}
		else if(rx_data == '7') { // Front
			front_gain = -DEGREE_DIF;
			back_gain = DEGREE_DIF;
		}
		else if(rx_data == '8') { // Back
			front_gain = DEGREE_DIF;
			back_gain = -DEGREE_DIF;
		}
		else if(rx_data == '9') { // Stop
			Motor_Stop();
			IMU_Stop();
		}
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		LSM6DS3TR_C_IRQ();
	}
	else if (htim->Instance == TIM4) {
		Motor_IRQ();
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

//  	GPIO_init();
//  	USART_init();
//  	HM19_init(USART1);
//
//  	USART_print_string (USART2, "[SYSTEM] DEVICE IS READY FOR WORK!\n");
//
//  	Delay_ms(5);
//  	HM19_send_command("AT");
//
//  	static uint8_t first_on = 1;
//	RCC->APB2ENR |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 14) | (1 << 0);
//
//	// 2. [모터 정지] JTAG 끄기 & 모든 핀 0V 만들기
//	AFIO->MAPR &= ~(7 << 24); // JTAG 설정 초기화
//	AFIO->MAPR |= (2 << 24);  // JTAG 끔 (PB3, PB4 제어권 확보)
//
//	// 모든 핀을 '출력'으로 설정 (PA13,14 제외)
//	GPIOA->CRL = 0x33333333;
//	GPIOA->CRH = 0x44433333;
//	GPIOB->CRL = 0x33333333;
//	GPIOB->CRH = 0x33333333;
//
//	// 모든 핀 끄기 (Low) -> 모터 4개 전부 멈춤!
//	GPIOA->BRR = 0xEFFF;
//	GPIOB->BRR = 0xFFFF;
//
//	// 3. [LED 설정] PC13 출력 설정
//	GPIOC->CRH &= ~(0xF << 20);
//	GPIOC->CRH |= (0x3 << 20);
//
//	// 4. [블루투스] 핀 및 속도 설정 (9600bps)
//	GPIOA->CRH &= ~(0xF << 4);
//	GPIOA->CRH |= (0xB << 4); // TX
//	GPIOA->CRH &= ~(0xF << 8);
//	GPIOA->CRH |= (0x8 << 8);
//	GPIOA->ODR |= (1 << 10); // RX
//
//	USART1->BRR = 833; // 9600bps
//	USART1->CR1 = (1 << 13) | (1 << 3) | (1 << 2);
//
//	HAL_UART_Receive_IT(&huart2, &uart2_rx_buff, 1);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	HAL_Delay(10);

	LSM6DS3TR_C_Init();

	HAL_Delay(10);

//	IMU_Start();

	HAL_UART_Receive_IT(&huart2, &rx_data, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
//	  USART_RX_timer();
//
//	  		static char data[128];
//	  		int lng = USART1_read(data, 128);
//
//	  		if (lng > 0) {
//	  			data[lng] = 0;
//	  			USART_printf_string (USART2, "[DEBUG] <- ", data);
//
//	  			if (strcmp("red-on", data) == 0) {
//	  				RED_LED_ON
//	  			} else if (strcmp("red-off", data) == 0) {
//	  				RED_LED_OFF
//	  			} else if (strcmp("green-on", data) == 0) {
//	  				GREEN_LED_ON
//	  			} else if (strcmp("green-off", data) == 0) {
//	  				GREEN_LED_OFF
//	  			} else if (strcmp("blue-on", data) == 0) {
//	  				BLUE_LED_ON
//	  			} else if (strcmp("blue-off", data) == 0) {
//	  				BLUE_LED_OFF
//	  			}
//
//	  			if (first_on) {
//	  				HM19_send_command("AT+NAME?");
//	  				first_on = 0;
//	  			}
//
//	  			USART1_read_completed();
//	  		}
//		if (USART1->SR & (1 << 5)) // 데이터 수신 확인
//				{
//			char cmd = USART1->DR; // 문자 받기
//
//			// '0'을 받으면 -> "OFF" 전송
//			if (cmd == '0') {
//				while (!(USART1->SR & (1 << 7)))
//					;
//				USART1->DR = 'O';
//				while (!(USART1->SR & (1 << 7)))
//					;
//				USART1->DR = 'F';
//				while (!(USART1->SR & (1 << 7)))
//					;
//				USART1->DR = 'F';
//				while (!(USART1->SR & (1 << 7)))
//					;
//				USART1->DR = '\n';
//			}
//			// '1'을 받으면 -> "ON" 전송
//			else if (cmd == '1') {
//				while (!(USART1->SR & (1 << 7)))
//					;
//				USART1->DR = 'O';
//				while (!(USART1->SR & (1 << 7)))
//					;
//				USART1->DR = 'N';
//				while (!(USART1->SR & (1 << 7)))
//					;
//				USART1->DR = '\n';
//			}
//		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void GPIO_init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
//	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
//	RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;

//	AFIO->MAPR |= AFIO_MAPR_USART2_REMAP;

	GPIO_setup(GPIOA, POWER_KEY, OUTPUT_50MHZ_PP);
//	GPIO_setup(GPIOE, BT_RESET, OUTPUT_50MHZ_PP);
	GPIO_setup(GPIOB, BT_KEY, OUTPUT_50MHZ_PP);

	GPIOA->BSRR = (1 << POWER_KEY);
//	GPIOE->BSRR = (1 << BT_RESET);
	GPIOB->BSRR = (1 << BT_KEY);

	// LEDS
	GPIO_setup(GPIOB, RED_LED, OUTPUT_50MHZ_PP);
	GPIO_setup(GPIOB, GREEN_LED, OUTPUT_50MHZ_PP);
	GPIO_setup(GPIOB, BLUE_LED, OUTPUT_50MHZ_PP);
	RED_LED_OFF
	GREEN_LED_OFF
	BLUE_LED_OFF
}

//void USART_init(void) {
//	USART_setup(USART1, A9_TX_A10_RX, 9600, 2);
//	USART_setup(USART2, A2_TX_A3_RX, 115200, 2);
//
//	NVIC_EnableIRQ(USART1_IRQn);
//	NVIC_SetPriority(USART1_IRQn, 1);
//
//	static char ble_usart_rx_buff[128] = { 0 };
//	Set_USART_buff(USART1, ble_usart_rx_buff);
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
