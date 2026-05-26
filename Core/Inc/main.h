/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// LEDs
#define RED_LED 15
#define GREEN_LED 14
#define BLUE_LED 13

#define RED_LED_ON GPIOB->BRR = (1 << RED_LED);
#define RED_LED_OFF GPIOB->BSRR = (1 << RED_LED);
#define GREEN_LED_ON GPIOB->BRR = (1 << GREEN_LED);
#define GREEN_LED_OFF GPIOB->BSRR = (1 << GREEN_LED);
#define BLUE_LED_ON GPIOB->BRR = (1 << BLUE_LED);
#define BLUE_LED_OFF GPIOB->BSRR = (1 << BLUE_LED);

// BLE
#define POWER_KEY 1
#define BT_RESET 9
#define BT_KEY 7

void GPIO_init (void);
void USART_init (void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
