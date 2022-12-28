/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define ADC_DMA_ARRAY_RANK_LTRAVEL 0
#define ADC_DMA_ARRAY_RANK_RTRAVEL 1
#define ADC_DMA_ARRAY_RANK_OILPRESSURE 2
#define ADC_DMA_ARRAY_RANK_APPS1 3
#define ADC_DMA_ARRAY_RANK_APPS2 4
#define ADC_DMA_ARRAY_RANK_BSE 5
#define TEST_LED_Pin GPIO_PIN_13
#define TEST_LED_GPIO_Port GPIOC
#define STEER_SENS_CS_Pin GPIO_PIN_14
#define STEER_SENS_CS_GPIO_Port GPIOC
#define BSE_MICRO_Pin GPIO_PIN_0
#define BSE_MICRO_GPIO_Port GPIOA
#define BSE_MICRO_EXTI_IRQn EXTI0_IRQn
#define APPS_MICRO_Pin GPIO_PIN_1
#define APPS_MICRO_GPIO_Port GPIOA
#define APPS_MICRO_EXTI_IRQn EXTI1_IRQn
#define LEFT_HALL_SENS_Pin GPIO_PIN_7
#define LEFT_HALL_SENS_GPIO_Port GPIOA
#define LEFT_HALL_SENS_EXTI_IRQn EXTI9_5_IRQn
#define RIGHT_HALL_SENS_Pin GPIO_PIN_5
#define RIGHT_HALL_SENS_GPIO_Port GPIOB
#define RIGHT_HALL_SENS_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

//#define ADC_LTRAVEL 0		/*IN6,PA6*/
//#define ADC_RTRAVEL 1		/*IN9,PB1*/
//#define ADC_OILPRESSURE 2	/*IN8,PB0*/
//#define ADC_APPS1 3		/*IN2,PA2*/
//#define ADC_APPS2 4		/*IN3,PA3*/
//#define ADC_BSE 5			/*IN4,PA4*/

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
