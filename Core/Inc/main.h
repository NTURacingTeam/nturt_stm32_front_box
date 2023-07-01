/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

extern FDCAN_HandleTypeDef hfdcan3;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c5;

extern IWDG_HandleTypeDef hiwdg1;

extern SPI_HandleTypeDef hspi4;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim17;

extern UART_HandleTypeDef huart3;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WHEEL_SPEED_TIMER_PERIOD 10.0
#define WHEEL_SPEED_TIMER_COUNT_PERIOD 0.001
#define Encoder_SCK_Pin GPIO_PIN_2
#define Encoder_SCK_GPIO_Port GPIOE
#define ENCODER_SS_Pin GPIO_PIN_4
#define ENCODER_SS_GPIO_Port GPIOE
#define Encoder_MISO_Pin GPIO_PIN_5
#define Encoder_MISO_GPIO_Port GPIOE
#define Encoder_MOSI_Pin GPIO_PIN_6
#define Encoder_MOSI_GPIO_Port GPIOE
#define BUTTON_BUILTIN_Pin GPIO_PIN_13
#define BUTTON_BUILTIN_GPIO_Port GPIOC
#define RPI_SCL_Pin GPIO_PIN_0
#define RPI_SCL_GPIO_Port GPIOF
#define RPI_SDA_Pin GPIO_PIN_1
#define RPI_SDA_GPIO_Port GPIOF
#define MICRO_BSE_Pin GPIO_PIN_2
#define MICRO_BSE_GPIO_Port GPIOF
#define HALL_L_Pin GPIO_PIN_3
#define HALL_L_GPIO_Port GPIOF
#define SUSPENSION_L_Pin GPIO_PIN_6
#define SUSPENSION_L_GPIO_Port GPIOF
#define BSE_Pin GPIO_PIN_10
#define BSE_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define APPS_2_Pin GPIO_PIN_2
#define APPS_2_GPIO_Port GPIOC
#define LED_RTD_Pin GPIO_PIN_0
#define LED_RTD_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define SUSPENSION_R_Pin GPIO_PIN_3
#define SUSPENSION_R_GPIO_Port GPIOA
#define OIL_PRESSURE_Pin GPIO_PIN_4
#define OIL_PRESSURE_GPIO_Port GPIOA
#define STRAIN_Pin GPIO_PIN_5
#define STRAIN_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LED_BUILTIN_GREEN_Pin GPIO_PIN_0
#define LED_BUILTIN_GREEN_GPIO_Port GPIOB
#define APPS_1_Pin GPIO_PIN_1
#define APPS_1_GPIO_Port GPIOB
#define LED_VCU_Pin GPIO_PIN_7
#define LED_VCU_GPIO_Port GPIOE
#define GEAR_HIGH_Pin GPIO_PIN_8
#define GEAR_HIGH_GPIO_Port GPIOE
#define MICRO_APPS_Pin GPIO_PIN_9
#define MICRO_APPS_GPIO_Port GPIOE
#define GEAR_REVERSE_Pin GPIO_PIN_12
#define GEAR_REVERSE_GPIO_Port GPIOE
#define LED_GEAR_Pin GPIO_PIN_13
#define LED_GEAR_GPIO_Port GPIOE
#define BUTTON_RTD_Pin GPIO_PIN_15
#define BUTTON_RTD_GPIO_Port GPIOE
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LED_BUILTIN_RED_Pin GPIO_PIN_14
#define LED_BUILTIN_RED_GPIO_Port GPIOB
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define USB_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_FS_PWR_EN_GPIO_Port GPIOD
#define LED_CAN_TX_Pin GPIO_PIN_2
#define LED_CAN_TX_GPIO_Port GPIOG
#define LED_CAN_RX_Pin GPIO_PIN_3
#define LED_CAN_RX_GPIO_Port GPIOG
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define HALL_R_Pin GPIO_PIN_8
#define HALL_R_GPIO_Port GPIOC
#define LED_WARN_Pin GPIO_PIN_9
#define LED_WARN_GPIO_Port GPIOC
#define USB_FS_VBUS_Pin GPIO_PIN_9
#define USB_FS_VBUS_GPIO_Port GPIOA
#define USB_FS_ID_Pin GPIO_PIN_10
#define USB_FS_ID_GPIO_Port GPIOA
#define USB_FS_DM_Pin GPIO_PIN_11
#define USB_FS_DM_GPIO_Port GPIOA
#define USB_FS_DP_Pin GPIO_PIN_12
#define USB_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TEMP_R_SCL_Pin GPIO_PIN_10
#define TEMP_R_SCL_GPIO_Port GPIOC
#define TEMP_R_SDA_Pin GPIO_PIN_11
#define TEMP_R_SDA_GPIO_Port GPIOC
#define LED_ERROR_Pin GPIO_PIN_12
#define LED_ERROR_GPIO_Port GPIOC
#define SIREN_RTD_Pin GPIO_PIN_0
#define SIREN_RTD_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define TEMP_L_SCL_Pin GPIO_PIN_8
#define TEMP_L_SCL_GPIO_Port GPIOB
#define TEMP_L_SDA_Pin GPIO_PIN_9
#define TEMP_L_SDA_GPIO_Port GPIOB
#define LED_BUILTIN_YELLOW_Pin GPIO_PIN_1
#define LED_BUILTIN_YELLOW_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
