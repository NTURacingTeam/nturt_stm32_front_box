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
#define Encoder_SCK_Pin GPIO_PIN_2
#define Encoder_SCK_GPIO_Port GPIOE
#define EncoderSS_Pin GPIO_PIN_4
#define EncoderSS_GPIO_Port GPIOE
#define Encoder_MISO_Pin GPIO_PIN_5
#define Encoder_MISO_GPIO_Port GPIOE
#define Encoder_MOSI_Pin GPIO_PIN_6
#define Encoder_MOSI_GPIO_Port GPIOE
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define RPi_SCL_Pin GPIO_PIN_0
#define RPi_SCL_GPIO_Port GPIOF
#define RPi_SDA_Pin GPIO_PIN_1
#define RPi_SDA_GPIO_Port GPIOF
#define BSE_micro_Pin GPIO_PIN_2
#define BSE_micro_GPIO_Port GPIOF
#define HallL_Pin GPIO_PIN_3
#define HallL_GPIO_Port GPIOF
#define SuspensionL_Pin GPIO_PIN_6
#define SuspensionL_GPIO_Port GPIOF
#define BSE_Pin GPIO_PIN_10
#define BSE_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define APPS2_Pin GPIO_PIN_2
#define APPS2_GPIO_Port GPIOC
#define RTD_light_Pin GPIO_PIN_0
#define RTD_light_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define SuspensionR_Pin GPIO_PIN_3
#define SuspensionR_GPIO_Port GPIOA
#define Oil_Pin GPIO_PIN_4
#define Oil_GPIO_Port GPIOA
#define Strain_Pin GPIO_PIN_5
#define Strain_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define APPS1_Pin GPIO_PIN_1
#define APPS1_GPIO_Port GPIOB
#define VCU_light_Pin GPIO_PIN_7
#define VCU_light_GPIO_Port GPIOE
#define Gear1_Pin GPIO_PIN_8
#define Gear1_GPIO_Port GPIOE
#define APPS_micro_Pin GPIO_PIN_9
#define APPS_micro_GPIO_Port GPIOE
#define Gear3_Pin GPIO_PIN_10
#define Gear3_GPIO_Port GPIOE
#define Gear2_Pin GPIO_PIN_12
#define Gear2_GPIO_Port GPIOE
#define Gear_light_Pin GPIO_PIN_13
#define Gear_light_GPIO_Port GPIOE
#define RTD_button_Pin GPIO_PIN_15
#define RTD_button_GPIO_Port GPIOE
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define USB_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_FS_PWR_EN_GPIO_Port GPIOD
#define can_led_Pin GPIO_PIN_2
#define can_led_GPIO_Port GPIOG
#define reserverd_led_Pin GPIO_PIN_3
#define reserverd_led_GPIO_Port GPIOG
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define HallR_Pin GPIO_PIN_8
#define HallR_GPIO_Port GPIOC
#define warn_led_Pin GPIO_PIN_9
#define warn_led_GPIO_Port GPIOC
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
#define TempR_SCL_Pin GPIO_PIN_10
#define TempR_SCL_GPIO_Port GPIOC
#define TempR_SDA_Pin GPIO_PIN_11
#define TempR_SDA_GPIO_Port GPIOC
#define error_led_Pin GPIO_PIN_12
#define error_led_GPIO_Port GPIOC
#define RTD_siren_Pin GPIO_PIN_0
#define RTD_siren_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define TempL_SCL_Pin GPIO_PIN_8
#define TempL_SCL_GPIO_Port GPIOB
#define TempL_SDA_Pin GPIO_PIN_9
#define TempL_SDA_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_1
#define LED_YELLOW_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
