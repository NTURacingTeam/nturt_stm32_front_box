#ifndef PROJECT_DEF_H
#define PROJECT_DEF_H

// stm32 include
#include "main.h"

#define __itcmram __attribute__((section(".itcmram")))
#define __dtcmram __attribute__((section(".dtcmram")))

/* project config ------------------------------------------------------------*/
// freertos stack size
#define DASHBOARD_TASK_STACK_SIZE 128
#define STATUS_CONTROLLER_TASK_STACK_SIZE 256
#define FREERTOS_STATS_TASK_STACK_SIZE 256
#define TORQUE_CONTROLLER_TASK_STACK_SIZE 256

/* module config -------------------------------------------------------------*/
// button

// led

/* gpio config ---------------------------------------------------------------*/
// button
#define BUTTON_BUILTIN_PORT B1_GPIO_Port
#define BUTTON_BUILTIN_PIN B1_Pin

#define BUTTON_RTD_PORT GPIOE
#define BUTTON_RTD_PIN GPIO_PIN_15
#define BUTTON_GEAR1_PORT GPIOE
#define BUTTON_GEAR1_PIN GPIO_PIN_8
#define BUTTON_GEAR2_PORT GPIOE
#define BUTTON_GEAR2_PIN GPIO_PIN_12
#define BUTTON_GEAR3_PORT GPIOE
#define BUTTON_GEAR3_PIN GPIO_PIN_10

// microswitch
#define MICRO_BSE_PORT GPIOF
#define MICRO_BSE_PIN GPIO_PIN_2
#define MICRO_APPS_PORT GPIOE
#define MICRO_APPS_PIN GPIO_PIN_9

// led
#define LED_BUILTIN_GREEN_PORT LED_GREEN_GPIO_Port
#define LED_BUILTIN_GREEN_PIN LED_GREEN_Pin
#define LED_BUILTIN_YELLOW_PORT LED_YELLOW_GPIO_Port
#define LED_BUILTIN_YELLOW_PIN LED_YELLOW_Pin
#define LED_BUILTIN_RED_PORT LED_RED_GPIO_Port
#define LED_BUILTIN_RED_PIN LED_RED_Pin

#define LED_WARN_PORT GPIOC
#define LED_WARN_PIN GPIO_PIN_9
#define LED_ERROR_PORT GPIOC
#define LED_ERROR_PIN GPIO_PIN_12
#define LED_CAN_PORT GPIOG
#define LED_CAN_PIN GPIO_PIN_2
#define LED_RESERVED_PORT GPIOG
#define LED_RESERVED_PIN GPIO_PIN_3

#define LED_VCU_PORT GPIOE
#define LED_VCU_PIN GPIO_PIN_7
#define LED_RTD_PORT GPIOA
#define LED_RTD_PIN GPIO_PIN_0
#define LED_GEAR_PORT GPIOE
#define LED_GEAR_PIN GPIO_PIN_13

#define RTD_SIREN_PORT GPIOD
#define RTD_SIREN_PIN GPIO_PIN_0

#endif  // PROJECT_DEF
