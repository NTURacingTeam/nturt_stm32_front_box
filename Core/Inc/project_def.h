#ifndef PROJECT_DEF_H
#define PROJECT_DEF_H

// stm32 include
#include "main.h"

#define __itcmram __attribute__((section(".itcmram")))
#define __dtcmram __attribute__((section(".dtcmram")))
#define __dma_buffer __attribute__((section(".dma_buffer")))

/* project config ------------------------------------------------------------*/
// freertos stack size
#define FREERTOS_STATS_TASK_STACK_SIZE 256

// fsae rule
#define PEDAL_PLAUSIBILITY_CHECK_APPS_THRESHOLD 0.25F

// vehicle parameter
/// @brief Moror forward direction bit for sending command to inverter.
#define MOTOR_FORWARD 0

/// @brief Moror reverse direction bit for sending command to inverter.
#define MOTOR_REVERSE 1

/// @brief Maximum torque at normal gear in [N * m].
#define MAXIMUM_TORQUE_NORMAL_GEAR 80.0F

/// @brief Maximum torque in high gear in [N * m].
#define MAXIMUM_TORQUE_HIGH_GEAR 134.0F

/// @brief Threshold for motor spped when lower, trigger soft start in [RPM].
#define SOFT_START_SPEED_THRESHOLD 60.0F

/// @brief Torque slope for soft start [N * m / s].
#define SOFT_START_TORQUE_SLOPE 10.0F

/// @brief Torque output starting point when triggering soft start [N * m].
#define SOFT_START_TORQUE_STARTING_POINT 5.0F

/// @brief Miniumum safe battery voltage in [V].
#define MINIUMUM_BATTERY_VOLTAGE 250

/* module config -------------------------------------------------------------*/
/* button --------------------------------------------------------------------*/
#define NUM_BUTTON_BUILTIN 1
#define NUM_BUTTON_USER 1
#define NUM_GEAR 2
#define NUM_MICRO 2
#define NUM_BUTTON (NUM_BUTTON_BUILTIN + NUM_BUTTON_USER + NUM_GEAR + NUM_MICRO)

// built in button
#define BUTTON_BUILTIN 0

// user button
#define BUTTON_USER_BASE (NUM_BUTTON_BUILTIN + NUM_BUTTON_BUILTIN)
#define BUTTON_USER(X) (BUTTON_USER_BASE + X)

#define BUTTON_RTD BUTTON_USER(0)

// gear
#define GEAR_BASE (BUTTON_USER_BASE + NUM_BUTTON_USER)
#define GEAR(X) (GEAR_BASE + X)

#define GEAR_HIGH GEAR(0)
#define GEAR_REVERSE GEAR(1)

// microswitch
#define MICRO_BASE (GEAR_BASE + NUM_GEAR)
#define MICRO(X) (MICRO_BASE + X)

#define MICRO_APPS MICRO(0)
#define MICRO_BSE MICRO(1)

/* led (plus siren) ----------------------------------------------------------*/
#define NUM_LED_BUILTIN 3
#define NUM_LED_ONBOARD 4
#define NUM_LED_USER 3
#define NUM_SIREN 1
#define NUM_LED (NUM_LED_BUILTIN + NUM_LED_ONBOARD + NUM_LED_USER + NUM_SIREN)

// built in led
#define LED_BUILTIN_BASE 0
#define LED_BUILTIN(X) (LED_BUILTIN_BASE + X)

#define LED_BUILTIN_GREEN LED_BUILTIN(0)
#define LED_BUILTIN_YELLOW LED_BUILTIN(1)
#define LED_BUILTIN_RED LED_BUILTIN(2)

// onboard led
#define LED_ONBOARD_BASE (LED_BUILTIN_BASE + NUM_LED_BUILTIN)
#define LED_ONBOARD(X) (LED_ONBOARD_BASE + X)

#define LED_WARN LED_ONBOARD(0)
#define LED_ERROR LED_ONBOARD(1)
#define LED_CAN_TX LED_ONBOARD(2)
#define LED_CAN_RX LED_ONBOARD(3)

// user led
#define LED_USER_BASE (LED_ONBOARD_BASE + NUM_LED_ONBOARD)
#define LED_USER(X) (LED_USER_BASE + X)

#define LED_VCU LED_USER(0)
#define LED_RTD LED_USER(1)
#define LED_GEAR LED_USER(2)

// siren
#define SIREN_RTD (LED_USER_BASE + NUM_LED_USER)

#endif  // PROJECT_DEF
