/**
 * @file user_main.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef USER_MAIN_H
#define USER_MAIN_H

// glibc include
#include <stdint.h>

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "front_box_can.h"
#include "project_def.h"

/* Exported variable ---------------------------------------------------------*/
extern FrontBoxCan front_box_can;
extern ErrorHandler error_handler;

/* Entry point ---------------------------------------------------------------*/
/**
 * @brief Project initialize entry point.
 * @retval None
 */
void user_init();

/* Task control --------------------------------------------------------------*/
extern uint32_t freertos_stats_task_buffer[FREERTOS_STATS_TASK_STACK_SIZE];
extern StaticTask_t freertos_stats_task_cb;
extern TaskHandle_t freertos_stats_task_handle;

/* Task implementation -------------------------------------------------------*/
/**
 * @brief Function implementing the freertos_stats_task thread.
 * @param argument: Not used
 * @retval None
 */
void freertos_stats_task(void *argument);

/* Exported function ---------------------------------------------------------*/
/**
 * @brief Function to get 10 microsecond since the microcontroller booted.
 *
 * The time source comes from the counter of 32-bit timer TIM3 that tick up
 * every 10 microseconds.
 * @retval Current 10 microsecond.
 * @warning The value will wraparound roughly 11 hours.
 */
uint32_t get_10us();

#endif  // USER_MAIN_H
