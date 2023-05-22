/**
 * @file dashboard.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef DASHBOARD_H
#define DASHBOARD_H

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// project include
#include "project_def.h"

/* Task control --------------------------------------------------------------*/
extern uint32_t dashboard_task_buffer[DASHBOARD_TASK_STACK_SIZE];
extern StaticTask_t dashboard_task_cb;
extern TaskHandle_t dashboard_task_handle;

/* Task implementation -------------------------------------------------------*/
/**
 * @brief Function implementing the dashboard_task thread.
 * @param argument: Not used
 * @retval None
 */
void dashboard_task(void *argument);

#endif  // DASHBOARD_H
