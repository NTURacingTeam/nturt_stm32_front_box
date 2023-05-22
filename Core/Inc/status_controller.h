/**
 * @file status_controller.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef STATUS_CONTROLLER_H
#define STATUS_CONTROLLER_H

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// project include
#include "project_def.h"

/* Task control --------------------------------------------------------------*/
extern uint32_t
    status_controller_task_buffer[STATUS_CONTROLLER_TASK_STACK_SIZE];
extern StaticTask_t status_controller_task_cb;
extern TaskHandle_t status_controller_task_handle;

/* Task implementation -------------------------------------------------------*/
/**
 * @brief Function implementing the status_controller_task thread.
 * @param argument: Not used
 * @retval None
 */
void status_controller_task(void *argument);

#endif  // STATUS_CONTROLLER_H
