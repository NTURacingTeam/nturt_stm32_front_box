/**
 * @file torque_controller.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// project include
#include "project_def.h"

/* Task control --------------------------------------------------------------*/
extern uint32_t
    torque_controller_task_buffer[TORQUE_CONTROLLER_TASK_STACK_SIZE];
extern StaticTask_t torque_controller_task_cb;
extern TaskHandle_t torque_controller_task_handle;

/* Task implementation -------------------------------------------------------*/
/**
 * @brief Function implementing the torque_controller_task thread.
 * @param argument: Not used
 * @retval None
 */
void torque_controller_task(void *argument);

#endif  // TORQUE_CONTROLLER_H
