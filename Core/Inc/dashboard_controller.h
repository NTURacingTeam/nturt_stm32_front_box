/**
 * @file dashboard_controller.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef DASHBOARD_CONTROLLER_H
#define DASHBOARD_CONTROLLER_H

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// stm32_module incude
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

/* macro ---------------------------------------------------------------------*/
// parameter
#define DASHBOARD_CONTROLLER_TASK_PRIORITY TaskPriorityLow
#define DASHBOARD_CONTROLLER_TASK_STACK_SIZE configMINIMAL_STACK_SIZE

/* Exported variable ---------------------------------------------------------*/

/* class ---------------------------------------------------------------------*/
/**
 * @brief Class for controlling dashboard.
 *
 */
typedef struct dashboard_controller {
  // inherited class
  Task super_;

  StackType_t task_stack_[DASHBOARD_CONTROLLER_TASK_STACK_SIZE];
} DashboardController;

/* constructor ---------------------------------------------------------------*/
/**
 * @brief Constructor for DashboardController.
 *
 * @param[in,out] self The instance of the class.
 * @return None.
 */
void DashboardController_ctor(DashboardController* const self);

/* member function -----------------------------------------------------------*/
/**
 * @brief Function to add dashboard controller to freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @return ModuleRet Error code.
 */
ModuleRet DashboardController_start(DashboardController* const self);

/**
 * @brief Function to run in freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @retval None
 * @warning For internal use only.
 */
void DashboardController_task_code(void* const self);

/* Exported function ---------------------------------------------------------*/

#endif  // DASHBOARD_CONTROLLER_H
