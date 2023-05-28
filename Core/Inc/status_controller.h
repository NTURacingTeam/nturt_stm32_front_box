/**
 * @file status_controller.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef STATUS_CONTROLLER_H
#define STATUS_CONTROLLER_H

// glibc include
#include <stdbool.h>
#include <stdint.h>

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// stm32_module incude
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

/* macro ---------------------------------------------------------------------*/
// parameter
#define STATUS_CONTROLLER_TASK_PRIORITY TaskPriorityHigh
#define STATUS_CONTROLLER_TASK_STACK_SIZE 256
#define STATUS_CONTROLLER_TASK_PERIOD 50

/* type ----------------------------------------------------------------------*/
typedef enum {
  /// @brief Initial state, checking RTD condition.
  StatusInit = 0,

  /// @brief Ready state, waiting RTD command to run.
  StatusReady,

  /// @brief Transition state, playing RTD sound.
  StatusRTD,

  /// @brief Running state, motor torque enabled.
  StatusRunning,

  /// @brief Error state, motor torque disabled, error handling.
  StatusError,
} StatusControllerState;

/* Exported variable ---------------------------------------------------------*/

/* class ---------------------------------------------------------------------*/
/**
 * @brief Class for controlling vehicle status.
 *
 */
typedef struct status_controller {
  // inherited class
  Task super_;

  /// @brief Current status.
  StatusControllerState status_;

  /// @brief If APPS signal is good. (Checked by APPS error handler.)
  bool apps_signal_;

  /// @brief If BSE signal is good. (Checked by BSE error handler.)
  bool bse_signal_;

  /// @brief If pedal plausibility check passes.
  bool pedal_plausibility_;

  /// @brief Inverter, BMS and rear box. (Checked by CAN timeout error handler.)
  bool receive_critical_can_;

  /// @brief If the status of inverter, bms and rear box is ok.
  bool critical_node_status_;

  /// @brief If inverter DC bus voltage is higher than minium battery voltage.
  bool inverter_voltage_;

  StackType_t task_stack_[STATUS_CONTROLLER_TASK_STACK_SIZE];
} StatusController;

/* constructor ---------------------------------------------------------------*/
/**
 * @brief Constructor for StatusController.
 *
 * @param[in,out] self The instance of the class.
 * @return None.
 */
void StatusController_ctor(StatusController* const self);

/* member function -----------------------------------------------------------*/
/**
 * @brief Function to add status controller to freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @return ModuleRet Error code.
 */
ModuleRet StatusController_start(StatusController* const self);

/**
 * @brief Function to get currnet status.
 *
 * @param[in,out] self The instance of the class.
 * @param[out] status Current status.
 * @return ModuleRet Error code.
 */
ModuleRet StatusController_get_status(StatusController* const self,
                                      StatusControllerState* const status);

/**
 * @brief Function to reset status to StatusInit.
 *
 * @param self The instance of the class.
 * @return ModuleRet Error code.
 */
ModuleRet StatusController_reset_status(StatusController* const self);

/**
 * @brief Function to handler APPS error.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] error_code Error code.
 * @retval None
 */
void StatusController_apps_error_handler(void* const _self,
                                         uint32_t error_code);

/**
 * @brief Function to handler BSE error.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] error_code Error code.
 * @retval None
 */
void StatusController_bse_error_handler(void* const _self, uint32_t error_code);

/**
 * @brief Function to handler apps error.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] error_code Error code.
 * @retval None
 */
void StatusController_can_error_handler(void* const _self, uint32_t error_code);

/**
 * @brief Function to run in freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @retval None
 * @warning For internal use only.
 */
void StatusController_task_code(void* const _self);

/* Exported function ---------------------------------------------------------*/

#endif  // STATUS_CONTROLLER_H
