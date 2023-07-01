/**
 * @file torque_controller.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H

// glibc include
#include <stdbool.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// stm32_module incude
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

/* macro ---------------------------------------------------------------------*/
// parameter
#define TORQUE_CONTROLLER_TASK_PRIORITY TaskPriorityAboveNormal
#define TORQUE_CONTROLLER_TASK_STACK_SIZE (2 * configMINIMAL_STACK_SIZE)
#define TORQUE_CONTROLLER_TASK_PERIOD 10

/* Exported variable ---------------------------------------------------------*/

/* class ---------------------------------------------------------------------*/
/**
 * @brief Class for controlling motor torque output.
 *
 */
typedef struct torque_controller {
  // inherited class
  Task super_;

  /// @brief Maximum torque [N * m].
  float maximum_torque_;

  /// @brief If is reverse gear.
  bool reverse_gear_;

  /// @brief Last torque command [N * m].
  float torque_command_last_;

  /// @brief Task stack buffer.
  StackType_t task_stack_[TORQUE_CONTROLLER_TASK_STACK_SIZE];
} TorqueController;

/* constructor ---------------------------------------------------------------*/
/**
 * @brief Constructor for TorqueController.
 *
 * @param[in,out] self The instance of the class.
 * @return None.
 */
void TorqueController_ctor(TorqueController* const self);

/* member function -----------------------------------------------------------*/
/**
 * @brief Function to add torque controller to freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @return ModuleRet Error code.
 * @warning For internal use only.
 * @note This function is virtual.
 */
ModuleRet TorqueController_start(TorqueController* const _self);

/**
 * @brief Function to hendle high gear.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] state The state of the button.
 * @return None.
 * @warning For internal use only.
 */
void TorqueController_gear_high_button_callback(void* const _self,
                                                const GPIO_PinState state);

/**
 * @brief Function to hendle reverse gear.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] state The state of the button.
 * @return None.
 * @warning For internal use only.
 */
void TorqueController_gear_reverse_button_callback(void* const _self,
                                                   const GPIO_PinState state);

/**
 * @brief Function to run in freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @return None
 * @warning For internal use only.
 */
void TorqueController_task_code(void* const _self);

/* Exported function ---------------------------------------------------------*/

#endif  // TORQUE_CONTROLLER_H
