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

// can_config include
#include "nturt_can_config.h"

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

  /// @brief Multiplier for maximum torque for pedal plausibility check.
  double torque_multiplier_;

  /// @brief Maximum torque [N * m].
  float maximum_torque_;

  /// @brief Last torque command [N * m].
  float torque_command_last_;

  /// @brief Inverter command message.
  INV_Command_Message_t inverter_command_;

  /// @brief Task stack buffer.
  StackType_t task_stack_[TORQUE_CONTROLLER_TASK_STACK_SIZE];

  /// @brief Task handle for blink_gear_light_task.
  TaskHandle_t blink_gear_light_task_handle_;

  /// @brief Task control block for blink_gear_light_task.
  StaticTask_t blink_gear_light_task_cb_;

  /// @brief Error callback control block.
  struct error_callback_cb error_callback_cb_;

  /// @brief Task stack buffer for link_gear_light_task.
  StackType_t blink_gear_light_task_stack_[configMINIMAL_STACK_SIZE];
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
 * @brief Function to handle pedal plausibility check.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] error_code Error code.
 * @return None
 * @warning For internal use only.
 */
void TorqueController_error_handler(void* const _self, uint32_t error_code);

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
