#include "torque_controller.h"

// glibc include
#include <math.h>
#include <stdint.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

// can_config include
#include "nturt_can_config.h"

// project include
#include "front_box_can.h"
#include "project_def.h"
#include "sensors.h"
#include "status_controller.h"
#include "user_main.h"

/* Static function prototype -------------------------------------------------*/
static void blink_gear_light_task_code(void* argument);

/* virtual function redirection ----------------------------------------------*/
inline ModuleRet TorqueController_start(TorqueController* const self) {
  return self->super_.vptr_->start((Task*)self);
}

/* virtual function definition -----------------------------------------------*/
// from Task base class
ModuleRet __TorqueController_start(Task* const _self) {
  module_assert(IS_NOT_NULL(_self));

  TorqueController* const self = (TorqueController*)_self;
  return Task_create_freertos_task(
      (Task*)self, "torque_controller", TORQUE_CONTROLLER_TASK_PRIORITY,
      self->task_stack_, TORQUE_CONTROLLER_TASK_STACK_SIZE);
}

/* constructor ---------------------------------------------------------------*/
void TorqueController_ctor(TorqueController* const self) {
  module_assert(IS_NOT_NULL(self));

  // construct inherited class and redirect virtual function
  Task_ctor((Task*)self, TorqueController_task_code);
  static struct TaskVtbl vtbl = {
      .start = __TorqueController_start,
  };
  self->super_.vptr_ = &vtbl;

  // initialize member variable
  self->torque_multiplier_ = 1.0F;
  self->torque_command_last_ = 0.0F;

  self->inverter_command_.VCU_Speed_Command = 0;
  self->inverter_command_.VCU_Inverter_Enable = 1;
  self->inverter_command_.VCU_Inverter_Discharge = 0;
  self->inverter_command_.VCU_Speed_Mode_Enable = 0;
  self->inverter_command_.VCU_Torque_Limit_Command_phys = 0.0F;

  // register error callback function
  ErrorHandler_add_error_callback(&error_handler, &self->error_callback_cb_,
                                  TorqueController_error_handler, (void*)self,
                                  ERROR_CODE_PEDAL_IMPLAUSIBILITY);

  // register button callback
  ButtonMonitor_register_callback(&button_monitor, GEAR_HIGH,
                                  TorqueController_gear_high_button_callback,
                                  (void*)self);
  ButtonMonitor_register_callback(&button_monitor, GEAR_REVERSE,
                                  TorqueController_gear_reverse_button_callback,
                                  (void*)self);
}

/* member function -----------------------------------------------------------*/
void TorqueController_error_handler(void* const _self, uint32_t error_code) {
  TorqueController* const self = (TorqueController*)_self;

  if (error_code & ERROR_SET) {
    self->torque_multiplier_ = 0.0F;
  } else {
    self->torque_multiplier_ = 1.0F;
  }
}

void TorqueController_gear_high_button_callback(void* const _self,
                                                const GPIO_PinState state) {
  TorqueController* const self = (TorqueController*)_self;

  if (state == GPIO_PIN_SET) {
    LedController_turn_on(&led_controller, LED_GEAR);
    self->maximum_torque_ = MAXIMUM_TORQUE_HIGH_GEAR;
  } else {
    LedController_turn_off(&led_controller, LED_GEAR);
    self->maximum_torque_ = MAXIMUM_TORQUE_NORMAL_GEAR;
  }
}

void TorqueController_gear_reverse_button_callback(void* const _self,
                                                   const GPIO_PinState state) {
  TorqueController* const self = (TorqueController*)_self;

  StatusController_reset_status(&status_controller);

  if (state == GPIO_PIN_SET) {
    self->blink_gear_light_task_handle_ = xTaskCreateStatic(
        blink_gear_light_task_code, "blink_gear_light",
        configMINIMAL_STACK_SIZE, NULL, TaskPriorityLowest,
        self->blink_gear_light_task_stack_, &self->blink_gear_light_task_cb_);
    self->inverter_command_.VCU_Direction_Command = MOTOR_REVERSE;
  } else {
    if (self->blink_gear_light_task_handle_ == NULL ||
        eTaskGetState(self->blink_gear_light_task_handle_) != eDeleted) {
      vTaskDelete(self->blink_gear_light_task_handle_);
    }
    self->inverter_command_.VCU_Direction_Command = MOTOR_FORWARD;
  }
}

void TorqueController_task_code(void* const _self) {
  TorqueController* const self = (TorqueController*)_self;
  TickType_t last_wake = xTaskGetTickCount();

  // initialize gear
  GPIO_PinState gear_state;
  ButtonMonitor_read_state(&button_monitor, GEAR_HIGH, &gear_state);
  if (gear_state == GPIO_PIN_SET) {
    LedController_turn_on(&led_controller, LED_GEAR);
    self->maximum_torque_ = MAXIMUM_TORQUE_HIGH_GEAR;
  } else {
    self->maximum_torque_ = MAXIMUM_TORQUE_NORMAL_GEAR;
    ButtonMonitor_read_state(&button_monitor, GEAR_REVERSE, &gear_state);
    if (gear_state == GPIO_PIN_SET) {
      self->blink_gear_light_task_handle_ = xTaskCreateStatic(
          blink_gear_light_task_code, "blink_gear_light",
          configMINIMAL_STACK_SIZE, NULL, TaskPriorityLowest,
          self->blink_gear_light_task_stack_, &self->blink_gear_light_task_cb_);
      self->inverter_command_.VCU_Direction_Command = MOTOR_REVERSE;
    } else {
      self->inverter_command_.VCU_Direction_Command = MOTOR_FORWARD;
    }
  }

  while (1) {
    StatusControllerState status;
    StatusController_get_status(&status_controller, &status);

    if (status == StatusRunning) {
      double maximum_torque = self->torque_multiplier_ * self->maximum_torque_;
      // calculate torque command
      xSemaphoreTake(pedal.mutex, portMAX_DELAY);
      float torque_command = fminf(pedal.apps1, pedal.apps2) * maximum_torque;
      xSemaphoreGive(pedal.mutex);
      xSemaphoreTake(can_vcu_hp_rx_mutex, portMAX_DELAY);
      float motor_speed_ =
          fabsf((float)can_vcu_hp_rx.INV_Fast_Info.INV_Fast_Motor_Speed);
      xSemaphoreGive(can_vcu_hp_rx_mutex);

      float torque_command_threshold;
#ifdef ENABLE_SOFTATART
      // if trigger soft start
      if (motor_speed_ < SOFT_START_SPEED_THRESHOLD) {
        torque_command_threshold =
            fminf(maximum_torque, fmaxf(SOFT_START_TORQUE_STARTING_POINT,
                                        self->torque_command_last_) +
                                      SOFT_START_TORQUE_SLOPE *
                                          (float)TORQUE_CONTROLLER_TASK_PERIOD /
                                          1000.0F);

      } else {
        torque_command_threshold = fminf(
            maximum_torque,
            fmaxf(NORMAL_TORQUE_STARTING_POINT, self->torque_command_last_) +
                NORMAL_TORQUE_SLOPE * (float)TORQUE_CONTROLLER_TASK_PERIOD /
                    1000.0F);
      }
#else
      torque_command_threshold = maximum_torque;
#endif  // ENABLE_SOFTATART
      self->torque_command_last_ =
          fminf(torque_command, torque_command_threshold);
      self->inverter_command_.VCU_Torque_Command_phys =
          self->torque_command_last_;

      // send inverter command
      xSemaphoreTake(can_vcu_tx_mutex, portMAX_DELAY);
      can_vcu_tx.INV_Command_Message = self->inverter_command_;
      xSemaphoreGive(can_vcu_tx_mutex);

    } else {
      static const INV_Command_Message_t disable_inverter_command = {
          .VCU_Torque_Command_phys = 0.0F,
          .VCU_Speed_Command = 0,
          .VCU_Direction_Command = MOTOR_FORWARD,
          .VCU_Inverter_Enable = 0,
          .VCU_Inverter_Discharge = 0,
          .VCU_Speed_Mode_Enable = 0,
          .VCU_Torque_Limit_Command_phys = 0.0F,
      };
      self->torque_command_last_ = 0.0F;

      // send inverter disable command
      xSemaphoreTake(can_vcu_tx_mutex, portMAX_DELAY);
      can_vcu_tx.INV_Command_Message = disable_inverter_command;
      xSemaphoreGive(can_vcu_tx_mutex);
    }

    vTaskDelayUntil(&last_wake, TORQUE_CONTROLLER_TASK_PERIOD);
  }
}

/* Exported function ---------------------------------------------------------*/

/* Static function -----------------------------------------------------------*/
static void blink_gear_light_task_code(void* argument) {
  (void)argument;

  while (1) {
    LedController_blink(&led_controller, LED_GEAR, 500);
    vTaskDelay(1000);
  }
}

/* Callback function ---------------------------------------------------------*/
