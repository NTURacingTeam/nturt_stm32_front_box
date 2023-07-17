#include "status_controller.h"

// glibc include
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

// project include
#include "front_box_can.h"
#include "project_def.h"
#include "sensors.h"
#include "user_main.h"

/* Exported variable ---------------------------------------------------------*/

/* Static function prototype -------------------------------------------------*/
static void blink_rtd_light_task_code(void* argument);

static void play_rtd_sound_task_code(void* argument);

/* virtual function redirection ----------------------------------------------*/
inline ModuleRet StatusController_start(StatusController* const self) {
  return self->super_.vptr_->start((Task*)self);
}

/* virtual function definition -----------------------------------------------*/
// from Task base class
ModuleRet __StatusController_start(Task* const _self) {
  module_assert(IS_NOT_NULL(_self));

  StatusController* const self = (StatusController*)_self;

  // register error callback function
  ErrorHandler_add_error_callback(&error_handler, &self->error_callback_cb_,
                                  StatusController_error_handler, (void*)self,
                                  ERROR_CODE_ALL);

  return Task_create_freertos_task(
      (Task*)self, "status_controller", STATUS_CONTROLLER_TASK_PRIORITY,
      self->task_stack_, STATUS_CONTROLLER_TASK_STACK_SIZE);
}

/* constructor ---------------------------------------------------------------*/
void StatusController_ctor(StatusController* const self) {
  module_assert(IS_NOT_NULL(self));

  // construct inherited class and redirect virtual function
  Task_ctor((Task*)self, StatusController_task_code);
  static struct TaskVtbl vtbl = {
      .start = __StatusController_start,
  };
  self->super_.vptr_ = &vtbl;

  // initialize member variable
  self->status_ = StatusInit;

  // set rtd condition controlled by error handler to true
  self->rtd_condition_ =
      RTD_CON_CAN_TX | RTD_CON_CAN_RX_CRITICAL | RTD_CON_APPS | RTD_CON_BSE;

  self->blink_rtd_light_task_handle_ = NULL;
  self->play_rtd_sound_task_handle_ = NULL;
}

/* member function -----------------------------------------------------------*/
ModuleRet StatusController_get_status(StatusController* const self,
                                      StatusControllerState* const status) {
  *status = self->status_;

  return ModuleOK;
}

ModuleRet StatusController_reset_status(StatusController* const self) {
  switch (self->status_) {
    case StatusRTD:
      if (self->play_rtd_sound_task_handle_ != NULL &&
          eTaskGetState(self->play_rtd_sound_task_handle_) != eDeleted) {
        vTaskDelete(self->play_rtd_sound_task_handle_);
      }
      self->status_ = StatusReady;
      break;

    case StatusRunning:
      self->status_ = StatusReady;
      break;

    case StatusInit:
    case StatusReady:
    case StatusError:
      break;
  }

  return ModuleOK;
}

void StatusController_error_handler(void* const _self, uint32_t error_code) {
  StatusController* const self = (StatusController*)_self;

  if (error_code & ERROR_CODE_CAN_TX) {
    if (error_code & ERROR_SET) {
      self->rtd_condition_ &= ~RTD_CON_CAN_TX;
    } else {
      self->rtd_condition_ |= RTD_CON_CAN_TX;
    }
  }

  if (error_code & ERROR_CODE_CAN_RX_CRITICAL) {
    if (error_code & ERROR_SET) {
      self->rtd_condition_ &= ~RTD_CON_CAN_RX_CRITICAL;
    } else {
      self->rtd_condition_ |= RTD_CON_CAN_RX_CRITICAL;
    }
  }

  if (error_code & ERROR_CODE_APPS_MASK) {
    if (error_code & ERROR_SET) {
      self->rtd_condition_ &= ~RTD_CON_APPS;
    } else {
      self->rtd_condition_ |= RTD_CON_APPS;
    }
  }

  if (error_code & ERROR_CODE_BSE_MASK) {
    if (error_code & ERROR_SET) {
      self->rtd_condition_ &= ~RTD_CON_BSE;
    } else {
      self->rtd_condition_ |= RTD_CON_BSE;
    }
  }
}

void StatusController_task_code(void* const _self) {
  StatusController* const self = (StatusController*)_self;
  TickType_t last_wake = xTaskGetTickCount();

  while (1) {
    /* check states ----------------------------------------------------------*/
    // pedal plausibility
    GPIO_PinState bse_micro;
    ButtonMonitor_read_state(&button_monitor, MICRO_BSE, &bse_micro);
    xSemaphoreTake(pedal.mutex, portMAX_DELAY);
    float apps = fmaxf(pedal.apps1, pedal.apps2);
    xSemaphoreGive(pedal.mutex);
    if (self->rtd_condition_ & RTD_CON_PEDAL_PLAUSIBILITY) {
      if (bse_micro == GPIO_PIN_SET &&
          apps > PEDAL_PLAUSIBILITY_CHECK_APPS_THRESHOLD) {
        ErrorHandler_write_error(&error_handler,
                                 ERROR_CODE_PEDAL_IMPLAUSIBILITY, ERROR_SET);
        self->rtd_condition_ &= ~RTD_CON_PEDAL_PLAUSIBILITY;
      }
    } else {
      if (apps < PEDAL_PLAUSIBILITY_CHECK_RESET_APPS_THRESHOLD) {
        ErrorHandler_write_error(&error_handler,
                                 ERROR_CODE_PEDAL_IMPLAUSIBILITY, ERROR_CLEAR);
        self->rtd_condition_ |= RTD_CON_PEDAL_PLAUSIBILITY;
      }
    }

    // critical node status (rear sensor, bms, inverter)
    xSemaphoreTake(can_vcu_rx_mutex, portMAX_DELAY);
    if (can_vcu_rx.REAR_SENSOR_Status.REAR_SENSOR_Status != StatusRunning ||
        // can_vcu_rx.BMS_Status.BMS_Error_Code != 0 ||
        can_vcu_rx.INV_Fault_Codes.INV_Post_Fault_Lo ||
        can_vcu_rx.INV_Fault_Codes.INV_Post_Fault_Hi ||
        can_vcu_rx.INV_Fault_Codes.INV_Run_Fault_Lo ||
        can_vcu_rx.INV_Fault_Codes.INV_Run_Fault_Hi) {
      self->rtd_condition_ &= ~RTD_CON_CRITICAL_NODE_STATUS;
    } else {
      self->rtd_condition_ |= RTD_CON_CRITICAL_NODE_STATUS;
    }
    xSemaphoreGive(can_vcu_rx_mutex);

    // inverter voltage
    xSemaphoreTake(can_vcu_hp_rx_mutex, portMAX_DELAY);
    if (can_vcu_hp_rx.INV_Fast_Info.INV_Fast_DC_Bus_Voltage_phys >=
        MINIUMUM_BATTERY_VOLTAGE) {
      self->rtd_condition_ |= RTD_CON_INVERTER_VOLTAGE;
    } else {
      self->rtd_condition_ &= ~RTD_CON_INVERTER_VOLTAGE;
    }
    xSemaphoreGive(can_vcu_hp_rx_mutex);

    /* update status ---------------------------------------------------------*/
    switch (self->status_) {
      case StatusInit:
        if ((self->rtd_condition_ & RTD_CON_ALL) == RTD_CON_ALL) {
          LedController_turn_off(&led_controller, LED_VCU);
          self->status_ = StatusReady;
        }
        break;

      case StatusReady:
        if ((self->rtd_condition_ & RTD_CON_ALL) == RTD_CON_ALL) {
          GPIO_PinState button_state;
          ButtonMonitor_read_state(&button_monitor, MICRO_BSE, &button_state);

          if (button_state == GPIO_PIN_SET) {
            ButtonMonitor_read_state(&button_monitor, BUTTON_RTD,
                                     &button_state);
            if (button_state == GPIO_PIN_SET) {
              self->play_rtd_sound_task_handle_ = xTaskCreateStatic(
                  play_rtd_sound_task_code, "play_rtd_sound",
                  configMINIMAL_STACK_SIZE, NULL, TaskPriorityLowest,
                  self->play_rtd_sound_task_stack_,
                  &self->play_rtd_sound_task_cb_);
              self->status_ = StatusRTD;

            } else {
              if (self->blink_rtd_light_task_handle_ != NULL &&
                  eTaskGetState(self->blink_rtd_light_task_handle_) !=
                      eDeleted) {
                vTaskDelete(self->blink_rtd_light_task_handle_);
              }
              LedController_turn_on(&led_controller, LED_RTD);
            }

          } else {
            if (self->blink_rtd_light_task_handle_ == NULL ||
                eTaskGetState(self->blink_rtd_light_task_handle_) == eDeleted) {
              LedController_turn_off(&led_controller, LED_RTD);
              self->blink_rtd_light_task_handle_ = xTaskCreateStatic(
                  blink_rtd_light_task_code, "blink_rtd_light",
                  configMINIMAL_STACK_SIZE, NULL, TaskPriorityLowest,
                  self->blink_rtd_light_task_stack_,
                  &self->blink_rtd_light_task_cb_);
            }
          }

        } else {
          self->status_ = StatusError;
          if (self->blink_rtd_light_task_handle_ != NULL &&
              eTaskGetState(self->blink_rtd_light_task_handle_) != eDeleted) {
            vTaskDelete(self->blink_rtd_light_task_handle_);
          }
          LedController_turn_off(&led_controller, LED_RTD);
          LedController_turn_on(&led_controller, LED_VCU);
        }
        break;

      case StatusRTD:
        if (self->play_rtd_sound_task_handle_ == NULL ||
            eTaskGetState(self->play_rtd_sound_task_handle_) == eDeleted) {
          LedController_turn_off(&led_controller, LED_RTD);
          self->status_ = StatusRunning;
        }
        break;

      case StatusRunning:
        if ((self->rtd_condition_ & RTD_CON_ALL) != RTD_CON_ALL) {
          LedController_turn_on(&led_controller, LED_VCU);
          self->status_ = StatusError;
        }
        break;

      case StatusError:
        if ((self->rtd_condition_ & RTD_CON_ALL) == RTD_CON_ALL) {
          LedController_turn_off(&led_controller, LED_VCU);
          self->status_ = StatusReady;
        }
        break;
    }

    /* write status to can frame ---------------------------------------------*/
    xSemaphoreTake(can_vcu_tx_mutex, portMAX_DELAY);
    can_vcu_tx.VCU_Status.VCU_Status = self->status_;
    can_vcu_tx.VCU_Status.VCU_RTD_Condition = self->rtd_condition_;
    xSemaphoreGive(can_vcu_tx_mutex);

    vTaskDelayUntil(&last_wake, STATUS_CONTROLLER_TASK_PERIOD);
  }
}

/* Exported function ---------------------------------------------------------*/

/* Static function -----------------------------------------------------------*/
static void blink_rtd_light_task_code(void* argument) {
  (void)argument;

  while (1) {
    LedController_blink(&led_controller, LED_RTD, 500);
    vTaskDelay(1000);
  }
}

static void play_rtd_sound_task_code(void* argument) {
  (void)argument;

  for (int i = 0; i < 3; i++) {
    LedController_blink(&led_controller, SIREN_RTD, 300);
    vTaskDelay(500);
  }

  vTaskDelete(NULL);
}

/* Callback function ---------------------------------------------------------*/
