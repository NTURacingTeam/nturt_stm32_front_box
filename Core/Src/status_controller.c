#include "status_controller.h"

// glibc include
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

/* Static variable -----------------------------------------------------------*/
static TaskHandle_t blink_rtd_light_task_handle = NULL;
static TaskHandle_t play_rtd_sound_task_handle = NULL;

static StaticTask_t blink_rtd_light_task_cb;
static StaticTask_t play_rtd_sound_task_cb;

static uint32_t blink_rtd_light_task_stack[configMINIMAL_STACK_SIZE];
static uint32_t play_rtd_sound_task_stack[configMINIMAL_STACK_SIZE];

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
                                  ERROR_CODE_APPS_IMPLAUSIBILITY |
                                      ERROR_CODE_BSE_IMPLAUSIBILITY |
                                      ERROR_CODE_CAN_RX_CRITICAL);

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
  self->rtd_condition_ = RTD_CON_APPS | RTD_CON_BSE | RTD_CON_CAN_TX |
                         RTD_CON_CAN_RX_CRITICAL | RTD_CON_PEDAL_PLAUSIBILITY;
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
      if (play_rtd_sound_task_handle != NULL &&
          eTaskGetState(play_rtd_sound_task_handle) != eDeleted) {
        vTaskDelete(play_rtd_sound_task_handle);
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

  if (error_code & ERROR_CODE_APPS_IMPLAUSIBILITY) {
    if (error_code & ERROR_SET) {
      self->rtd_condition_ &= ~RTD_CON_APPS;
    } else {
      self->rtd_condition_ |= RTD_CON_APPS;
    }
  }

  if (error_code & ERROR_CODE_BSE_IMPLAUSIBILITY) {
    if (error_code & ERROR_SET) {
      self->rtd_condition_ &= ~RTD_CON_BSE;
    } else {
      self->rtd_condition_ |= RTD_CON_BSE;
    }
  }

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
    float apps = pedal.apps1;
    xSemaphoreGive(pedal.mutex);
    if (self->rtd_condition_ & RTD_CON_PEDAL_PLAUSIBILITY) {
      if (bse_micro == GPIO_PIN_SET &&
          apps > PEDAL_PLAUSIBILITY_CHECK_APPS_THRESHOLD) {
        ErrorHandler_write_error(&error_handler,
                                 ERROR_CODE_PEDAL_IMPLAUSIBILITY, ERROR_SET);
        self->rtd_condition_ &= ~RTD_CON_PEDAL_PLAUSIBILITY;
      }
    } else {
      if (apps < PEDAL_PLAUSIBILITY_CHECK_APPS_THRESHOLD) {
        ErrorHandler_write_error(&error_handler,
                                 ERROR_CODE_PEDAL_IMPLAUSIBILITY, ERROR_CLEAR);
        self->rtd_condition_ |= RTD_CON_PEDAL_PLAUSIBILITY;
      }
    }

    // critical node status (rear sensor)
    xSemaphoreTake(can_vcu_rx_mutex, portMAX_DELAY);
    // if (can_vcu_rx.REAR_SENSOR_Status.REAR_SENSOR_Status == StatusRunning) {
    if (true) {
      self->rtd_condition_ |= RTD_CON_CRITICAL_NODE_STATUS;
    } else {
      self->rtd_condition_ &= ~RTD_CON_CRITICAL_NODE_STATUS;
    }

    if (can_vcu_rx.INV_Fault_Codes.INV_Post_Fault_Lo ||
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
    // if (can_vcu_hp_rx.INV_Fast_Info.INV_Fast_DC_Bus_Voltage_phys >=
    //     MINIUMUM_BATTERY_VOLTAGE) {
    if (true) {
      self->rtd_condition_ |= RTD_CON_INVERTER_VOLTAGE;
    } else {
      self->rtd_condition_ &= ~RTD_CON_INVERTER_VOLTAGE;
    }
    xSemaphoreGive(can_vcu_hp_rx_mutex);

    /* update status ---------------------------------------------------------*/
    switch (self->status_) {
      case StatusInit:
        if (self->rtd_condition_ == RTD_CON_ALL) {
          LedController_turn_off(&led_controller, LED_VCU);
          self->status_ = StatusReady;
        }
        break;

      case StatusReady:
        if (self->rtd_condition_ == RTD_CON_ALL) {
          GPIO_PinState button_state;
          ButtonMonitor_read_state(&button_monitor, MICRO_BSE, &button_state);

          if (button_state == GPIO_PIN_SET) {
            ButtonMonitor_read_state(&button_monitor, BUTTON_RTD,
                                     &button_state);
            if (button_state == GPIO_PIN_SET) {
              play_rtd_sound_task_handle = xTaskCreateStatic(
                  play_rtd_sound_task_code, "play_rtd_sound",
                  configMINIMAL_STACK_SIZE, NULL, TaskPriorityLowest,
                  play_rtd_sound_task_stack, &play_rtd_sound_task_cb);
              self->status_ = StatusRTD;

            } else {
              if (blink_rtd_light_task_handle != NULL &&
                  eTaskGetState(blink_rtd_light_task_handle) != eDeleted) {
                vTaskDelete(blink_rtd_light_task_handle);
              }
              LedController_turn_on(&led_controller, LED_RTD);
            }

          } else {
            if (blink_rtd_light_task_handle == NULL ||
                eTaskGetState(blink_rtd_light_task_handle) == eDeleted) {
              LedController_turn_off(&led_controller, LED_RTD);
              blink_rtd_light_task_handle = xTaskCreateStatic(
                  blink_rtd_light_task_code, "blink_rtd_light",
                  configMINIMAL_STACK_SIZE, NULL, TaskPriorityLowest,
                  blink_rtd_light_task_stack, &blink_rtd_light_task_cb);
            }
          }

        } else {
          self->status_ = StatusError;
          if (blink_rtd_light_task_handle != NULL &&
              eTaskGetState(blink_rtd_light_task_handle) != eDeleted) {
            vTaskDelete(blink_rtd_light_task_handle);
          }
          LedController_turn_off(&led_controller, LED_RTD);
          LedController_turn_on(&led_controller, LED_VCU);
        }
        break;

      case StatusRTD:
        if (play_rtd_sound_task_handle == NULL ||
            eTaskGetState(play_rtd_sound_task_handle) == eDeleted) {
          LedController_turn_off(&led_controller, LED_RTD);
          self->status_ = StatusRunning;
        }
        break;

      case StatusRunning:
        if (self->rtd_condition_ != RTD_CON_ALL) {
          LedController_turn_on(&led_controller, LED_VCU);
          self->status_ = StatusError;
        }
        break;

      case StatusError:
        if (self->rtd_condition_ == RTD_CON_ALL) {
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
