#include "dashboard_controller.h"

// glibc include
#include <stdint.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// stm32_module incude
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

/* Exported variable ---------------------------------------------------------*/

/* Static variable -----------------------------------------------------------*/

/* Static function prototype -------------------------------------------------*/

/* virtual function redirection ----------------------------------------------*/
inline ModuleRet DashboardController_start(DashboardController* const self) {
  return self->super_.vptr_->start((Task*)self);
}

/* virtual function definition -----------------------------------------------*/
// from Task base class
ModuleRet __DashboardController_start(Task* const _self) {
  module_assert(IS_NOT_NULL(_self));

  DashboardController* const self = (DashboardController*)_self;
  return Task_create_freertos_task(
      (Task*)self, "status_controller", DASHBOARD_CONTROLLER_TASK_PRIORITY,
      self->task_stack_, DASHBOARD_CONTROLLER_TASK_STACK_SIZE);
}

/* constructor ---------------------------------------------------------------*/
void DashboardController_ctor(DashboardController* const self) {
  module_assert(IS_NOT_NULL(self));

  // construct inherited class and redirect virtual function
  Task_ctor((Task*)self, DashboardController_task_code);
  static struct TaskVtbl vtbl = {
      .start = __DashboardController_start,
  };
  self->super_.vptr_ = &vtbl;
}

/* member function -----------------------------------------------------------*/
void DashboardController_task_code(void* const self) {
  (void)self;

  while (1) {
    vTaskDelay(1000);
  }
}

/* Exported function ---------------------------------------------------------*/

/* Static function -----------------------------------------------------------*/

/* Callback function ---------------------------------------------------------*/
