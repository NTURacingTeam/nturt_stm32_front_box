#include "sensor_reader.h"

// freertos include
#include "FreeRTOS.h"
#include "semphr.h"

// stm32_module incude
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

/* Exported variable ---------------------------------------------------------*/
// sensor data
__dtcmram PedalData pedal_data;

// mutex
__dtcmram SemaphoreHandle_t pedal_data_mutex;

/* Static variable -----------------------------------------------------------*/
// mutex control block
static __dtcmram StaticSemaphore_t pedal_data_mutex_cb;

/* Static function prototype -------------------------------------------------*/

/* virtual function redirection ----------------------------------------------*/
inline ModuleRet SensorReader_start(SensorReader* const self) {
  return self->super_.vptr_->start((Task*)self);
}

/* virtual function definition -----------------------------------------------*/
// from Task base class
ModuleRet __SensorReader_start(Task* const _self) {
  module_assert(IS_NOT_NULL(_self));

  SensorReader* const self = (SensorReader*)_self;

  // mutex for sensor data
  pedal_data_mutex = xSemaphoreCreateMutexStatic(&pedal_data_mutex_cb);

  return Task_create_freertos_task(
      (Task*)self, "sensor_reader", SENSOR_READER_TASK_PRIORITY,
      self->task_stack_, SENSOR_READER_TASK_STACK_SIZE);
}

/* constructor ---------------------------------------------------------------*/
void SensorReader_ctor(SensorReader* const self) {
  module_assert(IS_NOT_NULL(self));

  // construct inherited class and redirect virtual function
  Task_ctor((Task*)self, SensorReader_task_code);
  static struct TaskVtbl vtbl = {
      .start = __SensorReader_start,
  };
  self->super_.vptr_ = &vtbl;
}

/* member function -----------------------------------------------------------*/
void SensorReader_task_code(void* const _self) {
  SensorReader* const self = (SensorReader*)_self;
  TickType_t last_wake = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&last_wake, SENSOR_READER_TASK_PERIOD);
  }
}

/* Exported function ---------------------------------------------------------*/

/* Static function -----------------------------------------------------------*/

/* Callback function ---------------------------------------------------------*/
