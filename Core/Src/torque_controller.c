#include "torque_controller.h"

// glibc include
#include <stdint.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// project include
#include "project_def.h"

/* Task control --------------------------------------------------------------*/
__dtcmram uint32_t
    torque_controller_task_buffer[TORQUE_CONTROLLER_TASK_STACK_SIZE];
__dtcmram StaticTask_t torque_controller_task_cb;
TaskHandle_t torque_controller_task_handle;

/* Other variable ------------------------------------------------------------*/

/* Task implementation -------------------------------------------------------*/
void torque_controller_task(void *argument) {
  (void)argument;

  while (1) {
    vTaskDelay(1000);
  }
}

/* Static and callback function ----------------------------------------------*/
