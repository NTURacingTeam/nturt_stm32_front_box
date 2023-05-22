#include "dashboard.h"

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
__dtcmram uint32_t dashboard_task_buffer[DASHBOARD_TASK_STACK_SIZE];
__dtcmram StaticTask_t dashboard_task_cb;
TaskHandle_t dashboard_task_handle;

/* Other variable ------------------------------------------------------------*/

/* Task implementation -------------------------------------------------------*/
void dashboard_task(void *argument) {
  (void)argument;

  while (1) {
    vTaskDelay(1000);
  }
}

/* Static and callback function ----------------------------------------------*/
