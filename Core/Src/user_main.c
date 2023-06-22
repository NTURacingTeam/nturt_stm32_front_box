#include "user_main.h"

// glibc include
#include <stdio.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "dashboard.h"
#include "front_box_can.h"
#include "project_def.h"
#include "status_controller.h"
#include "torque_controller.h"
#include "sensors.h"

/* Exported variable ---------------------------------------------------------*/
__dtcmram FrontBoxCan front_box_can;
__dtcmram ErrorHandler error_handler;

/* Static variable -----------------------------------------------------------*/

/* Entry point ---------------------------------------------------------------*/
void user_init() {
  // stm32_module
  FrontBoxCan_ctor(&front_box_can, &hfdcan3);
  CanTransceiver_start((CanTransceiver *)&front_box_can);
  ErrorHandler_ctor(&error_handler);
  ErrorHandler_start(&error_handler);

  // project
  dashboard_task_handle = xTaskCreateStatic(
      dashboard_task, "dashboard_task", FREERTOS_STATS_TASK_STACK_SIZE, NULL,
      TaskPriorityLow, dashboard_task_buffer, &dashboard_task_cb);
  freertos_stats_task_handle =
      xTaskCreateStatic(freertos_stats_task, "freertos_stats_task",
                        FREERTOS_STATS_TASK_STACK_SIZE, NULL, TaskPriorityLow,
                        freertos_stats_task_buffer, &freertos_stats_task_cb);
  status_controller_task_handle = xTaskCreateStatic(
      status_controller_task, "status_controller_task",
      STATUS_CONTROLLER_TASK_STACK_SIZE, NULL, TaskPriorityHigh,
      status_controller_task_buffer, &status_controller_task_cb);
  torque_controller_task_handle = xTaskCreateStatic(
      torque_controller_task, "torque_controller_task",
      TORQUE_CONTROLLER_TASK_STACK_SIZE, NULL, TaskPriorityHigh,
      torque_controller_task_buffer, &torque_controller_task_cb);
  sensors_data_task_handle = xTaskCreateStatic(
      sensor_handler,
      "sensors_data_task",
      SENSOR_DATA_TASK_STACK_SIZE,
      NULL,
      TaskPriorityHigh,
      sensors_data_task_buffer,
      &sensors_data_task_cb
  );
  sensor_timer_handle = xTimerCreateStatic(
    "sensors_data_timer",
    pdMS_TO_TICKS(SENSOR_TIMER_PERIOD),
    pdTRUE,
    0,
    sensor_timer_callback,
    &sensor_timer_buffer
  );
  pedal.mutex = xSemaphoreCreateMutex();
  travel_strain_sensor.mutex = xSemaphoreCreateMutex();
  tire_temp_sensor.mutex = xSemaphoreCreateMutex();
}

/* Task control --------------------------------------------------------------*/
__dtcmram uint32_t freertos_stats_task_buffer[FREERTOS_STATS_TASK_STACK_SIZE];
__dtcmram StaticTask_t freertos_stats_task_cb;
TaskHandle_t freertos_stats_task_handle;

/* Task implementation -------------------------------------------------------*/
void freertos_stats_task(void *argument) {
  (void)argument;

  while (1) {
    int task_size = uxTaskGetNumberOfTasks();
    TaskStatus_t *task_status = pvPortMalloc(task_size * sizeof(TaskStatus_t));
    uint32_t total_run_time;
    uxTaskGetSystemState(task_status, task_size, &total_run_time);

    printf("Task count = %d\n", task_size);
    printf("No Name             Status      Usage   HW\n");

    for (int i = 0; i < task_size; i++) {
      float runtime_percent = (100.0 * (float)task_status[i].ulRunTimeCounter /
                               (float)total_run_time);

      char *task_state;
      switch (task_status[i].eCurrentState) {
        case eRunning:
          task_state = "RUNNING";
          break;
        case eReady:
          task_state = "READY";
          break;
        case eBlocked:
          task_state = "BLOCKED";
          break;
        case eSuspended:
          task_state = "SUSPENED";
          break;
        case eDeleted:
          task_state = "DELETED";
          break;
        default:
          task_state = "UNKNOWN";
      }

      printf("%2d %-16s %-8s %7.4f%% %4u\n", i, task_status[i].pcTaskName,
             task_state, runtime_percent, task_status[i].usStackHighWaterMark);
    }
    vPortFree(task_status);
    vTaskDelay(1000);
  }
}

/* Exported function ---------------------------------------------------------*/
uint32_t get_10us() { return htim2.Instance->CNT; }

/* Static and callback function ----------------------------------------------*/
// glibc callback function for printf
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

// freertos callback functions for runtime stats
void configureTimerForRunTimeStats(void) { HAL_TIM_Base_Start_IT(&htim2); }
unsigned long getRunTimeCounterValue(void) { return get_10us(); }

// freertos callback function when task stack overflowed
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
  __disable_irq();
  // if under debugging
  if ((CoreDebug->DHCSR & 0x1) == 0x1) {
    __asm volatile("BKPT #0");
  } else {
    printf("Stack overflowed for %s\n", pcTaskName);

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    while (1) {
    }
  }
}

// stm32_module callback function when assertion failed
void __module_assert_fail(const char *assertion, const char *file,
                          unsigned int line, const char *function) {
  __disable_irq();
  // if under debugging
  if ((CoreDebug->DHCSR & 0x1) == 0x1) {
    __asm volatile("BKPT #0");
  } else {
    printf("%s:%u: %s: STM32 module assertion `%s' failed.\n", file, line,
           function, assertion);

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    while (1) {
    }
  }
}
