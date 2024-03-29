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
#include "dashboard_controller.h"
#include "front_box_can.h"
#include "project_def.h"
#include "sensors.h"
#include "status_controller.h"
#include "tests.h"
#include "torque_controller.h"

/* Exported variable ---------------------------------------------------------*/
// stm32_module
__dtcmram ButtonMonitor button_monitor;
__dtcmram ErrorHandler error_handler;
__dtcmram LedController led_controller;

// project
__dtcmram DashboardController dashboard_controller;
__dtcmram FrontBoxCan front_box_can;
__dtcmram StatusController status_controller;
__dtcmram TorqueController torque_controller;

__dtcmram TaskHandle_t freertos_stats_task_handle;

/* Static variable -----------------------------------------------------------*/
// stm32_module
static __dtcmram struct button_cb button_cb[NUM_BUTTON];
static __dtcmram struct error_callback_cb auxiliary_error_callback_cb;
static __dtcmram struct led_cb led_cb[NUM_LED];

#ifndef PRODUCTION
// project
static __dtcmram StaticTask_t freertos_stats_task_cb;
static __dtcmram uint32_t
    freertos_stats_task_buffer[FREERTOS_STATS_TASK_STACK_SIZE];
#endif  // PRODUCTION

/* Static function prototype -------------------------------------------------*/
/**
 * @brief Function to initialize button module.
 *
 * @return None.
 */
static void button_module_init();

/**
 * @brief Function to initialize led module.
 *
 * @return None.
 */
static void led_module_init();

/**
 * @brief Function to augment the function of error handler when error occurred.
 *
 * @param[in] argument Not used.
 * @param[in] error_code The error code.
 * @return None.
 */
static void auxiliary_error_handler(void *const argument, uint32_t error_code);

/* Entry point ---------------------------------------------------------------*/
void user_init() {
  // light up vcu light when initializing
  HAL_GPIO_WritePin(LED_VCU_GPIO_Port, LED_VCU_Pin, GPIO_PIN_SET);

  // stm32_module
  button_module_init();
  ErrorHandler_ctor(&error_handler);
  ErrorHandler_start(&error_handler);
  led_module_init();

// project
#ifndef TESTING
  DashboardController_ctor(&dashboard_controller);
  DashboardController_start(&dashboard_controller);
  FrontBoxCan_ctor(&front_box_can, &hfdcan3);
  FrontBoxCan_start(&front_box_can);
  StatusController_ctor(&status_controller);
  StatusController_start(&status_controller);
  TorqueController_ctor(&torque_controller);
  TorqueController_start(&torque_controller);

  sensor_init();

  // register error callback function
  ErrorHandler_add_error_callback(&error_handler, &auxiliary_error_callback_cb,
                                  auxiliary_error_handler, NULL,
                                  ERROR_CODE_ALL);
#endif  // TESTING

// test
#ifdef LED_TEST
  xTaskCreate(led_test, "led_test_task", configMINIMAL_STACK_SIZE, NULL,
              TaskPriorityNormal, NULL);
#endif  // LED_TEST

#ifndef PRODUCTION
  // start freertos stats task for monitoring freertos states
  freertos_stats_task_handle = xTaskCreateStatic(
      freertos_stats_task, "freertos_stats_task",
      FREERTOS_STATS_TASK_STACK_SIZE, NULL, TaskPriorityLowest,
      freertos_stats_task_buffer, &freertos_stats_task_cb);
#endif  // PRODUCTION
}

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

/* Static function -----------------------------------------------------------*/
static void button_module_init() {
  ButtonMonitor_ctor(&button_monitor);

  // should be in the same order as taht in project_def.h
  ButtonMonitor_add_button(&button_monitor, &button_cb[BUTTON_BUILTIN],
                           BUTTON_BUILTIN_GPIO_Port, BUTTON_BUILTIN_Pin);
  ButtonMonitor_add_button(&button_monitor, &button_cb[BUTTON_RTD],
                           BUTTON_RTD_GPIO_Port, BUTTON_RTD_Pin);
  ButtonMonitor_add_button(&button_monitor, &button_cb[GEAR_HIGH],
                           GEAR_HIGH_GPIO_Port, GEAR_HIGH_Pin);
  ButtonMonitor_add_button(&button_monitor, &button_cb[GEAR_REVERSE],
                           GEAR_REVERSE_GPIO_Port, GEAR_REVERSE_Pin);
  ButtonMonitor_add_button(&button_monitor, &button_cb[MICRO_APPS],
                           MICRO_APPS_GPIO_Port, MICRO_APPS_Pin);
  ButtonMonitor_add_button(&button_monitor, &button_cb[MICRO_BSE],
                           MICRO_BSE_GPIO_Port, MICRO_BSE_Pin);

#ifndef TESTING
  ButtonMonitor_register_callback(&button_monitor, GEAR_HIGH,
                                  &TorqueController_gear_high_button_callback,
                                  (void *)&torque_controller);
  ButtonMonitor_register_callback(
      &button_monitor, GEAR_REVERSE,
      &TorqueController_gear_reverse_button_callback,
      (void *)&torque_controller);
#endif  // TESTING

#ifdef BUTTON_TEST
  ButtonMonitor_register_callback(&button_monitor, BUTTON_BUILTIN,
                                  &button_test_button_callback,
                                  (void *)BUTTON_BUILTIN);
  ButtonMonitor_register_callback(&button_monitor, BUTTON_RTD,
                                  &button_test_button_callback,
                                  (void *)BUTTON_RTD);
  ButtonMonitor_register_callback(&button_monitor, GEAR_HIGH,
                                  &button_test_button_callback,
                                  (void *)GEAR_HIGH);
  ButtonMonitor_register_callback(&button_monitor, GEAR_REVERSE,
                                  &button_test_button_callback,
                                  (void *)GEAR_REVERSE);
  ButtonMonitor_register_callback(&button_monitor, MICRO_APPS,
                                  &button_test_button_callback,
                                  (void *)MICRO_APPS);
  ButtonMonitor_register_callback(&button_monitor, MICRO_BSE,
                                  &button_test_button_callback,
                                  (void *)MICRO_BSE);

#endif  // BUTTON_TEST

  ButtonMonitor_start(&button_monitor);
}

static void led_module_init() {
  LedController_ctor(&led_controller);

  // should be in the same order as taht in project_def.h
  LedController_add_led(&led_controller, &led_cb[LED_BUILTIN_GREEN],
                        LED_BUILTIN_GREEN_GPIO_Port, LED_BUILTIN_GREEN_Pin);
  LedController_add_led(&led_controller, &led_cb[LED_BUILTIN_YELLOW],
                        LED_BUILTIN_YELLOW_GPIO_Port, LED_BUILTIN_YELLOW_Pin);
  LedController_add_led(&led_controller, &led_cb[LED_BUILTIN_RED],
                        LED_BUILTIN_RED_GPIO_Port, LED_BUILTIN_RED_Pin);
  LedController_add_led(&led_controller, &led_cb[LED_WARN], LED_WARN_GPIO_Port,
                        LED_WARN_Pin);
  LedController_add_led(&led_controller, &led_cb[LED_ERROR],
                        LED_ERROR_GPIO_Port, LED_ERROR_Pin);
  LedController_add_led(&led_controller, &led_cb[LED_CAN_TX],
                        LED_CAN_TX_GPIO_Port, LED_CAN_TX_Pin);
  LedController_add_led(&led_controller, &led_cb[LED_CAN_RX],
                        LED_CAN_RX_GPIO_Port, LED_CAN_RX_Pin);
  LedController_add_led(&led_controller, &led_cb[LED_VCU], LED_VCU_GPIO_Port,
                        LED_VCU_Pin);
  LedController_add_led(&led_controller, &led_cb[LED_RTD], LED_RTD_GPIO_Port,
                        LED_RTD_Pin);
  LedController_add_led(&led_controller, &led_cb[LED_GEAR], LED_GEAR_GPIO_Port,
                        LED_GEAR_Pin);
  LedController_add_led(&led_controller, &led_cb[SIREN_RTD],
                        SIREN_RTD_GPIO_Port, SIREN_RTD_Pin);

  LedController_start(&led_controller);

  // turn on vcu light
  LedController_turn_on(&led_controller, LED_VCU);
}

static void auxiliary_error_handler(void *const argument, uint32_t error_code) {
  (void)argument;

  // write error code to can frame
  uint32_t full_error_code;
  ErrorHandler_get_error(&error_handler, &full_error_code);
  xSemaphoreTake(can_vcu_tx_mutex, portMAX_DELAY);
  can_vcu_tx.VCU_Status.VCU_Error_Code = full_error_code;
  xSemaphoreGive(can_vcu_tx_mutex);

  // blink led
  if (error_code & ERROR_SET) {
    LedController_blink(&led_controller, LED_ERROR, 3000);
  }
}

/* Callback function ---------------------------------------------------------*/
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
  printf("Stack overflowed for %s\n", pcTaskName);
  Error_Handler();
}

// stm32_module callback function when assertion failed
void __module_assert_fail(const char *assertion, const char *file,
                          unsigned int line, const char *function) {
  printf("%s:%u: %s: STM32 module assertion `%s' failed.\n", file, line,
         function, assertion);
  Error_Handler();
}
