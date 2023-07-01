#include "tests.h"

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// project include
#include "project_def.h"
#include "user_main.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

void led_test(void *argument) {
  (void)argument;

  while (true) {
    for (int i = 0; i < NUM_LED; i++) {
      LedController_blink(&led_controller, i, 1000);
      vTaskDelay(1000);
    }
  }
}

void button_test_button_callback(void *_argument, const GPIO_PinState state) {
  int argument = (int)_argument;

  switch (argument) {
    case BUTTON_BUILTIN:
      state == GPIO_PIN_SET
          ? LedController_turn_on(&led_controller, LED_BUILTIN_GREEN)
          : LedController_turn_off(&led_controller, LED_BUILTIN_GREEN);
      break;

    case BUTTON_RTD:
      state == GPIO_PIN_SET ? LedController_turn_on(&led_controller, LED_RTD)
                            : LedController_turn_off(&led_controller, LED_RTD);
      break;

    case GEAR_HIGH:
      state == GPIO_PIN_SET ? LedController_turn_on(&led_controller, LED_GEAR)
                            : LedController_turn_off(&led_controller, LED_GEAR);
      break;

    case GEAR_REVERSE:
      state == GPIO_PIN_SET ? LedController_turn_on(&led_controller, LED_VCU)
                            : LedController_turn_off(&led_controller, LED_VCU);
      break;

    case MICRO_APPS:
      state == GPIO_PIN_SET
          ? LedController_turn_on(&led_controller, LED_BUILTIN_YELLOW)
          : LedController_turn_off(&led_controller, LED_BUILTIN_YELLOW);
      break;

    case MICRO_BSE:
      state == GPIO_PIN_SET
          ? LedController_turn_on(&led_controller, LED_BUILTIN_RED)
          : LedController_turn_off(&led_controller, LED_BUILTIN_RED);
      break;
  }
}
