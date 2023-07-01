#ifndef _SENSORS_H
#define _SENSORS_H

#include <stdbool.h>
#include <stdint.h>

#include "semphr.h"
#include "timers.h"

/*macro: freertos object parameter*/
#define SENSOR_DATA_TASK_STACK_SIZE 256

// freertos timer info
#define SENSOR_TIMER_PERIOD 5  // in ms
#define TIRE_TEMP_PERIOD 250   // in ms

/**
 * @brief structure to hold the data that is outputed by this function
 *
 */
typedef struct {
  float apps1;
  float apps2;
  float bse;
  bool micro_apps;
  bool micro_bse;
  SemaphoreHandle_t mutex;
} pedal_data_t;

extern pedal_data_t pedal;

typedef struct {
  float left;
  float right;
  uint16_t strain;
  float oil_pressure;
  SemaphoreHandle_t mutex;
} travel_strain_data_t;

extern travel_strain_data_t travel_strain_oil_sensor;

typedef struct {
  float left[8];
  float right[8];
  SemaphoreHandle_t mutex;
} tire_temp_data_t;

extern tire_temp_data_t tire_temp_sensor;

/*task controls*/
void sensor_handler(void*);
extern uint32_t sensors_data_task_buffer[SENSOR_DATA_TASK_STACK_SIZE];
extern StaticTask_t sensors_data_task_cb;
extern TaskHandle_t sensors_data_task_handle;

/*sensor timer*/
void sensor_timer_callback(TimerHandle_t);
extern StaticTimer_t sensor_timer_buffer;
extern TimerHandle_t sensor_timer_handle;

#endif  //_SENSORS_H
