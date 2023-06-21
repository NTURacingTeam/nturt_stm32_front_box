#ifndef _SENSORS_H
#define _SENSORS_H

#include <stdint.h>
#include "semphr.h"
#include "timers.h"

/**
 * @brief structure to hold the data that is outputed by this function
 * 
 */
typedef struct {
    float apps1;
    float apps2;
    float bse;
    uint8_t micro_apps;
    uint8_t micro_bse;
    SemaphoreHandle_t mutex;
} pedal_data_t;

extern pedal_data_t pedal;

typedef struct {
    uint16_t left;
    uint16_t right;
    SemaphoreHandle_t mutex;
} travel_data_t;

extern travel_data_t travel_sensor;

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

#endif //_SENSORS_H