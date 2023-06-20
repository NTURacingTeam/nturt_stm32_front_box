#ifndef _SENSORS_H
#define _SENSORS_H

#include <stdint.h>
#include "semphr.h"

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

/*task controls*/
void sensor_handler(void*);
extern uint32_t sensors_data_task_buffer[SENSOR_DATA_TASK_STACK_SIZE];
extern StaticTask_t sensors_data_task_cb;
extern TaskHandle_t sensors_data_task_handle;

#endif //_SENSORS_H