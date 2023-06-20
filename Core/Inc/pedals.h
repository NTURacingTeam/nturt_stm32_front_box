#ifndef _PEDALS_H
#define _PEDALS_H

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

#endif