/**
 * @file pedals.c
 * @author MinLun Tsou (astatine1184@gmail.com)
 * @brief the controller function for the 2 pedals, APPS and BSE
 * @version 0.1
 * @date 2023-05-29
 * 
 * @copyright Copyright (c) 2023
 * 
 * There are 3 foot pedal sensors: 2 APPS and 1 BSE
 * The relative rules for APPS and BSE are in T.4
 * 
 * analog sensors are used for the APPS, and the numbers are read by an internal ADC.
 * The numbers are linearly transformed into 0~1 with a positive slope and offset.
 * The normalized numbers are then passed to the other tasks.
 * 
 * There are a few safety checks done by this tasks before normalizing the readings
 * If the sensor reading go out of bounds, error is reported
 * If the 2 APPS deviate for more than 10 %, error is reported
 * 
 * This function is executed periodically, and the process is:
 * 1. calls the ADC and DMA to read the numbers automatically
 * 2. check if the numbers are out of bounds
 * 3. (optional) DSP filter the readings
 * 4. normalize the reading to 0 and 1
 * 5. push the new numbers onto a shared resource
 */

// glibc include
#include <stdint.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
// #include "task.h"
#include "semphr.h"

// project include
#include "project_def.h"

/**
 * @brief structure to hold the data acquired by DMA
 * 
 */
typedef struct{
    uint16_t apps1;
    uint16_t apps2;
    uint16_t bse
} adc_dma_buffer_t;
static adc_dma_buffer_t adc_dma_buffer = {0} ;

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
    SemaphoreHandle_t mutex
} pedal_data_t;

/**
 * @brief handler function for the data acquisition task of the pedal sensors
 * @note this function is to be executed periodically
 * 
 */
void pedal_handler() {
    
    //TODO: set the conversion mode for the ADC to not blow up the buffers accidentally
    HAL_ADC_Start_DMA(&hadc1, &(adc_dma_buffer.apps1), 1);
    HAL_ADC_Start_DMA(&hadc3, &(adc_dma_buffer.apps2), 2);

    uint8_t micro_apps = (uint8_t)HAL_GPIO_ReadPin(MICRO_APPS_PORT, MICRO_APPS_PIN);
    uint8_t micro_bse = (uint8_t)HAL_GPIO_ReadPin(MICRO_BSE_PORT, MICRO_BSE_PORT);

    return;
}