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

#define MUTEX_TIMEOUT 0x02
#define ADC_TIMEOUT 0x02

#define FLAG_ADC1_FINISH 0b10
#define FLAG_ADC3_FINISH 0b100

//private functions
static inline float APPS1_transfer_function(uint16_t reading);
static inline float APPS2_transfer_function (uint16_t reading);
static inline float BSE_transfer_function(uint16_t reading);
#define OUT_OF_BOUNDS_MARGIN 0.05

/**
 * @brief structure to hold the data acquired by DMA
 * 
 */
typedef struct{
    uint16_t apps1;
    uint16_t apps2;
    uint16_t bse
} adc_dma_buffer_t;

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
    adc_dma_buffer_t adc_dma_buffer = {0};
    pedal_data_t pedal = {
        .apps1 = 0.0,
        .apps2 = 0.0,
        .bse = 0.0,
        .micro_apps = 1,
        .micro_bse = 1
        //TODO: set mutex
        // .mutex = 
    };

    while(1) {
                
        //TODO: set the conversion mode for the ADC to not blow up the buffers accidentally
        HAL_ADC_Start_DMA(&hadc1, &(adc_dma_buffer.apps1), 1);
        HAL_ADC_Start_DMA(&hadc3, &(adc_dma_buffer.apps2), 2);

        uint8_t micro_apps = (uint8_t)HAL_GPIO_ReadPin(MICRO_APPS_PORT, MICRO_APPS_PIN);
        uint8_t micro_bse = (uint8_t)HAL_GPIO_ReadPin(MICRO_BSE_PORT, MICRO_BSE_PORT);

        xSemaphoreTake(pedal.mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT));
            pedal.micro_apps = micro_apps;
            pedal.micro_bse = micro_bse;
        xSemaphoreGive(pedal.mutex);

        //wait for both flags to be set
        {    
            uint32_t wait_flag = 0U;
            TickType_t t0 = xTaskGetTickCount();
            //if either of which is not set
            do {
                BaseType_t Wait_result = xTaskNotifyWait(0, 0, &wait_flag, pdMS_TO_TICKS(ADC_TIMEOUT));
                if(xTaskGetTickCount() - t0 >= ADC_TIMEOUT || Wait_result == pdFALSE) {
                    //TODO: report error
                }
            } while(~wait_flag & (FLAG_ADC1_FINISH | FLAG_ADC3_FINISH));
            ulTaskNotifyValueClear(NULL, (FLAG_ADC1_FINISH | FLAG_ADC3_FINISH));
        }

        //TODO: safety checks
        {
            float apps1 = APPS1_transfer_function(adc_dma_buffer.apps1);
            if(apps1 > 1 || apps1 < 0); //TODO: report error
            float apps2 = APPS2_transfer_function(adc_dma_buffer.apps2);
            if(apps2 > 1 || apps2 < 0); //TODO: report error
            if(apps1-apps2 > 0.1 || apps2-apps1 > 0.1); //TODO: report error
            
            
            float bse = BSE_transfer_function(adc_dma_buffer.bse);
            if(bse > 1 || bse < 0); //TODO: report error
        }
    }
}

/**
 * @brief transfer function for the first APPS
 * 
 * @param reading the 12 bit value read from the ADC
 * @return float the normalized read value spanning from 0 to 1
 * 
 * The detailed description about the transfer function can be found here:
 * https://hackmd.io/@nturacing/ByOF6I5T9/%2F2Jgh0ieyS0mc_r-6pHKQyQ
 */
static inline float APPS1_transfer_function(uint16_t reading) {
    float buf = (reading-860)/(3891-860);
    if(buf < 0 && buf > -(OUT_OF_BOUNDS_MARGIN)) buf = 0;
    if(buf > 1 && buf > OUT_OF_BOUNDS_MARGIN) buf = 1;
    return buf;
}

static inline float APPS2_transfer_function (uint16_t reading) {
    float buf = (reading*2-860)/(3891-860);
    if(buf < 0 && buf > -(OUT_OF_BOUNDS_MARGIN)) buf = 0;
    if(buf > 1 && buf > OUT_OF_BOUNDS_MARGIN) buf = 1;
    return buf;
}

static inline float BSE_transfer_function(uint16_t reading) {
    float buf = (reading-860)/(3891-860);
    if(buf < 0 && buf > -(OUT_OF_BOUNDS_MARGIN)) buf = 0;
    if(buf > 1 && buf > OUT_OF_BOUNDS_MARGIN) buf = 1;
    return buf;
}