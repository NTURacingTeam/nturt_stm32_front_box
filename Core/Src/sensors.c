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

//module include
#include "stm32_module/stm32_module.h"

//own include
#include "sensors.h"

#define MUTEX_TIMEOUT 0x02
#define ADC_TIMEOUT 0x02

#define FLAG_ADC1_FINISH 0x2
#define FLAG_ADC3_FINISH 0x4
#define FLAG_READ_PEDAL 0x1000
#define FLAG_READ_TIRE_TEMP 0x2000
#define FLAG_READ_SUS 0x4000

//private functions
static BaseType_t wait_for_notif_flags(uint32_t target, uint32_t timeout, uint32_t* const gotten);
static inline float APPS1_transfer_function(uint16_t reading);
static inline float APPS2_transfer_function (uint16_t reading);
static inline float BSE_transfer_function(uint16_t reading);
#define OUT_OF_BOUNDS_MARGIN 0.05

/**
 * @brief structure to hold the data acquired by DMA
 * 
 */
typedef struct{
    //TODO: organize the buffer space and the peripheral settings since other sensors uses ADC as well
    uint16_t apps1; //ADC12 rank1
    uint16_t travel_r; //ADC12 rank2
    uint16_t apps2; //ADC3 rank1
    uint16_t bse; //ADC3 rank2
    uint16_t travel_l //ADC3 rank3
} adc_dma_buffer_t;

/**
 * @brief global singleton resource that stores the current state of the pedal sensors
 * 
 */
pedal_data_t pedal = {
    .apps1 = 0.0,
    .apps2 = 0.0,
    .bse = 0.0,
    .micro_apps = 1,
    .micro_bse = 1
    //mutex is intitialized in user_main.c along with everything freertos
};

/*task controls*/
__dtcmram uint32_t sensors_data_task_buffer[SENSOR_DATA_TASK_STACK_SIZE];
__dtcmram StaticTask_t sensors_data_task_cb;
TaskHandle_t sensors_data_task_handle;

/*timer controls*/
__dtcmram StaticTimer_t sensor_timer_buffer;
TimerHandle_t sensor_timer_handle;

void sensor_timer_callback(TimerHandle_t timer) {
    xTaskNotify(sensors_data_task_handle, FLAG_READ_PEDAL, eSetBits);
}


/**
 * @brief handler function for the data acquisition task of the pedal sensors
 * 
 */
void sensor_handler(void* argument) {
    //TODO: handle every return status of FreeRTOS and HAL API

    (void)argument;

    adc_dma_buffer_t adc_dma_buffer = {0};
    
    uint32_t pending_notifications = 0U;

    while(1) {
        if(!pending_notifications) {
            //wait for notifications from timer if there are no pending ones
            (void)xTaskNotifyWait(0, 0xFFFFFFFFUL, &pending_notifications, portMAX_DELAY);
        }

        if(pending_notifications & FLAG_READ_PEDAL) {
            //clear bit
            pending_notifications &= ~FLAG_READ_PEDAL;

            //TODO: set the conversion mode for the ADC to not blow up the buffers accidentally
            HAL_ADC_Start_DMA(&hadc1, &(adc_dma_buffer.apps1), 2);
            HAL_ADC_Start_DMA(&hadc3, &(adc_dma_buffer.apps2), 3);

            uint8_t micro_apps = (uint8_t)HAL_GPIO_ReadPin(MICRO_APPS_PORT, MICRO_APPS_PIN);
            uint8_t micro_bse = (uint8_t)HAL_GPIO_ReadPin(MICRO_BSE_PORT, MICRO_BSE_PORT);

            xSemaphoreTake(pedal.mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT));
                pedal.micro_apps = micro_apps;
                pedal.micro_bse = micro_bse;
            xSemaphoreGive(pedal.mutex);

            //wait for both flags to be set
            if (wait_for_notif_flags(FLAG_ADC1_FINISH | FLAG_ADC3_FINISH, pdMS_TO_TICKS(ADC_TIMEOUT), &pending_notifications) != pdTRUE) {
                //TODO: edge case where only one or neither flags set
                pending_notifications &= ~(FLAG_ADC1_FINISH | FLAG_ADC3_FINISH);
            }

            {
                //TODO: another error handler for implausibility
                //TODO: how to use errorhandler API
                float apps1 = APPS1_transfer_function(adc_dma_buffer.apps1);
                if(apps1 > 1.0 || apps1 < 0.0) ErrorHandler_write_error(&Error_Handler, ERROR_CODE_APPS_IMPLAUSIBILITY, ERROR_SET); 

                float apps2 = APPS2_transfer_function(adc_dma_buffer.apps2);
                if(apps2 > 1.0 || apps2 < 0.0) ErrorHandler_write_error(&Error_Handler, ERROR_CODE_APPS_IMPLAUSIBILITY, ERROR_SET); 

                if(apps1-apps2 > 0.1 || apps2-apps1 > 0.1) ErrorHandler_write_error(&Error_Handler, ERROR_CODE_APPS_IMPLAUSIBILITY, ERROR_SET); 
                
                float bse = BSE_transfer_function(adc_dma_buffer.bse);
                if(bse > 1 || bse < 0); ErrorHandler_write_error(&Error_Handler, ERROR_CODE_BSE_IMPLAUSIBILITY, ERROR_SET); 
                
                xSemaphoreTake(pedal.mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT));
                    pedal.apps1 = apps1;
                    pedal.apps2 = apps2;
                    pedal.bse = bse;
                xSemaphoreGive(pedal.mutex);
            }
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc == &hadc1) {
        xTaskNotifyFromISR(sensors_data_task_handle, FLAG_ADC1_FINISH, eSetBits, NULL);
    } else if(hadc == &hadc3) {
        xTaskNotifyFromISR(sensors_data_task_handle, FLAG_ADC3_FINISH, eSetBits, NULL);
    }
}

/**
 * @brief wrapper function to wait for specific bits in the task notification
 * @param target the target that needs to be waited in ticks
 * @param timeout the timeout
 * @param gotten all the flags that are set. target bits would be autocleared if they are gotten
 * @return the status of the function
 */
BaseType_t wait_for_notif_flags(uint32_t target, uint32_t timeout, uint32_t* const gotten) {    
    uint32_t flag_buf = 0U;
    uint32_t flag_gotten = 0U;
    const TickType_t t0 = xTaskGetTickCount();
    TickType_t tlast = t0;
    //if either of which is not set
    do {
        BaseType_t Wait_result = xTaskNotifyWait(0, 0xFFFFFFFFUL, &flag_buf, timeout-(tlast-t0));
        if(Wait_result == pdFALSE) {
            return pdFALSE;
        }

        flag_gotten |= flag_buf;
        *gotten = flag_gotten;
        tlast = xTaskGetTickCount();
    } while(~flag_gotten & target);

    //remove the gotten flags if the flags are correctly received
    *gotten &= ~(target);
    return pdTRUE;
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