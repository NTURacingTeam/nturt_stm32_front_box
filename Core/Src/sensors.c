/**
 * @file sensors.c
 * @author MinLun Tsou (astatine1184@gmail.com)
 * @brief the controller function for the 2 pedals, APPS and BSE and every other sensors
 * @version 0.2
 * @date 2023-06-21
 * 
 * @copyright Copyright (c) 2023
 * 
 * TODO: update the explanation for other sensors
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
#include "user_main.h"

// freertos include
#include "FreeRTOS.h"
// #include "task.h"
#include "semphr.h"
#include "timers.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

//own include
#include "sensors.h"

#define MUTEX_TIMEOUT 0x02
#define ADC_TIMEOUT 0x02
#define I2C_TIMEOUT 0xFF

#define FLAG_ADC1_FINISH 0x10
#define FLAG_ADC3_FINISH 0x20
#define FLAG_I2C5_FINISH 0x40
#define FLAG_I2C1_FINISH 0x80
#define FLAG_READ_SUS_PEDAL 0x1000
#define FLAG_READ_TIRE_TEMP 0x2000
#define FLAG_D6T_STARTUP 0x100000

#define NUM_FILTER 3
#define MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE 20

MovingAverageFilter moving_average_filter[NUM_FILTER];
/* static variable -----------------------------------------------------------*/
// filter
static float moving_average_buffer[NUM_FILTER] [MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE];

/**
 * @brief structure to hold the data acquired by DMA
 * 
 */
typedef struct{
    uint16_t apps1; //ADC12 rank1
    uint16_t travel_r; //ADC12 rank2
    uint16_t strain; //ADC12 rank3
    uint16_t oil; //ADC12 rank4
    uint16_t apps2; //ADC3 rank1
    uint16_t bse; //ADC3 rank2
    uint16_t travel_l; //ADC3 rank3
} adc_dma_buffer_t;

/**
 * @brief structure to hold the set of data that is present in the transmission of i2c data with D6T sensors
 * @note see https://omronfs.omron.com/en_US/ecb/products/pdf/en_D6T_users_manual.pdf
 * @note the position of the members is important in memory. pls don't move them around.
 * 
 */
typedef struct {
    uint8_t addr_write;     //< address of D6T sensor that is left shifted once. Also the starting address of CRC calculation.
    uint8_t command;        //< command that is senst to the D6T sensor
    uint8_t addr_read;      //< address of D6T sensor that is left shited once then added 1. sent after Restarted signal.
    struct {
        uint8_t low;        //< the low byte of the sensor comes first before the high byte in the 16bit data.
        uint8_t high;
    }PTAT;                  //< the temperature of the sensor itself. Also the starting address of the Rx data buffer.        
    struct {
        uint8_t low;
        uint8_t high;    
    } temp[8];              //< the temperature of the 8 different measurement channels. low byte is sent first.
    uint8_t PEC;            //< packet error check code. Is to be compared with the CRC-8 result of previous 21 bytes.
} i2c_d6t_dma_buffer_t;


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

travel_strain_data_t travel_strain_oil_sensor = {
    .left = 0.0,
    .right = 0.0,
    .strain = 0,
    .oil_pressure = 0.0
    //mutex is initialized in user_main.c along with everything freertos
};

tire_temp_data_t tire_temp_sensor = {
    .left = {0.0},
    .right = {0.0}
    //mutex is initialized in user_main.c along with everything freertos
};

/*task controls*/
// __dtcmram 
uint32_t sensors_data_task_buffer[SENSOR_DATA_TASK_STACK_SIZE];
// __dtcmram 
StaticTask_t sensors_data_task_cb;
TaskHandle_t sensors_data_task_handle;

/*timer controls*/
__dtcmram StaticTimer_t sensor_timer_buffer;
TimerHandle_t sensor_timer_handle;

void filter_init() {
  for (int i = 0; i < NUM_FILTER; i++) {
    MovingAverageFilter_ctor(&moving_average_filter[i], moving_average_buffer[i],
                             MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE, NULL);
  }
}

//private functions
static BaseType_t wait_for_notif_flags(uint32_t target, uint32_t timeout, uint32_t* const gotten);
static inline float APPS1_transfer_function(const uint16_t reading, const float);
static inline float APPS2_transfer_function (const uint16_t reading, const float);
static inline float BSE_transfer_function(const uint16_t reading, float);
static inline float tire_temp_transfer_function(const uint8_t high, const uint8_t low);
static inline float oil_transfer_function(const uint16_t reading);
static inline float travel_transfer_function (const uint16_t reading);
#define START_TO_OUTPUT_MARGIN 0.007
#define OUT_OF_BOUNDS_MARGIN 0.05 //TODO: where do we put these fuzzy bound constants
float fuzzy_edge_remover(const float raw, const float highEdge, const float lowEdge);

static void init_D6T(I2C_HandleTypeDef* const hi2c, volatile i2c_d6t_dma_buffer_t* rawData, uint32_t txThreadFlag, uint32_t* otherflags);

void sensor_timer_callback(TimerHandle_t timer) {
    xTaskNotify(sensors_data_task_handle, FLAG_READ_SUS_PEDAL, eSetBits);

    // we use the timer ID to secretly count how many times have the timer expired
    // and update the tire temp data with also a fix interval
    uint32_t expire_count = (uint32_t)pvTimerGetTimerID(timer);
    expire_count++;
    vTimerSetTimerID(timer, (void*)expire_count);

    if((uint32_t)pvTimerGetTimerID(timer) >= TIRE_TEMP_PERIOD/SENSOR_TIMER_PERIOD) {
        xTaskNotify(sensors_data_task_handle, FLAG_READ_TIRE_TEMP, eSetBits);
        vTimerSetTimerID(timer, (void*)0);
    }
}

/**
 * @brief handler function for the data acquisition task of the pedal sensors
 * 
 */
void sensor_handler(void* argument) {
    //dma buffer zone
    static __dma_buffer adc_dma_buffer_t adc_dma_buffer = {0};
    static __dma_buffer i2c_d6t_dma_buffer_t d6t_dma_buffer_R = {0};
    static __dma_buffer i2c_d6t_dma_buffer_t d6t_dma_buffer_L = {0};

    //TODO: handle every return status of FreeRTOS and HAL API
    (void)argument;
    //variable to store the pending flags that is sent from xTaskNotify
    uint32_t pending_notifications = 0U;

    /*initialize D6T sensors*/
    //first wait for 20ms for the sensors to boot up
    vTaskDelay(pdMS_TO_TICKS(20));
    // init_D6T(&hi2c5, &d6t_dma_buffer_R, FLAG_D6T_STARTUP, &pending_notifications);
    // init_D6T(&hi2c1, &d6t_dma_buffer_L, FLAG_D6T_STARTUP, &pending_notifications);
    //wait for 500ms after initialization before starting to query the sensors
    vTaskDelay(pdMS_TO_TICKS(500));

    /*initialize the pedal sensors' compensation*/
    //wait until the pedals are set back to zero
    while(  HAL_GPIO_ReadPin(MICRO_APPS_GPIO_Port, MICRO_APPS_Pin) != GPIO_PIN_RESET ||
            HAL_GPIO_ReadPin(MICRO_BSE_GPIO_Port, MICRO_BSE_Pin) != GPIO_PIN_RESET ) {
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&(adc_dma_buffer.apps1), 4);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&(adc_dma_buffer.apps2), 3);
    //wait for both flags to be set
    wait_for_notif_flags(FLAG_ADC1_FINISH | FLAG_ADC3_FINISH, portMAX_DELAY, &pending_notifications);
    const float apps1_compensation = - APPS1_transfer_function(adc_dma_buffer.apps1, 0);
    const float apps2_compensation = - APPS2_transfer_function(adc_dma_buffer.apps2, 0);
    const float bse_compensation = - BSE_transfer_function(adc_dma_buffer.bse, 0);
    
    //start the initial read, and start the timer that should later notifies this task to do stuff again
    xTaskNotify(xTaskGetCurrentTaskHandle(), FLAG_READ_SUS_PEDAL | FLAG_READ_TIRE_TEMP, eSetBits);
    xTimerStart(sensor_timer_handle, portMAX_DELAY); //TODO: case where timer did not start

    while(1) {
        if(!pending_notifications) {
            //wait for notifications from timer if there are no pending ones
            (void)xTaskNotifyWait(0, 0xFFFFFFFFUL, &pending_notifications, portMAX_DELAY);
        }

        if(pending_notifications & FLAG_READ_SUS_PEDAL) {
            pending_notifications &= ~FLAG_READ_SUS_PEDAL; //clear bit

            HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&(adc_dma_buffer.apps1), 4);
            HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&(adc_dma_buffer.apps2), 3);

            const uint8_t micro_apps = (uint8_t)HAL_GPIO_ReadPin(MICRO_APPS_GPIO_Port, MICRO_APPS_Pin);
            const uint8_t micro_bse = (uint8_t)HAL_GPIO_ReadPin(MICRO_BSE_GPIO_Port, MICRO_BSE_Pin);

            xSemaphoreTake(pedal.mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT));
                pedal.micro_apps = micro_apps;
                pedal.micro_bse = micro_bse;
            xSemaphoreGive(pedal.mutex);

            //wait for both flags to be set
            if (wait_for_notif_flags(FLAG_ADC1_FINISH | FLAG_ADC3_FINISH, pdMS_TO_TICKS(ADC_TIMEOUT), &pending_notifications) != pdTRUE) {
                //TODO: edge case where only one or neither flags set
                pending_notifications &= ~(FLAG_ADC1_FINISH | FLAG_ADC3_FINISH);
            }

        
            //TODO: another error handler for implausibility
            //TODO: how to use errorhandler API
            float apps1_filtered, apps2_filtered, bse_filtered;
            MovingAverageFilter_update(&moving_average_filter[0], APPS1_transfer_function(adc_dma_buffer.apps1, apps1_compensation), &apps1_filtered);
            MovingAverageFilter_update(&moving_average_filter[1], APPS2_transfer_function(adc_dma_buffer.apps2, apps2_compensation), &apps2_filtered);
            MovingAverageFilter_update(&moving_average_filter[2], BSE_transfer_function(adc_dma_buffer.bse, bse_compensation), &bse_filtered);
            const float apps1 = fuzzy_edge_remover(apps1_filtered, 1.0, 0.0);
            const float apps2 = fuzzy_edge_remover(apps2_filtered, 1.0, 0.0);
            const float bse = fuzzy_edge_remover(bse_filtered, 1.0, 0.0);

            uint32_t prevErr = 0;
            ErrorHandler_get_error(&error_handler, &prevErr);
            const uint32_t isAppsErrSet = prevErr & ERROR_CODE_APPS_IMPLAUSIBILITY;
            const uint32_t isBseErrSet = prevErr & ERROR_CODE_BSE_IMPLAUSIBILITY;

            if( apps1 > 1.0 || apps1 < 0.0 || apps2 > 1.0 || apps2 < 0.0 || apps1-apps2 > 0.1 || apps2-apps1 > 0.1 ) {
                if(!isAppsErrSet) ErrorHandler_write_error(&error_handler, ERROR_CODE_APPS_IMPLAUSIBILITY, ERROR_SET);
            }
            else if(isAppsErrSet){
                ErrorHandler_write_error(&error_handler, ERROR_CODE_APPS_IMPLAUSIBILITY, ERROR_CLEAR);
            }
            
            if(bse > 1.0 || bse < 0.0) {
                if(!isBseErrSet) ErrorHandler_write_error(&error_handler, ERROR_CODE_BSE_IMPLAUSIBILITY, ERROR_SET);
            }
            else if(isBseErrSet) {
                ErrorHandler_write_error(&error_handler, ERROR_CODE_BSE_IMPLAUSIBILITY, ERROR_CLEAR);
            }
            
            xSemaphoreTake(pedal.mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT));
                pedal.apps1 = apps1;
                pedal.apps2 = apps2;
                pedal.bse = bse;
            xSemaphoreGive(pedal.mutex);
        
        
            //update the travel sensor's value
            xSemaphoreTake(travel_strain_oil_sensor.mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT));
                travel_strain_oil_sensor.left = travel_transfer_function(adc_dma_buffer.travel_l);
                travel_strain_oil_sensor.right = travel_transfer_function(adc_dma_buffer.travel_r);
                travel_strain_oil_sensor.strain = adc_dma_buffer.strain;
                travel_strain_oil_sensor.oil_pressure = oil_transfer_function(adc_dma_buffer.oil);
            xSemaphoreGive(travel_strain_oil_sensor.mutex);
        
        }
        if(pending_notifications & FLAG_READ_TIRE_TEMP) {
            pending_notifications &= ~FLAG_READ_TIRE_TEMP;
            //read the values from both sensors
            // HAL_I2C_Mem_Read_DMA(
            //     &hi2c5,
            //     d6t_dma_buffer_R.addr_write, 
            //     d6t_dma_buffer_R.command, 
            //     1, 
            //     &(d6t_dma_buffer_R.PTAT.low), 
            //     sizeof(d6t_dma_buffer_R.PTAT)+sizeof(d6t_dma_buffer_R.temp)+sizeof(d6t_dma_buffer_R.PEC));
            // HAL_I2C_Mem_Read_DMA(
            //     &hi2c1, 
            //     d6t_dma_buffer_L.addr_write, 
            //     d6t_dma_buffer_L.command, 
            //     1, 
            //     &(d6t_dma_buffer_L.PTAT.low), 
            //     sizeof(d6t_dma_buffer_L.PTAT)+sizeof(d6t_dma_buffer_L.temp)+sizeof(d6t_dma_buffer_L.PEC));
            //wait for the DMA to finish, while we can do other stuff in the mean time
            //TODO: setup timeout exception and deal with error case where the stuff did not finish
        }
        if(pending_notifications & FLAG_I2C1_FINISH) {
            pending_notifications &= ~FLAG_I2C1_FINISH; //clear flags

            //TODO: CRC the data

            xSemaphoreTake(tire_temp_sensor.mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT));
                for(int i=0; i<sizeof(d6t_dma_buffer_L.temp)/sizeof(d6t_dma_buffer_L.temp[0]); i++) {
                    tire_temp_sensor.left[i] = tire_temp_transfer_function(d6t_dma_buffer_L.temp[i].high, d6t_dma_buffer_L.temp[i].low);
                }
            xSemaphoreGive(tire_temp_sensor.mutex);
        }
        if(pending_notifications & FLAG_I2C5_FINISH) {
            pending_notifications &= ~FLAG_I2C5_FINISH; //clear flags

            //TODO: CRC the data

            xSemaphoreTake(tire_temp_sensor.mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT));
                for(int i=0; i<sizeof(d6t_dma_buffer_R.temp)/sizeof(d6t_dma_buffer_R.temp[0]); i++) {
                    tire_temp_sensor.right[i] = tire_temp_transfer_function(d6t_dma_buffer_R.temp[i].high, d6t_dma_buffer_R.temp[i].low);
                }
            xSemaphoreGive(tire_temp_sensor.mutex);   
        }
    }
}

/**
 * @brief wrapper function to wait for specific bits in the task notification
 * @param target the target that needs to be waited 
 * @param timeout the timeout in ticks
 * @param gotten all the flags that are set. target bits would be autocleared if they are gotten
 * @return the status of the function
 */
BaseType_t wait_for_notif_flags(uint32_t target, uint32_t timeout, uint32_t* const gotten) {    
    uint32_t flag_buf = 0U;
    uint32_t flag_gotten = *gotten;
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
static inline float APPS1_transfer_function(const uint16_t reading, const float compensation) {
    return (float)(reading-860)/(3891-860) + compensation;
}

static inline float APPS2_transfer_function (const uint16_t reading, const float compensation) {
    return (float)(reading*2-860)/(3891-860) + compensation;
}

static inline float BSE_transfer_function(const uint16_t reading, const float compensation) {
    // since we only use 2.5~24.5mm part of the domain instead of the full 0~25, we have
    return (float)(reading-82)/(3686-82) + compensation;
}

float fuzzy_edge_remover(const float raw, const float highEdge, const float lowEdge) {
    if(raw < lowEdge) {
        if(raw > -(OUT_OF_BOUNDS_MARGIN) + lowEdge) return lowEdge;
        else return raw;
    } 
    else if(raw > highEdge) {
        if(raw < highEdge + OUT_OF_BOUNDS_MARGIN) return highEdge;
        else return raw;
    }
    else {
        if(raw < START_TO_OUTPUT_MARGIN + lowEdge) return lowEdge;
        else return raw;
    }
}

static inline float tire_temp_transfer_function(const uint8_t highByte, const uint8_t lowByte) {
    return (float)((((int16_t)highByte) << 8) + (int16_t)lowByte)/5;
}

static inline float travel_transfer_function (const uint16_t reading) {
    return 75 - (float)reading/4096 * 75;
}

static inline float oil_transfer_function(const uint16_t reading) {
    //see https://www.mouser.tw/datasheet/2/418/8/ENG_DS_MSP300_B1-1130121.pdf
    //extra 3.3/3 is because a voltage divider moved 5V to 3V while max voltage on the system is 3.3V
    return (((float)reading /4096 *3.3/3 *5-1)/4 *15000-1000)/14000 * 70;
}

/**
 * @brief this function initializes the payload data of the i2c addresses and the D6T sensors themselves
 * 
 * @param hi2c the i2c handle that handles the i2c communications
 * @param rawData and array that stores the data to be transmitted and that to be received
 * @param txThreadFlag the task notification flag used to indicate completion of transfer
 * @param otherflags the other flags that might be caught when are waiting for the the DMA to complete through FreeRTOS notifications
 */
static void init_D6T(I2C_HandleTypeDef* const hi2c, volatile i2c_d6t_dma_buffer_t* rawData, uint32_t txThreadFlag, uint32_t* otherflags) {
    //TODO: use the return value to report error
    //fixed parameters of D6T sensors: address, I2C command to get data, and the startup transmissions
    const uint8_t D6Taddr = 0b0001010;
    const uint8_t getCommand = 0x4C;
    const uint8_t startupCommand[5][4] = {
        {0x02, 0x00, 0x01, 0xEE},
        {0x05, 0x90, 0x3A, 0xB8},
        {0x03, 0x00, 0x03, 0x8B},
        {0x03, 0x00, 0x07, 0x97},
        {0x92, 0x00, 0x00, 0xE9}
    };

    //fill the first 3 bytes of rawData with these data since they will be used in CRC
    rawData->addr_write = D6Taddr<<1;
    rawData->command = getCommand;
    rawData->addr_read = (D6Taddr << 1) + 1;

    //init sensor 
    if(HAL_I2C_IsDeviceReady(hi2c, D6Taddr << 1, 5, 0xF) == HAL_OK) {
        for(int i = 0; i<5; i++) {
            HAL_I2C_Master_Transmit_DMA(hi2c, D6Taddr << 1, startupCommand[i], 4);
            wait_for_notif_flags(txThreadFlag, I2C_TIMEOUT, otherflags);
        }
    } else {
        //TODO: report error - cannot connect to sensor
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    xTaskNotifyFromISR(sensors_data_task_handle, FLAG_D6T_STARTUP, eSetBits, NULL);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if(hi2c == &hi2c1) {
        xTaskNotifyFromISR(sensors_data_task_handle, FLAG_I2C1_FINISH, eSetBits, NULL);
    }
    else if (hi2c == &hi2c5) {
        xTaskNotifyFromISR(sensors_data_task_handle, FLAG_I2C5_FINISH, eSetBits, NULL);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc == &hadc1) {
        xTaskNotifyFromISR(sensors_data_task_handle, FLAG_ADC1_FINISH, eSetBits, NULL);
    } else if(hadc == &hadc3) {
        xTaskNotifyFromISR(sensors_data_task_handle, FLAG_ADC3_FINISH, eSetBits, NULL);
    }
}
