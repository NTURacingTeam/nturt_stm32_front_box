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
#include "transfer_functions.h"

//own include
#include "sensors.h"

#define MUTEX_TIMEOUT 0x02
#define ADC_TIMEOUT 0x02
#define ADC_RETRY_TIMEOUT 0x02
#define I2C_TIMEOUT 0xFF
#define SPI_TIMEOUT 0x02

#define FLAG_ADC1_FINISH 0x10
#define FLAG_ADC3_FINISH 0x20
#define FLAG_I2C5_FINISH 0x40
#define FLAG_I2C1_FINISH 0x80
#define FLAG_SPI4_FINISH 0x100
#define FLAG_3US_FINISH 0x200 
#define FLAG_READ_SUS_PEDAL 0x1000
#define FLAG_READ_TIRE_TEMP 0x2000
#define FLAG_READ_STEER 0x4000
#define FLAG_D6T_STARTUP 0x100000
#define FLAG_HALL_EDGE_RIGHT 0x20000 //TODO: makes sure the names are either hall or wheel speed but not both
#define FLAG_HALL_EDGE_LEFT 0x40000

#define NUM_FILTER 3
#define MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE 20

MovingAverageFilter moving_average_filter[NUM_FILTER];
/* static variable -----------------------------------------------------------*/
// filter
static float moving_average_buffer[NUM_FILTER] [MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE];

#define NUM_FILTER 3
#define MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE 20

MovingAverageFilter moving_average_filter[NUM_FILTER];
/* static variable -----------------------------------------------------------*/
// filter
static float moving_average_buffer[NUM_FILTER] [MOVING_AVERAGE_FILTER_MOVING_AVERAGE_SIZE];

#define STEER_PERIOD_NORMAL 5
#define STEER_PERIOD_ERR 15
uint32_t steer_angle_period = STEER_PERIOD_NORMAL; //in ms

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

typedef struct {
    uint32_t elapsed_count;
    uint32_t timer_count;
    SemaphoreHandle_t mutex;
} timer_time_t;

static volatile timer_time_t hall_time_L = {0};
static volatile timer_time_t hall_time_R = {0};
static volatile uint32_t hall_timer_elapsed = 0;

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

steer_angle_data_t steer_angle_sensor = {
    .steering_angle = 0.0
    //mutex is initialized in user_main.c along with everything freertos 
};

wheel_speed_data_t wheel_speed_sensor = {
    .left = 0.0,
    .right = 0.0
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
static uint8_t init_D6T(I2C_HandleTypeDef* const hi2c, volatile i2c_d6t_dma_buffer_t* rawData, uint32_t txThreadFlag, uint32_t* otherflags);
static void update_time_stamp(timer_time_t* last, volatile const timer_time_t* now, timer_time_t* diff);
static uint8_t ADC_request_retry(ADC_HandleTypeDef* hadc, uint16_t* buffer, uint8_t length, uint8_t count);
static uint8_t ADC_retry(ADC_HandleTypeDef* hadc, uint16_t* buffer, uint8_t length, uint8_t count, uint32_t* notifications);

void sensor_timer_callback(TimerHandle_t timer) {
    xTaskNotify(sensors_data_task_handle, FLAG_READ_SUS_PEDAL, eSetBits);

    // we use the timer ID to secretly count how many times have the timer expired
    // and update the tire temp data with also a fix interval
    uint32_t expire_count = (uint32_t)pvTimerGetTimerID(timer);
    expire_count++;
    vTimerSetTimerID(timer, (void*)expire_count);

    if((uint32_t)pvTimerGetTimerID(timer) % (steer_angle_period/SENSOR_TIMER_PERIOD) == 0) {
        xTaskNotify(sensors_data_task_handle, FLAG_READ_STEER, eSetBits);
    }

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
    static __dma_buffer uint8_t spi_rx_buffer[4] = {0};
    static __dma_buffer uint8_t spi_tx_buffer[4] = {0};

    timer_time_t hall_time_L_last = {0};
    timer_time_t hall_time_R_last = {0};

    //TODO: handle every return status of FreeRTOS and HAL API
    (void)argument;
    //variable to store the pending flags that is sent from xTaskNotify
    uint32_t pending_notifications = 0U;

    /*initialize D6T sensors*/
    //first wait for 20ms for the D6T sensors to boot up
    vTaskDelay(pdMS_TO_TICKS(20));

//    if(init_D6T(&hi2c5, &d6t_dma_buffer_R, FLAG_D6T_STARTUP, &pending_notifications)) {
//        Error_Handler();
//    }
//    if(init_D6T(&hi2c1, &d6t_dma_buffer_L, FLAG_D6T_STARTUP, &pending_notifications)) {
//        Error_Handler();
//    }]
    
    //wait for 500ms after initialization of D6T
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
    if(wait_for_notif_flags(FLAG_ADC1_FINISH | FLAG_ADC3_FINISH, ADC_TIMEOUT, &pending_notifications) != pdTRUE) {
        Error_Handler();
    }
    const float apps1_compensation = - APPS1_transfer_function(adc_dma_buffer.apps1, 0);
    const float apps2_compensation = - APPS2_transfer_function(adc_dma_buffer.apps2, 0);
    const float bse_compensation = - BSE_transfer_function(adc_dma_buffer.bse, 0);
    
    //start the initial read, and start the timer that should later notifies this task to do stuff again
    xTaskNotify(xTaskGetCurrentTaskHandle(), FLAG_READ_SUS_PEDAL | FLAG_READ_TIRE_TEMP, eSetBits);
    if(xTimerStart(sensor_timer_handle, portMAX_DELAY) != pdPASS) {
        Error_Handler();
    }

    //start the hall timer
#ifdef USE_HALL_SENSOR
    HAL_TIM_Base_Start(&htim7);
#endif

    while(1) {
        if(!pending_notifications) {
            //wait for notifications from timer if there are no pending ones
            (void)xTaskNotifyWait(0, 0xFFFFFFFFUL, &pending_notifications, portMAX_DELAY);
        }

        if(pending_notifications & FLAG_READ_SUS_PEDAL) {
            pending_notifications &= ~FLAG_READ_SUS_PEDAL; //clear bit

            if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&(adc_dma_buffer.apps1), 4) != HAL_OK) {
                if(ADC_request_retry(&hadc1, &adc_dma_buffer.apps1, 4, 3)) {
                    ErrorHandler_write_error(&error_handler, ERROR_CODE_APPS_IMPLAUSIBILITY | ERROR_CODE_BSE_IMPLAUSIBILITY, ERROR_SET);
                    break;
                }
            }
            if(HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&(adc_dma_buffer.apps2), 3) != HAL_OK) {
                if(ADC_request_retry(&hadc3, &adc_dma_buffer.apps2, 3, 3)) {
                    ErrorHandler_write_error(&error_handler, ERROR_CODE_APPS_IMPLAUSIBILITY | ERROR_CODE_BSE_IMPLAUSIBILITY, ERROR_SET);
                    break;
                }
            }

            const uint8_t micro_apps = (uint8_t)HAL_GPIO_ReadPin(MICRO_APPS_GPIO_Port, MICRO_APPS_Pin);
            const uint8_t micro_bse = (uint8_t)HAL_GPIO_ReadPin(MICRO_BSE_GPIO_Port, MICRO_BSE_Pin);

            xSemaphoreTake(pedal.mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT));
                pedal.micro_apps = micro_apps;
                pedal.micro_bse = micro_bse;
            xSemaphoreGive(pedal.mutex);

            //wait for both flags to be set
            if (wait_for_notif_flags(FLAG_ADC1_FINISH | FLAG_ADC3_FINISH, pdMS_TO_TICKS(ADC_TIMEOUT), &pending_notifications) != pdTRUE) {
                //edge case where only one or neither flags set
                const uint32_t is_ADC1_done = pending_notifications & FLAG_ADC1_FINISH;
                const uint32_t is_ADC3_done = pending_notifications & FLAG_ADC3_FINISH;
                if(!is_ADC3_done) {
                    if(ADC_retry(&hadc3, &adc_dma_buffer.apps2, 3, 3, &pending_notifications)) {
                        pending_notifications &= ~(FLAG_ADC1_FINISH | FLAG_ADC3_FINISH);
                        ErrorHandler_write_error(&error_handler, ERROR_CODE_APPS_IMPLAUSIBILITY | ERROR_CODE_BSE_IMPLAUSIBILITY, ERROR_SET);
                        break;
                    }
                }
                if(!is_ADC1_done) {
                    if(ADC_retry(&hadc1, &adc_dma_buffer.apps1, 4, 3, &pending_notifications)) {
                        pending_notifications &= ~(FLAG_ADC1_FINISH | FLAG_ADC3_FINISH);
                        ErrorHandler_write_error(&error_handler, ERROR_CODE_APPS_IMPLAUSIBILITY | ERROR_CODE_BSE_IMPLAUSIBILITY, ERROR_SET);
                        break;
                    }
                }
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
        if(pending_notifications & FLAG_HALL_EDGE_LEFT) {
            xSemaphoreTake(hall_time_L.mutex, MUTEX_TIMEOUT);
                pending_notifications &= ~FLAG_HALL_EDGE_LEFT; //clear flags

                timer_time_t diff = {0};
                update_time_stamp(&hall_time_L_last, &hall_time_L, &diff);

                xSemaphoreTake(wheel_speed_sensor.mutex, MUTEX_TIMEOUT); 
                    wheel_speed_sensor.left = wheel_speed_tranfser_function(diff.elapsed_count, diff.timer_count);
                xSemaphoreGive(wheel_speed_sensor.mutex);
            xSemaphoreGive(hall_time_L.mutex);
        }
        if(pending_notifications & FLAG_HALL_EDGE_RIGHT) {
            xSemaphoreTake(hall_time_R.mutex, MUTEX_TIMEOUT);
                pending_notifications &= ~FLAG_HALL_EDGE_RIGHT; //clear flags

                timer_time_t diff = {0};
                update_time_stamp(&hall_time_R_last, &hall_time_R, &diff);

                xSemaphoreTake(wheel_speed_sensor.mutex, MUTEX_TIMEOUT);
                    wheel_speed_sensor.right = wheel_speed_tranfser_function(diff.elapsed_count, diff.timer_count);
                xSemaphoreGive(wheel_speed_sensor.mutex);
            xSemaphoreGive(hall_time_R.mutex);
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
        if(pending_notifications & FLAG_READ_STEER) {
            pending_notifications &= ~FLAG_READ_STEER; //clear flags

            //see https://www.cuidevices.com/product/resource/amt22.pdf
            //TODO: using hardware NSS right now because SCK jitters when NSS is switched by software for some reason
            //but hardware NSS does not observe 3us after all bytes transmission
            spi_tx_buffer[0] = 0;
            spi_tx_buffer[1] = 0;

            if(HAL_SPI_TransmitReceive_IT(&hspi4, spi_tx_buffer, spi_rx_buffer, 2) != HAL_OK) {
                steer_angle_period = STEER_PERIOD_ERR;
                break;
            }
            if(wait_for_notif_flags(FLAG_SPI4_FINISH, pdMS_TO_TICKS(SPI_TIMEOUT), &pending_notifications) != pdTRUE) {
                steer_angle_period = STEER_PERIOD_ERR;
                break;
            }
            else {
                steer_angle_period = STEER_PERIOD_NORMAL;
            }

            //TODO: CRC?

            //calculate position and put the stuff in to variables
            const uint8_t highByte = spi_rx_buffer[0];
            const uint8_t lowByte = spi_rx_buffer[1];
            const uint16_t position = ((uint16_t)highByte << 8) + (uint16_t)lowByte;
            xSemaphoreTake(steer_angle_sensor.mutex, MUTEX_TIMEOUT);
                steer_angle_sensor.steering_angle = steer_angle_transfer_function((position & 0x3FFF) >> 2);
            xSemaphoreGive(steer_angle_sensor.mutex);
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
    while(~flag_gotten & target) {
        BaseType_t Wait_result = xTaskNotifyWait(0, 0xFFFFFFFFUL, &flag_buf, timeout-(tlast-t0));
        if(Wait_result == pdFALSE) {
            return pdFALSE; 
        }

        flag_gotten |= flag_buf;
        *gotten = flag_gotten;
        tlast = xTaskGetTickCount();
    } 

    //remove the gotten flags if the flags are correctly received
    *gotten &= ~(target);
    return pdTRUE;
}

/**
 * @brief this function initializes the payload data of the i2c addresses and the D6T sensors themselves
 * 
 * @param hi2c the i2c handle that handles the i2c communications
 * @param rawData and array that stores the data to be transmitted and that to be received
 * @param txThreadFlag the task notification flag used to indicate completion of transfer
 * @param otherflags the other flags that might be caught when are waiting for the the DMA to complete through FreeRTOS notifications
 */
static uint8_t init_D6T(I2C_HandleTypeDef* const hi2c, volatile i2c_d6t_dma_buffer_t* rawData, uint32_t txThreadFlag, uint32_t* otherflags) {
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
            if(wait_for_notif_flags(txThreadFlag, I2C_TIMEOUT, otherflags) != pdTRUE) {
                return 1;
            }
        }
    } else {
        return 1;
    }
    return 0;
}

/**
 * @brief this function calculates the difference between last and now, and updates the "now" time to "last"
 * 
 * @param last previous time this function is called
 * @param now the new time stamp at which this time is called
 * @param diff calculates the time difference between last and now
 */
void update_time_stamp(timer_time_t* last, volatile const timer_time_t* now, timer_time_t* diff) {
    diff->elapsed_count = now->elapsed_count - last->elapsed_count;
    diff->timer_count = now->timer_count - last->timer_count;

    if(now->timer_count < last->timer_count) {
        diff->elapsed_count -= 1;
    }
    last->elapsed_count = now->elapsed_count;
    last->timer_count = now->timer_count;
}

static uint8_t ADC_request_retry(ADC_HandleTypeDef* hadc, uint16_t* buffer, uint8_t length, const uint8_t count) {
    uint8_t toReturn = 0;
    for(int i = 0; i<length; i++) {
        if(HAL_ADC_Start_DMA(hadc, (uint32_t*)buffer, length) != HAL_OK) {
            toReturn = 1;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        else {
            toReturn = 0;
            break;
        }
    }

    return toReturn;
}

static uint8_t ADC_retry(ADC_HandleTypeDef* hadc, uint16_t* buffer, uint8_t length, uint8_t count, uint32_t* notifications) {
    if(ADC_request_retry(hadc, buffer, length, count)) {
        return 1;
    }
    if(wait_for_notif_flags(FLAG_ADC1_FINISH, pdMS_TO_TICKS(ADC_RETRY_TIMEOUT), notifications) != pdTRUE) {
        return 1;
    }
    return 0;
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

#ifdef USE_HALL_SENSOR
void __hall_timer_elapsed(TIM_HandleTypeDef *htim) {
    (void)htim;
    hall_timer_elapsed++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {    
    if(GPIO_Pin == HALL_L_Pin) {
        if(xSemaphoreTakeFromISR(hall_time_L.mutex, NULL) == pdTRUE) {
            hall_time_L.timer_count = __HAL_TIM_GET_COUNTER(&htim7);
            hall_time_L.elapsed_count = hall_timer_elapsed;
            xTaskNotifyFromISR(sensors_data_task_handle, FLAG_HALL_EDGE_LEFT, eSetBits, NULL);
            xSemaphoreGiveFromISR(hall_time_L.mutex, NULL);
        }
    }   
    if(GPIO_Pin == HALL_R_Pin) {
        if(xSemaphoreTakeFromISR(hall_time_R.mutex, NULL) == pdTRUE) {
            hall_time_R.timer_count = __HAL_TIM_GET_COUNTER(&htim7);
            hall_time_R.elapsed_count = hall_timer_elapsed;
            xTaskNotifyFromISR(sensors_data_task_handle, FLAG_HALL_EDGE_RIGHT, eSetBits, NULL);
            xSemaphoreGiveFromISR(hall_time_R.mutex, NULL);
        }
    }
}
#endif

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if(hspi == &hspi4) {
    	xTaskNotifyFromISR(sensors_data_task_handle, FLAG_SPI4_FINISH, eSetBits, NULL);
    }
}

void sensor_mutex_init(void) {
    sensors_data_task_handle = xTaskCreateStatic(
        sensor_handler, "sensors_data_task", SENSOR_DATA_TASK_STACK_SIZE, NULL,
        TaskPriorityHigh, sensors_data_task_buffer, &sensors_data_task_cb);
        
    sensor_timer_handle = xTimerCreateStatic(
        "sensors_data_timer", pdMS_TO_TICKS(SENSOR_TIMER_PERIOD), pdTRUE, 0,
        sensor_timer_callback, &sensor_timer_buffer);

    //create all the necessary mutexes
    hall_time_L.mutex = xSemaphoreCreateMutex();
    hall_time_R.mutex = xSemaphoreCreateMutex();
    pedal.mutex = xSemaphoreCreateMutex();
    travel_strain_oil_sensor.mutex = xSemaphoreCreateMutex();
    tire_temp_sensor.mutex = xSemaphoreCreateMutex();
    steer_angle_sensor.mutex = xSemaphoreCreateMutex();
    wheel_speed_sensor.mutex = xSemaphoreCreateMutex();
    return;
}
