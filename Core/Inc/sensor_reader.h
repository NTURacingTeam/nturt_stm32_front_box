/**
 * @file sensor_reader.h
 * @author
 * @brief
 */

#ifndef SENSOR_READER_H
#define SENSOR_READER_H

// glibc include
#include <stdbool.h>

// freertos include
#include "FreeRTOS.h"
#include "semphr.h"

// stm32_module incude
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

/* macro ---------------------------------------------------------------------*/
// parameter
#define SENSOR_READER_TASK_PRIORITY TaskPriorityNormal
#define SENSOR_READER_TASK_STACK_SIZE 128
#define SENSOR_READER_TASK_PERIOD 10

/* Typedef -------------------------------------------------------------------*/
typedef struct pedal_data {
  float apps;

  float bse;
} PedalData;

/* Exported variable ---------------------------------------------------------*/
// sensor data
extern PedalData pedal_data;

// mutex
extern SemaphoreHandle_t pedal_data_mutex;

/* class ---------------------------------------------------------------------*/
/**
 * @brief Class for reading sensor data.
 *
 */
typedef struct sensor_reader {
  // inherited class
  Task super_;

  StackType_t task_stack_[SENSOR_READER_TASK_STACK_SIZE];
} SensorReader;

/* constructor ---------------------------------------------------------------*/
/**
 * @brief Constructor for SensorReader.
 *
 * @param[in,out] self The instance of the class.
 * @return None.
 */
void SensorReader_ctor(SensorReader* const self);

/* member function -----------------------------------------------------------*/
/**
 * @brief Function to add torque controller to freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @return ModuleRet Error code.
 */
ModuleRet SensorReader_start(SensorReader* const _self);

/**
 * @brief Function to run in freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @retval None
 * @warning For internal use only.
 */
void SensorReader_task_code(void* const _self);

/* Exported function ---------------------------------------------------------*/

#endif  // SENSOR_READER_H
