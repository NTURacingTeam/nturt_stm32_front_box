/**
 * @file front_box_can.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef FRONT_BOX_CAN_H
#define FRONT_BOX_CAN_H

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "semphr.h"

// can_config include
#include "nturt_can_config_front_sensor-binutil.h"
#include "nturt_can_config_vcu-binutil.h"
#include "nturt_can_config_vcu_hp-binutil.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

/* Exported variable ---------------------------------------------------------*/
// c-coderdbc can singal struct
extern nturt_can_config_front_sensor_tx_t can_front_sensor_tx;
extern nturt_can_config_vcu_rx_t can_vcu_rx;
extern nturt_can_config_vcu_tx_t can_vcu_tx;
extern nturt_can_config_vcu_hp_rx_t can_vcu_hp_rx;

// mutex
extern SemaphoreHandle_t can_front_sensor_tx_mutex;
extern SemaphoreHandle_t can_vcu_rx_mutex;
extern SemaphoreHandle_t can_vcu_tx_mutex;
extern SemaphoreHandle_t can_vcu_hp_rx_mutex;

/* class inherited from CanTransceiver ---------------------------------------*/
typedef struct test_can {
  // inherited class
  CanTransceiver super_;

} FrontBoxCan;

/* constructor ---------------------------------------------------------------*/
void FrontBoxCan_ctor(FrontBoxCan* self, CanHandle* const can_handle);

/* member function -----------------------------------------------------------*/
/**
 * @brief Function to add front box can to freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @return ModuleRet Error code.
 */
ModuleRet FrontBoxCan_start(FrontBoxCan* const self);

ModuleRet FrontBoxCan_configure(FrontBoxCan* const self);

ModuleRet FrontBoxCan_receive(FrontBoxCan* const self, const bool is_extended,
                              const uint32_t id, const uint8_t dlc,
                              const uint8_t* const data);

ModuleRet FrontBoxCan_receive_hp(FrontBoxCan* const self,
                                 const bool is_extended, const uint32_t id,
                                 const uint8_t dlc, const uint8_t* const data);

ModuleRet FrontBoxCan_transmit(FrontBoxCan* const self, const bool is_extended,
                               const uint32_t id, const uint8_t dlc,
                               uint8_t* const data);

ModuleRet FrontBoxCan_periodic_update(FrontBoxCan* const self,
                                      const TickType_t current_tick);

/* Exported function ---------------------------------------------------------*/

#endif  // FRONT_BOX_CAN_H
