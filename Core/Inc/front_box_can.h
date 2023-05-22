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
#include "nturt_can_config_vcu-binutil.h"
#include "nturt_can_config_vcu_hp-binutil.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

/* Exported variable ---------------------------------------------------------*/
// c-coderdbc can singal struct
extern nturt_can_config_vcu_rx_t can_vcu_rx;
extern nturt_can_config_vcu_tx_t can_vcu_tx;
extern nturt_can_config_vcu_hp_rx_t can_vcu_hp_rx;

// mutex
extern SemaphoreHandle_t can_vcu_rx_mutex;
extern SemaphoreHandle_t can_vcu_tx_mutex;
extern SemaphoreHandle_t can_vcu_hp_rx_mutex;

/* class inherited from CanTransceiver ---------------------------------------*/
typedef struct test_can {
  CanTransceiver super_;
} FrontBoxCan;

/* constructor ---------------------------------------------------------------*/
void FrontBoxCan_ctor(FrontBoxCan* self, CanHandle* const can_handle);

/* member function -----------------------------------------------------------*/
ModuleRet FrontBoxCan_configure(FrontBoxCan* const self);

ModuleRet FrontBoxCan_receive(FrontBoxCan* const self, const bool is_extended,
                              const uint32_t id, const uint8_t dlc,
                              const uint8_t* const data);

ModuleRet FrontBoxCan_receive_hp(FrontBoxCan* const self,
                                 const bool is_extended, const uint32_t id,
                                 const uint8_t dlc, const uint8_t* const data);

ModuleRet FrontBoxCan_periodic_update(FrontBoxCan* const self,
                                      const TickType_t current_tick);

/* virtual function declaration ----------------------------------------------*/
ModuleRet __FrontBoxCan_configure(CanTransceiver* self);

ModuleRet __FrontBoxCan_receive(CanTransceiver* self, bool is_extended,
                                uint32_t id, uint8_t dlc, const uint8_t* data);

ModuleRet __FrontBoxCan_receive_hp(CanTransceiver* self, bool is_extended,
                                   uint32_t id, uint8_t dlc,
                                   const uint8_t* data);

ModuleRet __FrontBoxCan_periodic_update(CanTransceiver* self,
                                        TickType_t period);

#endif  // FRONT_BOX_CAN_H
