#include "front_box_can.h"

// glibc
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "semphr.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// can_config include
#include "nturt_can_config.h"
#include "nturt_can_config_vcu-binutil.h"
#include "nturt_can_config_vcu_hp-binutil.h"

// project include
#include "project_def.h"
#include "user_main.h"

/* Exported variable ---------------------------------------------------------*/
// c-coderdbc can singal struct
__dtcmram nturt_can_config_vcu_rx_t can_vcu_rx;
__dtcmram nturt_can_config_vcu_tx_t can_vcu_tx;
__dtcmram nturt_can_config_vcu_hp_rx_t can_vcu_hp_rx;

// mutex
__dtcmram SemaphoreHandle_t can_vcu_rx_mutex;
__dtcmram SemaphoreHandle_t can_vcu_tx_mutex;
__dtcmram SemaphoreHandle_t can_vcu_hp_rx_mutex;

/* Static variable -----------------------------------------------------------*/
// mutex control block
static __dtcmram StaticSemaphore_t can_vcu_rx_mutex_cb;
static __dtcmram StaticSemaphore_t can_vcu_tx_mutex_cb;
static __dtcmram StaticSemaphore_t can_vcu_hp_rx_mutex_cb;

// control flag
static bool tx_error = false;
static volatile bool rx_warn = false, rx_error = false;

/* virtual function redirection ----------------------------------------------*/
ModuleRet FrontBoxCan_configure(FrontBoxCan* const self) {
  return self->super_.vptr_->configure(&self->super_);
}

ModuleRet FrontBoxCan_receive(FrontBoxCan* const self, const bool is_extended,
                              const uint32_t id, const uint8_t dlc,
                              const uint8_t* const data) {
  return self->super_.vptr_->receive(&self->super_, is_extended, id, dlc, data);
}

ModuleRet FrontBoxCan_receive_hp(FrontBoxCan* const self,
                                 const bool is_extended, const uint32_t id,
                                 const uint8_t dlc, const uint8_t* const data) {
  return self->super_.vptr_->receive_hp(&self->super_, is_extended, id, dlc,
                                        data);
}

ModuleRet FrontBoxCan_periodic_update(FrontBoxCan* const self,
                                      const TickType_t current_tick) {
  return self->super_.vptr_->periodic_update(&self->super_, current_tick);
}

/* virtual function definition -----------------------------------------------*/
// from CanTransceiver base class
ModuleRet __FrontBoxCan_configure(CanTransceiver* const self) {
  (void)self;

  // config can filter
  FDCAN_FilterTypeDef can_standard_filter0 = {
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = 0,
      .FilterType = FDCAN_FILTER_DUAL,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO1,
      .FilterID1 = 0x11,
      .FilterID2 = 0x12,
      .RxBufferIndex = 0,
  };
  if (HAL_FDCAN_ConfigFilter(&hfdcan3, &can_standard_filter0) != HAL_OK) {
    Error_Handler();
  }
  FDCAN_FilterTypeDef can_standard_filter1 = {
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = 1,
      .FilterType = FDCAN_FILTER_DUAL,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterID1 = 0x13,
      .FilterID2 = 0x14,
      .RxBufferIndex = 0,
  };
  if (HAL_FDCAN_ConfigFilter(&hfdcan3, &can_standard_filter1) != HAL_OK) {
    Error_Handler();
  }

  // configure filter for non-matched id and remote frame
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE)) {
    Error_Handler();
  }

  // activate interrupt for high priority can signal to fifo1
  if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
                                     0) != HAL_OK) {
    Error_Handler();
  }

  // start can
  if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
    Error_Handler();
  }

  // can singal struct
  nturt_can_config_vcu_hp_Check_Receive_Timeout_Init(&can_vcu_hp_rx);
  nturt_can_config_vcu_Check_Receive_Timeout_Init(&can_vcu_rx);

  // mutex for can signal struct
  can_vcu_rx_mutex = xSemaphoreCreateMutexStatic(&can_vcu_rx_mutex_cb);
  can_vcu_tx_mutex = xSemaphoreCreateMutexStatic(&can_vcu_tx_mutex_cb);
  can_vcu_hp_rx_mutex = xSemaphoreCreateMutexStatic(&can_vcu_hp_rx_mutex_cb);

  return ModuleOK;
}

// from CanTransceiver base class
ModuleRet __FrontBoxCan_receive(CanTransceiver* const self,
                                const bool is_extended, const uint32_t id,
                                const uint8_t dlc, const uint8_t* const data) {
  (void)self;
  (void)is_extended;

  xSemaphoreTake(can_vcu_rx_mutex, portMAX_DELAY);
  nturt_can_config_vcu_Receive(&can_vcu_rx, data, id, dlc);
  xSemaphoreGive(can_vcu_rx_mutex);

  return ModuleOK;
}

// from CanTransceiver base class
ModuleRet __FrontBoxCan_receive_hp(CanTransceiver* const self,
                                   const bool is_extended, const uint32_t id,
                                   const uint8_t dlc,
                                   const uint8_t* const data) {
  (void)self;
  (void)is_extended;

  xSemaphoreTake(can_vcu_hp_rx_mutex, portMAX_DELAY);
  nturt_can_config_vcu_hp_Receive(&can_vcu_hp_rx, data, id, dlc);
  xSemaphoreGive(can_vcu_hp_rx_mutex);

  return ModuleOK;
}

// from CanTransceiver base class
ModuleRet __FrontBoxCan_periodic_update(CanTransceiver* const self,
                                        const TickType_t current_tick) {
  (void)self;
  (void)current_tick;

  xSemaphoreTake(can_vcu_rx_mutex, portMAX_DELAY);
  nturt_can_config_vcu_Check_Receive_Timeout(&can_vcu_rx);
  xSemaphoreGive(can_vcu_rx_mutex);

  xSemaphoreTake(can_vcu_hp_rx_mutex, portMAX_DELAY);
  nturt_can_config_vcu_hp_Check_Receive_Timeout(&can_vcu_hp_rx);
  xSemaphoreGive(can_vcu_hp_rx_mutex);

  xSemaphoreTake(can_vcu_tx_mutex, portMAX_DELAY);
  nturt_can_config_vcu_Transmit(&can_vcu_tx);
  xSemaphoreGive(can_vcu_tx_mutex);

  return ModuleOK;
}

/* constructor ---------------------------------------------------------------*/
void FrontBoxCan_ctor(FrontBoxCan* self, CanHandle* const can_handle) {
  // construct inherited class and redirect virtual function
  CanTransceiver_ctor(&self->super_, can_handle);
  static struct CanTransceiverVtbl vtbl = {
      .configure = __FrontBoxCan_configure,
      .receive = __FrontBoxCan_receive,
      .receive_hp = __FrontBoxCan_receive_hp,
      .periodic_update = __FrontBoxCan_periodic_update,
  };
  self->super_.vptr_ = &vtbl;
}

/* Static and callback function ----------------------------------------------*/
// coderdbc callback function for getting current time in ms
uint32_t __get__tick__() { return get_10us() / 100; }

// coderdbc callback function when frame reception timeouted
void __alert_reception_timeout__(uint32_t msgid, uint32_t lastcyc) {
  switch (msgid) {
    case INV_Fast_Info_CANID:
    case BMS_Current_Limit_CANID:
    default:
      if ((get_10us() / 100 - lastcyc) < 1000) {
        if (!rx_warn) {
          rx_warn = true;
          ErrorHandler_write_error(&error_handler,
                                   ERROR_CODE_CAN_RECEIVE_TIMEOUT_WARN,
                                   ERROR_OPTION_SET);
          rx_warn = true;
        }
      } else if (!rx_error) {
        rx_error = true;
        ErrorHandler_write_error(&error_handler,
                                 ERROR_CODE_CAN_RECEIVE_TIMEOUT_ERROR,
                                 ERROR_OPTION_SET);
      }
      break;
  }
}

// coderdbc callback function for sending can message
int inline __send_can_message__(uint32_t msgid, uint8_t ide, uint8_t* d,
                                uint8_t len) {
  return CanTransceiver_transmit((CanTransceiver*)&front_box_can, ide, msgid,
                                 len, d);
}
