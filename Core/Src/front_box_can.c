#include "front_box_can.h"

// glibc
#include <stdbool.h>
#include <stddef.h>
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
#include "canmonitorutil.h"
#include "nturt_can_config_front_sensor-binutil.h"
#include "nturt_can_config_vcu-binutil.h"
#include "nturt_can_config_vcu_hp-binutil.h"

// project include
#include "project_def.h"
#include "user_main.h"

/* Private macro -------------------------------------------------------------*/
// assert macro
#define IS_CRITICAL_FRAME(ID) \
  ((ID) == INV_Fast_Info_CANID || (ID) == BMS_Current_Limit_CANID)

#define IS_OPTIONAL_FRAME(ID) ((ID) == 0x00)

#define CHECK_TRANSMISSION(RET)                                               \
  do {                                                                        \
    if ((RET) != HAL_OK) {                                                    \
      ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_TX, ERROR_SET); \
      front_box_can.tx_error_ = true;                                         \
    }                                                                         \
  } while (0)

/* can frame index for rx receive timeout error ------------------------------*/
#define NUM_CRITICAL_FRAME 2
#define NUM_OPTIONAL_FRAME 2
#define NUM_FRAME (NUM_CRITICAL_FRAME + NUM_OPTIONAL_FRAME)

// critical frame
#define FRAME_CRITICAL_BASE 0
#define FRAME_CRITICAL_MASK ((1UL << NUM_CRITICAL_FRAME) - 1UL)
#define FRAME_CRITICAL(X) (FRAME_CRITICAL_BASE + X)

#define INV_Fast_Info_INDEX FRAME_CRITICAL(0)
#define BMS_Current_Limit_INDEX FRAME_CRITICAL(1)

// optional frame
#define FRAME_OPTIONAL_BASE (FRAME_CRITICAL_BASE + NUM_CRITICAL_FRAME)
#define FRAME_OPTIONAL_MASK                              \
  ((1UL << (FRAME_OPTIONAL_BASE + NUM_OPTIONAL_FRAME)) - \
   (1UL << FRAME_OPTIONAL_BASE))
#define FRAME_OPTIONAL(X) (FRAME_OPTIONAL_BASE + X)

/* Exported variable ---------------------------------------------------------*/
// c-coderdbc can singal struct
__dtcmram nturt_can_config_front_sensor_tx_t can_front_sensor_tx;
__dtcmram nturt_can_config_vcu_rx_t can_vcu_rx;
__dtcmram nturt_can_config_vcu_tx_t can_vcu_tx;
__dtcmram nturt_can_config_vcu_hp_rx_t can_vcu_hp_rx;

// mutex
__dtcmram SemaphoreHandle_t can_front_sensor_tx_mutex;
__dtcmram SemaphoreHandle_t can_vcu_rx_mutex;
__dtcmram SemaphoreHandle_t can_vcu_tx_mutex;
__dtcmram SemaphoreHandle_t can_vcu_hp_rx_mutex;

/* Static variable -----------------------------------------------------------*/
// mutex control block
static __dtcmram StaticSemaphore_t can_front_sensor_mutex_cb;
static __dtcmram StaticSemaphore_t can_vcu_rx_mutex_cb;
static __dtcmram StaticSemaphore_t can_vcu_tx_mutex_cb;
static __dtcmram StaticSemaphore_t can_vcu_hp_rx_mutex_cb;

/* Static function prototype -------------------------------------------------*/
/**
 * @brief Get the index of a frame from its can id.
 *
 * @param id The can id of the frame.
 * @return int The index of the frame. -1 if the frame is not defined.
 */
int frame_id_to_index(uint32_t id);

/* virtual function redirection ----------------------------------------------*/
ModuleRet FrontBoxCan_start(FrontBoxCan* const self) {
  return self->super_.super_.vptr_->start(&self->super_.super_);
}

void FrontBoxCan_configure(FrontBoxCan* const self) {
  self->super_.vptr_->configure(&self->super_);
}

void FrontBoxCan_receive(FrontBoxCan* const self, const bool is_extended,
                         const uint32_t id, const uint8_t dlc,
                         const uint8_t* const data) {
  self->super_.vptr_->receive(&self->super_, is_extended, id, dlc, data);
}

void FrontBoxCan_receive_hp(FrontBoxCan* const self, const bool is_extended,
                            const uint32_t id, const uint8_t dlc,
                            const uint8_t* const data) {
  self->super_.vptr_->receive_hp(&self->super_, is_extended, id, dlc, data);
}

void FrontBoxCan_periodic_update(FrontBoxCan* const self,
                                 const TickType_t current_tick) {
  self->super_.vptr_->periodic_update(&self->super_, current_tick);
}

/* virtual function definition -----------------------------------------------*/
// from CanTransceiver base class
void __FrontBoxCan_configure(CanTransceiver* const self) {
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
  can_front_sensor_tx_mutex =
      xSemaphoreCreateMutexStatic(&can_front_sensor_mutex_cb);
  can_vcu_rx_mutex = xSemaphoreCreateMutexStatic(&can_vcu_rx_mutex_cb);
  can_vcu_tx_mutex = xSemaphoreCreateMutexStatic(&can_vcu_tx_mutex_cb);
  can_vcu_hp_rx_mutex = xSemaphoreCreateMutexStatic(&can_vcu_hp_rx_mutex_cb);
}

// from CanTransceiver base class
void __FrontBoxCan_receive(CanTransceiver* const self, const bool is_extended,
                           const uint32_t id, const uint8_t dlc,
                           const uint8_t* const data) {
  (void)self;
  (void)is_extended;

  LedController_blink(&led_controller, LED_CAN_RX, 10);

  xSemaphoreTake(can_vcu_rx_mutex, portMAX_DELAY);
  nturt_can_config_vcu_Receive(&can_vcu_rx, data, id, dlc);
  xSemaphoreGive(can_vcu_rx_mutex);
}

// from CanTransceiver base class
void __FrontBoxCan_receive_hp(CanTransceiver* const self,
                              const bool is_extended, const uint32_t id,
                              const uint8_t dlc, const uint8_t* const data) {
  (void)self;
  (void)is_extended;

  LedController_blink(&led_controller, LED_CAN_RX, 10);

  xSemaphoreTake(can_vcu_hp_rx_mutex, portMAX_DELAY);
  nturt_can_config_vcu_hp_Receive(&can_vcu_hp_rx, data, id, dlc);
  xSemaphoreGive(can_vcu_hp_rx_mutex);
}

// from CanTransceiver base class
void __FrontBoxCan_periodic_update(CanTransceiver* const _self,
                                   const TickType_t current_tick) {
  (void)current_tick;

  FrontBoxCan* self = (FrontBoxCan*)_self;

  xSemaphoreTake(can_vcu_rx_mutex, portMAX_DELAY);
  nturt_can_config_vcu_Check_Receive_Timeout(&can_vcu_rx);
  xSemaphoreGive(can_vcu_rx_mutex);

  xSemaphoreTake(can_vcu_hp_rx_mutex, portMAX_DELAY);
  nturt_can_config_vcu_hp_Check_Receive_Timeout(&can_vcu_hp_rx);
  xSemaphoreGive(can_vcu_hp_rx_mutex);

  if (self->tx_error_) {
    if (HAL_FDCAN_GetTxFifoFreeLevel(self->super_.can_handle_) > 16) {
      ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_TX, ERROR_CLEAR);
      self->tx_error_ = false;
    }
  } else {
    xSemaphoreTake(can_front_sensor_tx_mutex, portMAX_DELAY);
    CHECK_TRANSMISSION(
        nturt_can_config_front_sensor_Transmit(&can_front_sensor_tx));
    xSemaphoreGive(can_front_sensor_tx_mutex);

    xSemaphoreTake(can_vcu_tx_mutex, portMAX_DELAY);
    CHECK_TRANSMISSION(nturt_can_config_vcu_Transmit(&can_vcu_tx));
    xSemaphoreGive(can_vcu_tx_mutex);
  }
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

  // initialize member variable
  self->tx_error_ = false;
  self->rx_error_ = 0;
}

/* member function -----------------------------------------------------------*/
inline ModuleRet FrontBoxCan_transmit(FrontBoxCan* const self,
                                      const bool is_extended, const uint32_t id,
                                      const uint8_t dlc, uint8_t* const data) {
  LedController_blink(&led_controller, LED_CAN_TX, 10);
  return CanTransceiver_transmit(&self->super_, is_extended, id, dlc, data);
}

/* Exported function ---------------------------------------------------------*/

/* Static function -----------------------------------------------------------*/
int frame_id_to_index(uint32_t id) {
  switch (id) {
    case INV_Fast_Info_CANID:
      return INV_Fast_Info_INDEX;
    case BMS_Current_Limit_CANID:
      return BMS_Current_Limit_INDEX;
    default:
      return -1;
  }
}
/* Callback function ---------------------------------------------------------*/
// coderdbc callback function for getting current time in ms
uint32_t __get__tick__() { return get_10us() / 100; }

// coderdbc callback function called when receiving a new frame
void _FMon_MONO_nturt_can_config(FrameMonitor_t* mon, uint32_t msgid) {
  if (mon->cycle_error) {
    if (IS_CRITICAL_FRAME(msgid)) {
      front_box_can.rx_error_ &= ~(1UL << frame_id_to_index(msgid));
      if (!(front_box_can.rx_error_ & FRAME_CRITICAL_MASK)) {
        ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_RX_CRITICAL,
                                 ERROR_CLEAR);
      }
    } else if (IS_OPTIONAL_FRAME(msgid)) {
      front_box_can.rx_error_ &= ~(1UL << frame_id_to_index(msgid));
      if (!(front_box_can.rx_error_ & FRAME_OPTIONAL_MASK)) {
        ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_RX_OPTIONAL,
                                 ERROR_CLEAR);
      }
    }

    mon->cycle_error = false;
  }
}

// coderdbc callback function called when reception timeout
void _TOut_MONO_nturt_can_config(FrameMonitor_t* mon, uint32_t msgid,
                                 uint32_t lastcyc) {
  (void)lastcyc;

  if (!mon->cycle_error) {
    if (IS_CRITICAL_FRAME(msgid)) {
      front_box_can.rx_error_ |= (1UL << frame_id_to_index(msgid));
      if (front_box_can.rx_error_ & FRAME_CRITICAL_MASK) {
        ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_RX_CRITICAL,
                                 ERROR_SET);
      }
    } else if (IS_OPTIONAL_FRAME(msgid)) {
      front_box_can.rx_error_ |= (1UL << frame_id_to_index(msgid));
      if (front_box_can.rx_error_ & FRAME_OPTIONAL_MASK) {
        ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_RX_OPTIONAL,
                                 ERROR_SET);
      }
    }

    mon->cycle_error = true;
  }
}

// coderdbc callback function for sending can message
inline int __send_can_message__(uint32_t msgid, uint8_t ide, uint8_t* d,
                                uint8_t len) {
  return FrontBoxCan_transmit(&front_box_can, ide, msgid, len, d);
}
