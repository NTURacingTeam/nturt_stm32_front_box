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
#include "nturt_can_config.h"
#include "nturt_can_config_front_sensor-binutil.h"
#include "nturt_can_config_vcu-binutil.h"
#include "nturt_can_config_vcu_hp-binutil.h"

// project include
#include "project_def.h"
#include "sensors.h"
#include "user_main.h"

/* Private macro -------------------------------------------------------------*/
/* can tx checking macro -----------------------------------------------------*/
#define CHECK_TRANSMISSION(RET)                                               \
  do {                                                                        \
    if ((RET) != HAL_OK) {                                                    \
      ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_TX, ERROR_SET); \
      front_box_can.tx_error_ = true;                                         \
    }                                                                         \
  } while (0)

/* can frame index for rx receive timeout error ------------------------------*/
#define NUM_NOT_DEFINED_FRAME 1
#define NUM_CRITICAL_FRAME 4
#define NUM_OPTIONAL_FRAME 2
#define NUM_FRAME (NUM_CRITICAL_FRAME + NUM_OPTIONAL_FRAME)

// not defined frame
#define FRAME_NOT_DEFINED 0UL

// critical frame
#define FRAME_CRITICAL_BASE (FRAME_NOT_DEFINED + NUM_NOT_DEFINED_FRAME)
#define FRAME_CRITICAL(X) (1UL << (FRAME_CRITICAL_BASE + X - 1UL))

// #define REAR_SENSOR_Status_INDEX FRAME_CRITICAL(0)
#define BMS_Status_INDEX FRAME_CRITICAL(1)
#define INV_Fault_Codes_INDEX FRAME_CRITICAL(2)
#define INV_Fast_Info_INDEX FRAME_CRITICAL(3)

#define FRAME_CRITICAL_MASK \
  ((1UL << (FRAME_CRITICAL_BASE + NUM_CRITICAL_FRAME - 1UL)) - 1UL)

// optional frame
#define FRAME_OPTIONAL_BASE (FRAME_CRITICAL_BASE + NUM_CRITICAL_FRAME)
#define FRAME_OPTIONAL(X) (1UL << (FRAME_OPTIONAL_BASE + X - 1UL))

#define REAR_SENSOR_Status_INDEX FRAME_OPTIONAL(0)

#define FRAME_OPTIONAL_MASK                                            \
  (((1UL << (FRAME_OPTIONAL_BASE + NUM_OPTIONAL_FRAME - 1UL)) - 1UL) - \
   FRAME_CRITICAL_MASK)

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
static __dtcmram StaticSemaphore_t can_front_sensor_tx_mutex_cb;
static __dtcmram StaticSemaphore_t can_vcu_rx_mutex_cb;
static __dtcmram StaticSemaphore_t can_vcu_tx_mutex_cb;
static __dtcmram StaticSemaphore_t can_vcu_hp_rx_mutex_cb;

/* Static function prototype -------------------------------------------------*/
/**
 * @brief Get the index of a frame from its can id.
 *
 * @param id The can id of the frame.
 * @return int The index of the frame. 0 if the frame is not defined.
 */
uint32_t frame_id_to_index(uint32_t id);

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

  /* config can filter -------------------------------------------------------*/
  FDCAN_FilterTypeDef can_filter0 = {
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = 0,
      .FilterType = FDCAN_FILTER_DUAL,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterID1 = INV_Fault_Codes_CANID,
      .FilterID2 = BMS_Status_CANID,
      .RxBufferIndex = 0,
  };
  CHECK_INIT(HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter0));

  FDCAN_FilterTypeDef can_filter1 = {
      .IdType = FDCAN_EXTENDED_ID,
      .FilterIndex = 1,
      .FilterType = FDCAN_FILTER_DUAL,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterID1 = REAR_SENSOR_Status_CANID,
      .FilterID2 = REAR_SENSOR_Status_CANID,
      .RxBufferIndex = 0,
  };
  CHECK_INIT(HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter1));

  FDCAN_FilterTypeDef can_filter2 = {
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = 2,
      .FilterType = FDCAN_FILTER_DUAL,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO1,
      .FilterID1 = INV_Fast_Info_CANID,
      .FilterID2 = INV_Fast_Info_CANID,
      .RxBufferIndex = 0,
  };
  CHECK_INIT(HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter2));

  // configure filter for non-matched id and remote frame
  CHECK_INIT(HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,
                                          FDCAN_REJECT_REMOTE,
                                          FDCAN_REJECT_REMOTE));

  // start can
  CHECK_INIT(HAL_FDCAN_Start(&hfdcan3));

  // activate interrupt for high priority can signal to fifo1
  CHECK_INIT(HAL_FDCAN_ActivateNotification(&hfdcan3,
                                            FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0));

  // can singal struct
  nturt_can_config_vcu_hp_Check_Receive_Timeout_Init(&can_vcu_hp_rx);
  nturt_can_config_vcu_Check_Receive_Timeout_Init(&can_vcu_rx);

  // mutex for can signal struct
  can_front_sensor_tx_mutex =
      xSemaphoreCreateMutexStatic(&can_front_sensor_tx_mutex_cb);
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
uint32_t frame_id_to_index(uint32_t id) {
  switch (id) {
    case REAR_SENSOR_Status_CANID:
      return REAR_SENSOR_Status_INDEX;

    case BMS_Status_CANID:
      return BMS_Status_INDEX;

    case INV_Fault_Codes_CANID:
      return INV_Fault_Codes_INDEX;

    case INV_Fast_Info_CANID:
      return INV_Fast_Info_INDEX;

    default:
      return 0;
  }
}

/* Callback function ---------------------------------------------------------*/
// coderdbc callback function for getting current time in ms
uint32_t __get__tick__() { return get_10us() / 100; }

// coderdbc callback function for sending can message
inline int __send_can_message__(uint32_t msgid, uint8_t ide, uint8_t* d,
                                uint8_t len) {
  return FrontBoxCan_transmit(&front_box_can, ide, msgid, len, d);
}

// coderdbc callback function called when receiving a new frame
void _FMon_MONO_nturt_can_config(FrameMonitor_t* mon, uint32_t msgid) {
  if (mon->cycle_error) {
    int index = frame_id_to_index(msgid);
    if (index != FRAME_NOT_DEFINED) {
      if (index & FRAME_CRITICAL_MASK) {
        if (front_box_can.rx_error_ & FRAME_CRITICAL_MASK) {
          ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_RX_CRITICAL,
                                   ERROR_CLEAR);
        }
      } else if (index & FRAME_OPTIONAL_MASK) {
        if (front_box_can.rx_error_ & FRAME_OPTIONAL_MASK) {
          ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_RX_OPTIONAL,
                                   ERROR_CLEAR);
        }
      }
      front_box_can.rx_error_ &= ~index;
    }

    mon->cycle_error = false;
  }
}

// coderdbc callback function called when reception timeout
void _TOut_MONO_nturt_can_config(FrameMonitor_t* mon, uint32_t msgid,
                                 uint32_t lastcyc) {
  (void)lastcyc;

  if (!mon->cycle_error) {
    int index = frame_id_to_index(msgid);
    if (index != FRAME_NOT_DEFINED) {
      if (index & FRAME_CRITICAL_MASK) {
        if (!(front_box_can.rx_error_ & FRAME_CRITICAL_MASK)) {
          ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_RX_CRITICAL,
                                   ERROR_SET);
        }
      } else if (index & FRAME_OPTIONAL_MASK) {
        if (!(front_box_can.rx_error_ & FRAME_OPTIONAL_MASK)) {
          ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_RX_OPTIONAL,
                                   ERROR_SET);
        }
      }
      front_box_can.rx_error_ |= index;
    }

    mon->cycle_error = true;
  }
}

/* coderdbc callback function called befor transmission ----------------------*/
// don't need to do anything since inverter command is update in
// torque_controller
void FTrn_INV_Command_Message_nturt_can_config(INV_Command_Message_t* m) {
  (void)m;
}

// don't need to do anything since status and error code are update in
// error_handler and status_controller respectively
void FTrn_VCU_Status_nturt_can_config(VCU_Status_t* m) { (void)m; }

// note: the mutex for can frame is already taken in the transmit function
void FTrn_FRONT_SENSOR_1_nturt_can_config(FRONT_SENSOR_1_t* m) {
  GPIO_PinState accelerator_micro, brake_micro;
  ButtonMonitor_read_state(&button_monitor, MICRO_APPS, &accelerator_micro);
  ButtonMonitor_read_state(&button_monitor, MICRO_BSE, &brake_micro);

  m->FRONT_SENSOR_Accelerator_Micro = accelerator_micro;
  m->FRONT_SENSOR_Brake_Micro = brake_micro;

  xSemaphoreTake(steer_angle_sensor.mutex, portMAX_DELAY);
  m->FRONT_SENSOR_Steer_Angle = (int)steer_angle_sensor.steering_angle;
  xSemaphoreGive(steer_angle_sensor.mutex);

  xSemaphoreTake(pedal.mutex, portMAX_DELAY);
  m->FRONT_SENSOR_Accelerator_1_phys = pedal.apps1;
  m->FRONT_SENSOR_Accelerator_2_phys = pedal.apps2;
  m->FRONT_SENSOR_Brake_phys = pedal.bse;
  xSemaphoreGive(pedal.mutex);
}

void FTrn_FRONT_SENSOR_2_nturt_can_config(FRONT_SENSOR_2_t* m) {
  xSemaphoreTake(travel_strain_oil_sensor.mutex, portMAX_DELAY);
  m->FRONT_SENSOR_Front_Left_Suspension_phys = travel_strain_oil_sensor.left;
  m->FRONT_SENSOR_Front_Right_Suspension_phys = travel_strain_oil_sensor.right;
  m->FRONT_SENSOR_Front_Brake_Pressure_phys =
      travel_strain_oil_sensor.oil_pressure;
  m->FRONT_SENSOR_Rear_Brake_Pressure_phys =
      travel_strain_oil_sensor.oil_pressure;
  xSemaphoreGive(travel_strain_oil_sensor.mutex);

  xSemaphoreTake(wheel_speed_sensor.mutex, portMAX_DELAY);
  m->FRONT_SENSOR_Front_Left_Wheel_Speed_phys = wheel_speed_sensor.left;
  m->FRONT_SENSOR_Front_Right_Wheel_Speed_phys = wheel_speed_sensor.right;
  xSemaphoreGive(wheel_speed_sensor.mutex);
}

void FTrn_FRONT_SENSOR_3_nturt_can_config(FRONT_SENSOR_3_t* m) {
  xSemaphoreTake(tire_temp_sensor.mutex, portMAX_DELAY);
  m->FRONT_SENSOR_Front_Left_Tire_Temperature_1_phys =
      (tire_temp_sensor.left[0] + tire_temp_sensor.left[1]) / 2;
  m->FRONT_SENSOR_Front_Left_Tire_Temperature_2_phys =
      (tire_temp_sensor.left[2] + tire_temp_sensor.left[3]) / 2;
  m->FRONT_SENSOR_Front_Left_Tire_Temperature_3_phys =
      (tire_temp_sensor.left[4] + tire_temp_sensor.left[5]) / 2;
  m->FRONT_SENSOR_Front_Left_Tire_Temperature_4_phys =
      (tire_temp_sensor.left[6] + tire_temp_sensor.left[7]) / 2;
  m->FRONT_SENSOR_Front_Right_Tire_Temperature_1_phys =
      (tire_temp_sensor.right[0] + tire_temp_sensor.right[1]) / 2;
  m->FRONT_SENSOR_Front_Right_Tire_Temperature_2_phys =
      (tire_temp_sensor.right[2] + tire_temp_sensor.right[3]) / 2;
  m->FRONT_SENSOR_Front_Right_Tire_Temperature_3_phys =
      (tire_temp_sensor.right[4] + tire_temp_sensor.right[5]) / 2;
  m->FRONT_SENSOR_Front_Right_Tire_Temperature_4_phys =
      (tire_temp_sensor.right[6] + tire_temp_sensor.right[7]) / 2;
  xSemaphoreGive(tire_temp_sensor.mutex);
}
