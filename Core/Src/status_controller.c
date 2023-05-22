#include "status_controller.h"

// glibc include
#include <stdint.h>
#include <stdio.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// project include
#include "project_def.h"

/* Task control --------------------------------------------------------------*/
__dtcmram uint32_t
    status_controller_task_buffer[STATUS_CONTROLLER_TASK_STACK_SIZE];
__dtcmram StaticTask_t status_controller_task_cb;
TaskHandle_t status_controller_task_handle;

/* Other variable ------------------------------------------------------------*/

/* Task implementation -------------------------------------------------------*/
void status_controller_task(void *argument) {
  (void)argument;

  while (1) {
    vTaskDelay(1000);
  }
}

/* Static and callback function ----------------------------------------------*/
// callback function when i2c slave address is called in listening state
// void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
//                           uint16_t AddrMatchCode) {
//   if (hi2c->Instance == I2C2) {
//     if (AddrMatchCode == 0x10) {
//       if (TransferDirection == 1) {
//         HAL_I2C_Slave_Seq_Receive_DMA(&hi2c2, i2c_rx_buf, sizeof(i2c_rx_buf),
//                                       I2C_FIRST_FRAME);
//         blink_built_in_green_led(1000);
//         printf("i2c address: [%d, %d]\n", i2c_rx_buf[0], i2c_rx_buf[1]);
//       }
//     }
//   }
// }

// void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//   if (hi2c->Instance == I2C2) {
//     blink_built_in_yellow_led(1000);
//     HAL_I2C_Slave_Seq_Receive_DMA(&hi2c2, i2c_tx_buf, sizeof(i2c_tx_buf),
//                                   I2C_LAST_FRAME);
//   }
// }

// void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
//   if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
//     Error_Handler();
//   }
// }
