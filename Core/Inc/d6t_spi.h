#ifndef _D6T_SPI_H
#define _D6T_SPI_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

void __delay_3us_done(TIM_HandleTypeDef *htim);

int spi4_state_machine_start(TaskHandle_t task, const uint32_t flag, uint8_t* txBuffer, uint8_t* rxBuffer);

#endif //_D6T_SPI_H