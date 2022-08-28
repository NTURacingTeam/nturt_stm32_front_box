/**
  ******************************************************************************
  * @file    analog_transfer_function.h
  * @author  Tsou, Min-Lun from 2022 NTUR
  * @brief   Header file of the transfer functions for the analog sensors on ep4
  ******************************************************************************
  */

#ifndef _ANALOG_TRANSFER_FUNCTION_H
#define _ANALOG_TRANSFER_FUNCTION_H
#include <stdint.h>

uint8_t APPS1_transfer_function(uint32_t reading);
uint8_t APPS2_transfer_function(uint32_t reading);
uint8_t BSE_transfer_function(uint32_t reading);
uint8_t oil_pressure_transfer_function(uint32_t reading);
uint8_t suspension_travel_transfer_function(uint32_t reading);
uint16_t wheel_speed_transfer_function(uint32_t reading);
uint8_t tire_temp_transfer_function(uint16_t);

#endif /*_ANALOG_TRANSFER_FUNCTION_H*/
