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

void throttle_sensors_calibration(uint32_t reading,uint8_t sensor_number);
float APPS1_conversion(uint32_t reading);
float APPS2_conversion(uint32_t reading);
float BSE_conversion(uint32_t reading);
uint8_t throttle_sensors_transfer_function(uint32_t reading, uint8_t sensor_number);
uint8_t BSE_transfer_function(uint32_t reading);
uint8_t oil_pressure_transfer_function(uint32_t reading);
uint8_t suspension_travel_transfer_function(uint32_t reading);
uint16_t wheel_speed_transfer_function(uint32_t reading);
uint8_t tire_temp_transfer_function(uint16_t);
uint16_t steering_transfer_function(uint16_t reading);

#endif /*_ANALOG_TRANSFER_FUNCTION_H*/
