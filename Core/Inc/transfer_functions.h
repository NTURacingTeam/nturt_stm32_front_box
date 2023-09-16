#ifndef _TRANSFER_FUNCTIONS_H
#define _TRANSFER_FUNCTIONS_H

#include <stdint.h>
#include "sensors.h"

float fuzzy_edge_remover(const float raw, const float highEdge, const float lowEdge, const float margin);

//we use the M3041-000005-01KPG for the sensor
//if this is not defined, we use M3041-000005-05KPG
//#define USE_01KPG

/**
 * @brief transfer function for the first APPS
 *
 * @param reading the 12 bit value read from the ADC
 * @return float the normalized read value spanning from 0 to 1
 *
 * The detailed description about the transfer function can be found here:
 * https://hackmd.io/@nturacing/ByOF6I5T9/%2F2Jgh0ieyS0mc_r-6pHKQyQ
 */
static inline float APPS1_transfer_function(const uint16_t reading, const float compensation) {
    return (float)(reading-860)/(3891-860) + compensation;
}

static inline float APPS2_transfer_function (const uint16_t reading, const float compensation) {
    return (float)(reading*2-860)/(3891-860) + compensation;
}

static inline float BSE_transfer_function(const uint16_t reading, const float compensation) {
    // since we only use 2.5~24.5mm part of the domain instead of the full 0~25, we have
    return (float)(reading-82)/(3686-82) + compensation;
}

static inline float tire_temp_transfer_function(const uint8_t highByte, const uint8_t lowByte) {
    return (float)((((int16_t)highByte) << 8) + (int16_t)lowByte)/5;
}

static inline float travel_transfer_function (const uint16_t reading) {
    return 75 - (float)reading/4096 * 75;
}

static inline float oil_transfer_function(const uint16_t reading) {
    //see https://www.mouser.tw/datasheet/2/418/8/ENG_DS_MSP300_B1-1130121.pdf
    //extra 3.3/3 is because a voltage divider moved 5V to 3V while max voltage on the system is 3.3V
#ifdef USE_01KPG
	return ((float)reading /4096 *3.3/3 *5-1)/4  * 70;
#else
    return ((float)reading /4096 *3.3/3 *5-1)/4  * 350;
#endif

}

static inline float steer_angle_transfer_function(const uint16_t reading) {
    //see https://www.cuidevices.com/product/resource/amt22.pdf
    //our model outputs 12bit number
    return (float)reading * 360.0/4096.0;
}
/**
 * @brief 
 * 
 * @param elapsed how many times have the timer expired during the 2 consecutive falling edge exti generated by the hall sensor
 * @param elapsed how much different is the counter's count
 * @return float that represents the speed of the wheel in rpm
 */
static inline float wheel_speed_tranfser_function(const uint32_t elapsed, const uint32_t count) {
    const float tooth_per_rev = 14.0;
    return (1.0/tooth_per_rev) / (elapsed*WHEEL_SPEED_TIMER_PERIOD + count*WHEEL_SPEED_TIMER_COUNT_PERIOD) * 1000 * 60;
}

/**
 * @brief exponential filter to filter data values of sensors
 * 
 * @param x the newly sampled input signal
 * @param y_last the last output signal of this filter upon some sensor
 * @param alpha the alpha value of the exponenetial filter. higher the alpha, higher the cutoff frequency
 * @return float the current output signal of this filter upon the sensor of interest
 */
static inline float exp_filter(const float x, const float y_last, const float alpha) {
  return alpha*y_last + (1-alpha)*x;
}

#endif //_TRANSFER_FUNCTION_H
