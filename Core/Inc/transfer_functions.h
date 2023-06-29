#ifndef _TRANSFER_FUNCTIONS_H
#define _TRANSFER_FUNCTIONS_H

#include <stdint.h>

static inline float APPS1_transfer_function(const uint16_t reading, const float);
static inline float APPS2_transfer_function (const uint16_t reading, const float);
static inline float BSE_transfer_function(const uint16_t reading, float);
static inline float tire_temp_transfer_function(const uint8_t high, const uint8_t low);
static inline float oil_transfer_function(const uint16_t reading);
static inline float travel_transfer_function (const uint16_t reading);
float fuzzy_edge_remover(const float raw, const float highEdge, const float lowEdge);

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
    return (((float)reading * 3.3/3)*4 - 1000) * (70)/(15000-1000);
}

static inline float steer_angle_transfer_function(const uint16_t reading) {
    //see https://www.cuidevices.com/product/resource/amt22.pdf
    //our model outputs 12bit number
    return (float)reading * 360.0/4096.0;
}

#endif //_TRANSFER_FUNCTION_H