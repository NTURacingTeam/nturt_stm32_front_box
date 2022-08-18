/**
  ******************************************************************************
  * @file    analog_transfer_function.c
  * @author  Tsou, Min-Lun from 2022 NTUR
  * @brief   (inverse) transfer function for all the analog sensors on epsilon 4, including
  *           + APPS
  *           + BSE
  *           + oil pressure sensor
  *           + suspension travel sensor
  *
  @verbatim
  ==============================================================================
                    ##### Operation of the Code #####
  ==============================================================================
  [..]
  All transfer functions take the raw 12bit value from the ADCs, calculate the original measured
  value base on the sensors' datasheet and the used resistors, then outputs an 8 bit number that
  fits the format on the designated CAN protocol.

  @endverbatim
  */

#include "math.h"
#include "analog_transfer_function.h"
#include "stdio.h"

/*need to make sure, not quite 4096*/
static const float max_adc_value=4096.0;


/**
  * @brief  transfer function for the analog APPS1 on ep4
  * @param  reading: the raw ADC 12bit number
  * @retval value: the relative ratio for how much pedal travel is being pressed times 254,
  * 		rounding down
  */
uint8_t APPS1_transfer_function(uint32_t reading){
	/*The transformation from stepped ratio to voltage is
	 * reading = y=adc_max_value*(5000x)/(5000x+((5000(1-x)+200)*3900)/((5000*(1-x)+200)+3900))
	 * where x is the ratio of the pressed displacement and the max displacement of the sensor
	 * the inverse for the desired domain and range is
	 * (13 (7 a - 4 x))/(100 (a - x)) - (13 sqrt(49 a^2 - 104 a x + 64 x^2))/(100 (a - x)),
	 * where a is max_adc_value
	 * However, since we only use 2.5~39.5mm part of the domain instead of the full 0~50, we have
	 * to scale the number to fit the proportions as well
	 *
	 * Note: the equation is based on the rated values, actual resistance needs to be measured.
	 * */
	float value;
	float x = (float)reading;
	float a = max_adc_value;
	value = (13*(7*a - 4*x))/(100*(a - x)) - (13*sqrt(49*a*a - 104*a*x + 64*x*x))/(100*(a - x));
	value = (value-2.5/50) * (50)/(37);
	value = value*254;
	/*snapping everything out of bounds to designated values*/
	value+=1;
	if(value<1)			{return 0;}
    else if(value>=255)	{return 255;}
    else 				{return (uint8_t)value;}
}

/**
  * @brief  transfer function for the analog APPS2 on ep4
  * @param  reading: the raw ADC 12bit number
  * @retval value: the relative ratio for how much pedal travel is being pressed times 254,
  * 		rounding down.
  */
uint8_t APPS2_transfer_function(uint32_t reading){
	/*The transformation from stepped ratio to voltage is
	 * reading = ADC_MAX_VALUE*(5000x)/(5000x+((5000(1-x)+1000)*3900)/((5000*(1-x)+1000)+3900))
	 * where x is the ratio of the pressed displacement and the max displacement of the sensor
	 * the inverse for the desired domain and range is
	 * (3 (33 a - 20 x))/(100 (a - x)) - (3 sqrt(1089 a^2 - 2360 a x + 1440 x^2))/(100 (a - x)) (a being max_adc_value)
	 * However, since we only use 2.5~39.5mm part of the domain instead of the full 0~50, we have
	 * to scale the number to fit the proportions as well
	 *
	 * Note: the equation is based on the rated values, actual resistance needs to be measured.
	 * */
	float value;
	float x = (float)reading;
	float a = max_adc_value;
	value = (3*(33*a - 20*x))/(100*(a - x)) - (3*sqrt(1089*a*a - 2360*a*x + 1440*x*x))/(100*(a - x));
	value = (value-2.5/50) * (50)/(37);
	value = value*254;
	/*snapping everything out of bounds to designated values*/
	value+=1;
	if(value<1)			{return 0;}
	else if(value>=255)	{return 255;}
	else 				{return (uint8_t)value;}
}

/**
  * @brief  transfer function for the analog BSE on ep4
  * @param  reading: the raw ADC 12bit number
  * @retval value: the relative ratio for how much pedal travel is being pressed times 254,
  * 		rounding down.
  */
uint8_t BSE_transfer_function(uint32_t reading){
	/*The transformation from stepped ratio to voltage is
	 * reading = y=adc_max_value*(5000x)/(5000x+((5000(1-x)+200)*3900)/((5000*(1-x)+200)+3900))
	 * where x is the ratio of the pressed displacement and the max displacement of the sensor
	 * the inverse for the desired domain and range is
	 * (13 (7 a - 4 x))/(100 (a - x)) - (13 sqrt(49 a^2 - 104 a x + 64 x^2))/(100 (a - x)) (a being max_adc_value)
	 * However, since we only use 27.5~49.5mm part of the domain instead of the full 0~50, we have
	 * to scale the number to fit the proportions as well
	 *
	 * Note: the equation is based on the rated values, actual resistance needs to be measured.
	 * */
	float value;
	float x=(float)reading;
	float a = max_adc_value;
	value = (13*(7*a - 4*x))/(100*(a - x)) - (13*sqrt(49*a*a - 104*a*x + 64*x*x))/(100*(a - x));
	value = (value-27.5/50) * (50)/(22);
	value = value*254;
	/*snapping everything out of bounds to designated values*/
	value+=1;
	if(value<1)			{return 0;}
	else if(value>=255)	{return 255;}
	else 				{return (uint8_t)value;}
}

uint8_t oil_pressure_transfer_function(uint32_t reading){
	uint8_t value=0;

	return value;
}

uint8_t suspension_travel_transfer_function(uint32_t reading){
	uint8_t value=0;

	return value;
}
