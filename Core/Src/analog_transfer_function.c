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
	//4.50
	/*The transformation from stepped ratio to voltage is
	 * reading = y=adc_max_value*(4500x)/(4500x+((4500(1-x)+997)*3900)/((4500*(1-x)+997)+3900))
	 * where x is the ratio of the pressed displacement and the max displacement of the sensor
	 * the inverse for the desired domain and range is
	 * (9397*a-5497*x)/(9000*(a-x))-sqrt(88303609*a*a-189063818*a*x+115970209*x*x)/(9000*(a-x)),
	 * where a is max_adc_value
	 * However, since we only use 2.5~39.5mm part of the domain instead of the full 0~50, we have
	 * to scale the number to fit the proportions as well
	 *
	 * Note: 39.5mm corresponds to 0% pedal, and 2.5mm corresponds to 100% pedal travel.
	 * */
	float value;
	float x = (float)reading;
	float a = max_adc_value;
	value = (9397*a-5497*x)/(9000*(a-x))-sqrt(88303609*a*a-189063818*a*x+115970209*x*x)/(9000*(a-x));
	value = (value-(50-39.5)/50) * (50)/(37);
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
	//5.10k
	/*The transformation from stepped ratio to voltage is
	 * reading = y = ADC_MAX_VALUE*(5100x)/(5100x+((5100(1-x)+200)*3890)/((5100*(1-x)+200)+3890))
	 * where x is the ratio of the pressed displacement and the max displacement of the sensor
	 * the inverse for the desired domain and range is
	 * (919 a - 530 x)/(1020 (a - x)) ± sqrt(844561 a^2 - 1798820 a x + 1105580 x^2)/(1020 (a - x)) (a being max_adc_value)
	 * However, since we only use 2.5~39.5mm part of the domain instead of the full 0~50, we have
	 * to scale the number to fit the proportions as well
	 *
	 * Note: 39.5mm corresponds to 0% pedal, and 2.5mm corresponds to 100% pedal travel.
	 * */
	float value;
	float x = (float)reading;
	float a = max_adc_value;
	value = (919*a - 530*x)/(1020*(a - x)) - sqrt(844561*a*a - 1798820*a*x + 1105580*x*x)/(1020*(a - x));
	value = (value-(50-39.5)/50) * (50)/(37);
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
	 * reading = y=a*(1624x)/(1624x+((1624(1-x))*3950)/((1624*(1-x))+3950))
	 * where x is the ratio of the pressed displacement and the max displacement of the sensor
	 * the inverse for the desired domain and range is
	 *(sqrt{(7767369a^{2}-10940888ax+7074144x^{2})}-2787a+812x)/(2(812x-812a)) (a being max_adc_value)
	 * However, since we only use 2.5~24.5mm part of the domain instead of the full 0~25, we have
	 * to scale the number to fit the proportions as well
	 *
	 * Note: 24.5mm corresponds to 0% pedal, and 2.5mm corresponds to 100% pedal travel.
	 * */
	float value;
	float x=(float)reading;
	float a = max_adc_value;
	value = (sqrt(7767369*a*a - 10940888*a*x + 7074144*x*x) - 2787*a + 812*x)/(2*(812*x - 812*a));
	value = (value-(50-24.5)/25) * (25)/(24.5-2.5);
	value *= 254;
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
