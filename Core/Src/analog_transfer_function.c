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
static const float pi = 3.1415927;
static int8_t APPS_calibration_value_1 = 2;
static int8_t APPS_calibration_value_2 = 2;

/**
  * @brief  calibration function for APPS transfer functions by setting the read value to 0 linearly
  * @param  reading: the raw ADC 12bit number read when the pedal is at rest
  * @retval none
  */
void APPS_calibration(uint32_t reading, uint8_t sensor_number){
	float value;
	if(sensor_number==1){
		value = APPS1_conversion(reading);
		APPS_calibration_value_1 += (int8_t)value;
	}
	if(sensor_number==2){
		value = APPS2_conversion(reading);
		APPS_calibration_value_2 += (int8_t)value;
	}

	return;
}

/**
 *  @brief	mathematical transfer function for the APPS1 potentiometer
 *  @param	reading the raw 12 bit ADC reading
 *  @retval	the relative ratio for how much of the accel pedal has been stepped, times 254
 */
float APPS1_conversion(uint32_t reading){
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
		return value;
}

/**
 *  @brief	mathematical transfer function for the APPS2 potentiometer
 *  @param	reading the raw 12 bit ADC reading
 *  @retval	the relative ratio for how much of the accel pedal has been stepped, times 254
 */
float APPS2_conversion(uint32_t reading){
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
	return value;
}

/**
  * @brief  transfer function for the two analog APPS on ep4
  * @param  reading: the raw ADC 12bit number
  * @param	sensor_number: the index for the sensor, 1 for APPS1 and so on
  * @retval value: the relative ratio for how much pedal travel is being pressed times 254,
  * 		rounding down.
  */
uint8_t APPS_transfer_function(uint32_t reading, uint8_t sensor_number){
	const float out_of_bounds_tolerance = 4.0;
	/*calculating the theoretical pedal press*/
	float value;
	if(sensor_number!=1&&sensor_number!=2) {return 0;}
	if(sensor_number==1){
		/*compensating the values read from the sensors after testing*/
		value = APPS1_conversion(reading);
		value -= APPS_calibration_value_1;
	}
	else{
		/*compensating the values read from the sensors after testing*/
		value = APPS2_conversion(reading);
		value -= APPS_calibration_value_2;
	}

	/*snapping everything out of bounds to designated values*/
	if(value>=0 && value<254)	{return (uint8_t)value+1;}
	if(value<0){
		if(value < -out_of_bounds_tolerance)	{return 0;}
		else 									{return 1;}
	}
	else{
		if(value >= 254.0+out_of_bounds_tolerance) 	{return 255;}
		else										{return 254;}
	}
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
	 *	(sqrt{(7767369a^{2}-10940888ax+7074144x^{2})}-2787a+812x)/(2(812x-812a)) (a being max_adc_value)
	 * However, since we only use 2.5~24.5mm part of the domain instead of the full 0~25, we have
	 * to scale the number to fit the proportions as well
	 *
	 * Note: 24.5mm corresponds to 0% pedal, and 2.5mm corresponds to 100% pedal travel.
	 * */
	const float out_of_bounds_tolerance = 4.0;
	float value;
	float x=(float)reading;
	float a = max_adc_value;
	value = (sqrt(7767369*a*a - 10940888*a*x + 7074144*x*x) - 2787*a + 812*x)/(2*(812*x - 812*a));
	value = (value-(25-24.5)/25) * (25)/(24.5-2.5);
	value *= 254;
	/*compensating the read value linearly*/
	value -= 27;
	value *= 254/185;

	/*snapping everything out of bounds to designated values*/
	if(value>=0 && value<254)	{return (uint8_t)value+1;}
	if(value<0){
		if(value < -out_of_bounds_tolerance)	{return 0;}
		else 									{return 1;}
	}
	else{
		if(value >= 254.0+out_of_bounds_tolerance) 	{return 255;}
		else										{return 254;}
	}
}

/**
  * @brief  transfer function for the brake oil pressure sensor on ep4
  * @param  reading: the raw ADC 12bit number
  * @retval value: the 8 bit number reperesenting the suspension travel that matches the format on the CAN protocol.
  */
uint8_t oil_pressure_transfer_function(uint32_t reading){
	/*TO BE DETERMINED not sure if it is the correct transfer function
	 * assume linear transfer:
	 * sensor outputs 1~5V, which is mapped to 0~5kPar
	 * 0~5kPar is mapped linearly to 0~255 in the CAN protocol*/
	float value=0;
	float input = reading;
	value = (input - 4096.0/5)*(255 /(4096 *(4.0/5.0) ) );

	if(value>=256)		{return 255;}
	else if(value<=0)	{return 0;}
	else				{return (uint8_t)value;}
}

/**
  * @brief  transfer function for the analog suspension travel on ep4
  * @param  reading: the raw ADC 12bit number
  * @retval value: the 8 bit number reperesenting the suspension travel that matches the format on the CAN protocol.
  */
uint8_t suspension_travel_transfer_function(uint32_t reading){

	/*The length of the suspension measuring sensor goes from 200~245mm, which corresponds to 5.5~50.5mm extension
	 * The length of the allowable extension is mapped linearly to 0~255
	 * that is, the function should be a straight line passing through( 5.5*(4096/75) , 0 ) and (50.5*(4096/75),256)
	 */

	float value = 0.0;
	float input = (float)reading;
	value = (input-5.5*(max_adc_value/75))*(256/(max_adc_value*(50.5-5.5)/75));

	if(value>=256)		{return 255;}
	else if(value<=0)	{return 0;}
	else				{return (uint8_t)value;}
}

/**
  * @brief  transfer function for the hall tachometer on ep4
  * @param  reading: the number of hall trigger per 10ms
  * @retval the wheel speed in rad/s, times 256
  * @note the result is multiplied by 256 so that the MSB represents the integer part of the number while LSB represents the part less than 1
  */
uint16_t wheel_speed_transfer_function(uint32_t reading){
	/**/
	float input = reading;
	const float tooth_per_rev = 1.0; /*	TO BE DETERMINED*/
	float value = 0.0;
	value = input *100 /tooth_per_rev *pi *256;
	return (uint16_t)value;

}

/**
  * @brief  transfer function for the IR tire temperature sensor on ep4
  * @param  the absolute measured temperature, times 50(the direct output of the MLX90614 sensors)
  * @retval the tire temperature in degree C, times 2
  */
uint8_t tire_temp_transfer_function(uint16_t reading){
	float value;
	float input = reading;
	value = (input/50-273.15)*2;

	if(value>=256)		{return 255;}
	else if(value<=0)	{return 0;}
	else				{return (uint8_t)value;}
}
