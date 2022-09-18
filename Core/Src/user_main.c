/**
  ******************************************************************************
  * @file    user_main.c
  * @author  Tsou, Min-Lun from 2022 NTUR
  * @brief   custom user code that is separated from the automatically generated main.c
  *          This file should manage all the code that was initially put in main.c,
  *          including
  *           + user code in main() such as the super loop
  *           + interrupt callback functions
  *           + CAN filter initialization
  *
  @verbatim
  ==============================================================================
                    ##### Operation of the Code #####
  ==============================================================================
  [..]
  The mission of the front box STM32 is to
  (#) Gather all the data from all the sensors, the most important ones being APPS and BSE
  (#) send all the sensor data into the CAN bus
  (#) Rules?(CHECKED, NOT REQUIRED)EV.5.5.3, EV.5.7, T.4.2.5

  [..]
  The sensors that connects to the front box include:
  (+) 2 APPS, each connected to an ADC channel
  (+) 1 BSE, connected to an ADC channel
  (+) 1 brake system oil pressure sensor, connected to an ADC channel
  (+) 2 suspension travel sensors, each connected to an ADC channel
  (+) 1 steering encoder, connected to an SPI peripheral
  (+) 2 tire temperature sensors, connected to an I^2C master peripheral
  (+) 2 wheel speed sensors, each connected to a GPIO EXTI line
  (+) 2 micro switches, each connected to a GPIO EXTI line
  The sensors occupies the ADC, I^2C, SPI, and EXTI peripherals. Each of the peripherals operate
  independently, so the code written can be generally divided into 4 parts, each dedicated to
  each of them.

  [..](OIL&TRAVEL CODE UNFINISHED)
  The ADC operates as such:
  (#) ADC1 is initiated in CubeMX with an active DMA and 6 channels each connected to a sensor.
  (#) an array of 6 uint32_t is allocated for the DMA
  (#) HAL_ADC_Start_DMA() is called once in user_main(), then the DMA automatically puts the data
  	  into the array.
  (#) various transfer functions from analog_transfer_function.c are called in user_main() to
  	  convert the numbers back into the original values.
  (#) the converted values can simply be put in the CAN array.


  [..](CODE_UNFINISHED)
  The software for the hall sensors operate as such:
  (#) EXTI for falling edge detection is initiated in CubeMX for each sensor, along with timer3 in
  	  regular up-counting mode.
  (#) int hall_counter is allocated for each sensor, which increase by 1 by the software every
  	  time an EXTI ISR is called.
  (#) HAL_TIM_Base_Start_IT() is called once in user_main() to start the timer. Then, timer3
  	  overflow ISR collects the counted value in a fixed interval, puts the number in
  	  hall_counter_result, then resets the hall_counter.
  (#) we can work out the wheel speed by the timer interval before putting the value in the
  	  CAN array.

  [..](CODE_UNFINISHED)
  The rest of the sensors that operates through I^2C and SPI are called in the super-loop. The
  MCU can simply grab the data by calling the library API.

  [..]
  Some calculation must be done in order to obtain the original data for the different sensors in
  order for it to fit the format in the CAN frame. After that is done, the MCU would put everything
  into 2 8-byte arrays, then sends it all through the CAN peripheral.
  The CAN HAL API is called as follows:
  (#) CAN is initiated in CubeMX in normal mode with 1Mb/s baud.
  (#) TxHeader struct is allocated, along with TxMailbox(unused but required for HAL API) and
  	  an 8 byte CAN_TxData array.
  (#) HAL_CAN_Start() is called once in user_main().
  (#) set up the information for arbitration fields with the TxHeader struct, then call
  	  HAL_CAN_AddTxMessage once CAN_TxData is ready.

  @endverbatim
  */

/*private include*/
#include "main.h"
#include <stdio.h>
#include "user_main.h"
#include "amt22.h"
#include "MLX.h"
#include "analog_transfer_function.h"

/*to use printf outputs to UART1 to monitor the state of the MCU or not*/
//#define PRINTF_TEST_OUTPUT

/*to use the live expressions to monitor the states or not*/
#define USE_LIVE_EXPRESSIONS
#ifdef USE_LIVE_EXPRESSIONS
uint8_t live_APPS1_signal;
uint8_t live_APPS2_signal;
uint8_t live_BSE_signal;
#endif

/* auto-generated peripheral handler structure by MX.
 * First defined in main.c
 * Must be updated to match the defined structures in main.c everytime peripheral
 *  usage is changed.
 */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;


#define ADC_DMA_ARRAY_RANK_LTRAVEL 0	/*IN6,PA6*/
#define ADC_DMA_ARRAY_RANK_RTRAVEL 1	/*IN9,PB1*/
#define ADC_DMA_ARRAY_RANK_OILPRESSURE 2/*IN8,PB0*/
#define ADC_DMA_ARRAY_RANK_APPS1 3		/*IN2,PA2*/
#define ADC_DMA_ARRAY_RANK_APPS2 4		/*IN3,PA3*/
#define ADC_DMA_ARRAY_RANK_BSE 5		/*IN4,PA4*/

/*ADC1 DMA destination array, the corresponding rank is defined above.*/
static uint32_t ADC_value[6]={0};

/*Hall effect sensor counter variable, 0 for left , 1 for right
 * hal_counter is updated by the EXTI ISR from the hall sensors
 * hall_counter_result is updated by the timer ISR at a fixed interval to obtain the counting
 * rate
 * */
#define NUMBER_OF_HALL_SENSORS 2
static int hall_counter[NUMBER_OF_HALL_SENSORS]={0};
static int hall_counter_result[NUMBER_OF_HALL_SENSORS]={0};


/*CAN required custom variables*/
CAN_TxHeaderTypeDef TxHeader1={
		.IDE = CAN_ID_EXT,
		.ExtId = 0x080AD091,
		.RTR = CAN_RTR_DATA,
		.DLC = 8,
		.TransmitGlobalTime = DISABLE
};
CAN_TxHeaderTypeDef TxHeader2={
		.IDE = CAN_ID_EXT,
		.ExtId = 0x080AD092,
		.RTR = CAN_RTR_DATA,
		.DLC = 8,
		.TransmitGlobalTime = DISABLE
};
uint8_t CAN_TxData_1[8]={0};
uint8_t CAN_TxData_2[8]={0};
uint32_t TxMailbox2;
uint32_t TxMailbox1;
CAN_RxHeaderTypeDef RxHeader;
uint8_t CAN_RxData[4]={0};


/**
  * @brief  contains the part of the application instructions that is originally put in main(),
  * 		including the super loop.
  * 		Should be called once after all auto-gererated init functions in main()
  * @param  None
  * @retval None
  */
void user_main(){
	 /*ADC1 DMA mode Start*/
	 HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_value,6);

	 /*timer3 interrupt mode start, used in hall sensors calculations*/
	 HAL_TIM_Base_Start_IT(&htim3);

	/*CAN receive filter configuration "for testing purposes"*/
	  CAN_FilterTypeDef canfilterconfig = {
			  .FilterActivation = CAN_FILTER_ENABLE,
			  .SlaveStartFilterBank = 0,	// how many filters to assign to the CAN1 (master can)
			  .FilterBank = 7,				// which filter bank to use from the assigned ones
			  .FilterFIFOAssignment = CAN_FILTER_FIFO0,
			  .FilterMode = CAN_FILTERMODE_IDMASK,
			  .FilterScale = CAN_FILTERSCALE_32BIT,
			  .FilterIdHigh = 0x333<<5,
			  .FilterIdLow = 0,
			  .FilterMaskIdHigh = 0x333<<5,
			  .FilterMaskIdLow = 0x0000
	  };
	  if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig)!=HAL_OK){
		  Error_Handler();
	  }
	 /*turn on receiving interrupt, then starts the CAN module*/
	  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
		  Error_Handler();
	  }
	  HAL_CAN_Start(&hcan);
	  /*wait until the accel pedal is released, then zero the APPS*/
	  while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1));
	  APPS_calibration(ADC_value[ADC_DMA_ARRAY_RANK_APPS1],1);
	  APPS_calibration(ADC_value[ADC_DMA_ARRAY_RANK_APPS2],2);

	  /*super loop*/
	  while(1){
		  /*APPS and BSE raw value obtaining and test output */
		  uint32_t APPS1test = ADC_value[ADC_DMA_ARRAY_RANK_APPS1];
		  uint32_t APPS2test = ADC_value[ADC_DMA_ARRAY_RANK_APPS2];
		  uint32_t BSEtest = ADC_value[ADC_DMA_ARRAY_RANK_BSE];
		  /*the pinsare connected to NO pin on the switch, which is connected to Gnd, and the switch is pressed when
		   *the pedals are at the fully extended state. So, The boolean state of the pin matches whether the pedal
		   *is pressed or not.*/
		  uint8_t APPSmicro = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);
		  uint8_t BSEmicro = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		  /*APPS&BSE value preprocessing*/
		  uint8_t APPS1Value = APPS_transfer_function(APPS1test,1);
		  uint8_t APPS2Value = APPS_transfer_function(APPS2test,2);
		  uint8_t BSEValue = BSE_transfer_function(BSEtest);

		  /*wheel speed output*/
		  uint16_t wheel_speedL = wheel_speed_transfer_function(hall_counter_result[0]);
		  uint16_t wheel_speedR = wheel_speed_transfer_function(hall_counter_result[1]);

		  /*temp sensor MLX90614 read API */
		  uint8_t temp_L1 = tire_temp_transfer_function( MLX90614_ReadReg(0x5A,0x08,0) );
		  //uint8_t temp_L2=tire_temp_transfer_function( MLX90614_ReadReg(0x5B,0x08,0) );
		  //uint8_t temp_L3=tire_temp_transfer_function( MLX90614_ReadReg(0x5C,0x08,0) );
		  //uint8_t temp_L4=tire_temp_transfer_function( MLX90614_ReadReg(0x5D,0x08,0) );
		 // printf("%.2f C \r\n",MLX90614_ReadReg(0x5A,0x06,0)*0.02-273.15);
		 // printf("%.2f C \r\n",MLX90614_ReadReg(0x5A,0x07,0)*0.02-273.15);
		 // printf("%.2f C \r\n",MLX90614_ReadReg(0x5A,0x08,0)*0.02-273.15);
		 // printf("%x\r\n",MLX90614_ReadReg(0x5A,0x08,0));

		  /*grabbing the suspension travel data*/
		  uint8_t travel_L = suspension_travel_transfer_function(ADC_value[ADC_DMA_ARRAY_RANK_LTRAVEL]);
		  uint8_t travel_R = suspension_travel_transfer_function(ADC_value[ADC_DMA_ARRAY_RANK_RTRAVEL]);

		  /*grabbing the oil pressure sensor data*/
		  uint8_t oil_pressure = oil_pressure_transfer_function(ADC_value[ADC_DMA_ARRAY_RANK_OILPRESSURE]);

		  /*grab the absolute encoder data TODO: needs to be tested*/
		  uint16_t amt22_pos = getPositionSPI(&hspi2, GPIOC, GPIO_PIN_14, 12);

#ifdef USE_LIVE_EXPRESSIONS
		  live_APPS1_signal = APPS1Value;
		  live_APPS2_signal = APPS2Value;
		  live_BSE_signal = BSEValue;
#endif

		  /*loading data into message array*/
		  CAN_TxData_1[0] = (uint8_t)(wheel_speedL>>8);
		  CAN_TxData_1[1] = (uint8_t)(wheel_speedL & 0x00FF);
		  CAN_TxData_1[2] = (uint8_t)(wheel_speedR>>8);
		  CAN_TxData_1[3] = (uint8_t)(wheel_speedR & 0x00FF);
		  CAN_TxData_1[4] = temp_L1;

		  CAN_TxData_2[0] = BSEValue;

		  /*check whether the 2 APPS read different values*/
		  if(APPS1Value!=255 && APPS2Value!=255 && APPS1Value!=0 && APPS2Value!=0){
			  int deviation = (int)APPS1Value-(int)APPS2Value;
			  if(deviation <= 25 && deviation >= -25){
				  CAN_TxData_2[1] = APPS1Value;
				  CAN_TxData_2[2] = APPS2Value;
			  }
			  else{
				  CAN_TxData_2[1] = 0;
				  CAN_TxData_2[2] = 0;
			  }
		  }

		  CAN_TxData_2[3] = (uint8_t)(amt22_pos>>4);
		  CAN_TxData_2[4] = travel_L;
		  CAN_TxData_2[5] = travel_R;
		  CAN_TxData_2[6] = oil_pressure;
		  CAN_TxData_2[7] = (APPSmicro|(BSEmicro<<1)); //bit0 contains APPS switch data, bit1 contains BSE switch data
		  /*the CAN transmit HAL API*/
		  HAL_CAN_AddTxMessage(&hcan,&TxHeader1,CAN_TxData_1,&TxMailbox1);
		  HAL_CAN_AddTxMessage(&hcan,&TxHeader2,CAN_TxData_2,&TxMailbox2);

		  if(HAL_CAN_GetError(&hcan)==HAL_CAN_ERROR_BOF){
			  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		  }

		  /*test printf output*/
#ifdef PRINTF_TEST_OUTPUT
		  printf("%ld,%ld,%ld,%ld,%ld,%ld\r\n",ADC_value[0],ADC_value[1],ADC_value[2],APPS1test,APPS2test,BSEtest);
		  printf("APPS micro: %d\r\n",APPSmicro);
		  printf("BSE micro: %d\r\n",BSEmicro);
		  printf("APPS1: %d\r\n",APPS1Value);
		  printf("APPS2: %d\r\n",APPS2Value);
		  printf("BSE: %d\r\n",BSEValue);
		  printf("tire temp L1: %.2f\r\n",temp_L1/2.0);
		  printf("brake oil pressure: %d\r\n",oil_pressure);
		  printf("left wheel speed is %d rpm\n",wheel_speedL);
		  printf("right wheel speed is %d rpm\n",wheel_speedR);
#endif
		  /*superloop execution interval*/
		  HAL_Delay(20);
	  }/*while(1)*/
}

/**
  * @brief  User defined EXTI interrupt callback function, namely EXTI ISR.
  * 		Shall only be called by HAL interrupt handlers
  * @param  GPIO_PIN: the GPIO pin that generated the interrupt.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	switch (GPIO_PIN){
		case GPIO_PIN_5: /*right wheel hall sensor*/
			hall_counter[1]++;
#ifdef PRINTF_TEST_OUTPUT
			printf("EXTI5:%d\n",hall_counter[1]);
#endif
			break;
		case GPIO_PIN_7: /*left wheel hall sensor*/
			hall_counter[0]++;
#ifdef PRINTF_TEST_OUTPUT
			printf("EXTI7:%d\n",hall_counter[0]);
#endif
			break;
		case GPIO_PIN_0:
			printf("EXTI0\n");
			break;
		case GPIO_PIN_1:
			printf("EXTI1\n");
			break;
		case GPIO_PIN_15:
			printf("EXTI15\n");
			break;
	}
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	return;
}

/**
  * @brief  User defined timer overflow interrupt callback function.
  * 		Shall only be called by HAL interrupt handlers.
  * @note 	For timer3, which should update in a fixed interval defined in CubeMX, we grab
  * 		the current hall sensor counts, then reset it.
  * @param  htim: the timer that generated the interrupt.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim3){
		int i=0;
		for(i=0;i<sizeof(hall_counter)/sizeof(hall_counter[0]);i++){
			hall_counter_result[i]=hall_counter[i];
			hall_counter[i]=0;
		}
	}
	return;
}

/**
  * @brief  User defined CAN receiving interrupt callback function, namely CAN receiving ISR.
  * 		Shall only be called by HAL interrupt handlers
  * @param  hcan: the can handle structure that received the message.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN_RxData) != HAL_OK){
    Error_Handler();
  }
   if (CAN_RxData[0]==0x01&&CAN_RxData[1]==0x02&&CAN_RxData[2]==0x03&&CAN_RxData[3]==0x04){
	  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
  }

   return;
}
