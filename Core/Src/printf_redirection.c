/**
  ******************************************************************************
  * @file    printf_redirection.c
  * @author  Tsou, Min-Lun from 2022 NTUR
  * @brief   redirecting where printf would output to by overwriting the definition for fputc or
  * 		 putchar
  @verbatim
  ==============================================================================
                    ##### Operation of the Code #####
  ==============================================================================
  [..]
  fputc(for MDK) or putchar(CubeIDE) is redefined to direct the output char to huart1 in order to
  catch the output message by a virtual COM port on a seperate PC with some additional hardware.
  The function can be rewritten to let printf output to a different location for debugging.


  @endverbatim
  */



#include "stdio.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

/**
  * @brief  redirection of printf function by forcing fputc or put_char to output to HAL_UART instead
  * @param  ch: see the original stdio.h description
  * @param  f: see the original stdio.h description
  * @retval None
  */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE{
	//print to UART
//	HAL_UART_Transmit(&huart1,(uint8_t*)&ch, 1, 0xFFFF);
//	return ch;

	//print to SWV
	// Write character to ITM ch.0
	 ITM_SendChar(ch);
	 return(ch);
}
