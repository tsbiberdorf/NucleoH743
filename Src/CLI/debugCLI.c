/*
 * debugCLI.c
 *
 *  Created on: Jul 25, 2019
 *      Author: TBiberdorf
 */

#include <stdio.h>
#include "stm32h7xx_hal_uart.h"

/* Private variables ---------------------------------------------------------*/
uint8_t aTxStartMessage[] = "\r\n****UART-Hyperterminal communication based on IT ****\r\nEnter 10 characters using keyboard :\r\n";

/* Buffer used for reception */
uint8_t aRxBuffer[20];
UART_HandleTypeDef *ptrUartHandler;

/* IRQ Handlers    ---------------------------------------------------------*/

/**
  * @brief Rx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file
   */
    HAL_UART_Transmit(&huart1, (uint8_t *)aRxBuffer, 10,0xFFFF);
}


/* Public Methods ------------------------------------------------------------*/


void DebugCliSetUartHandler(UART_HandleTypeDef *ptrHandler)
{
	ptrUartHandler = ptrHandler;
}
