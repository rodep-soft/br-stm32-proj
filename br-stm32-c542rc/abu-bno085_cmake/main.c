/**
  ******************************************************************************
  * file           : main.c
  * brief          : Main program body
  *                  Calls target system initialization then loop in main.
  ******************************************************************************
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>


// printfをUSART2にリダイレクトするための_write関数の実装
int _write(int file, char *ptr, int len) {
    hal_uart_handle_t *huart = mx_usart2_uart_gethandle();
    
    // タイムアウトを適切に設定して送信（HAL_MAX_DELAY または任意のミリ秒）
    if (HAL_UART_Transmit(huart, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY) != HAL_OK)
    {
        return -1;
    }
    return len;
}

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions prototype -----------------------------------------------*/

/**
  * brief:  The application entry point.
  * retval: none but we specify int to comply with C99 standard
  */
int main(void)
{
  
  /** System Init: this code placed in targets folder initializes your system.
    * It calls the initialization (and sets the initial configuration) of the peripherals.
    * You can use STM32CubeMX to generate and call this code or not in this project.
    * It also contains the HAL initialization and the initial clock configuration.
    */
  if (mx_system_init() != SYSTEM_OK)
  {
    return (-1);
  }
  else
  {
    /*
      * You can start your application code here
      */
    setvbuf(stdout, NULL, _IONBF, 0);
    while (1) {
      printf("Hello World!\n");
      HAL_Delay(1000); 
    }
  }
} /* end main */

