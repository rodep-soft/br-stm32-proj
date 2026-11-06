/**
  ******************************************************************************
  * @file           : mx_gpio_default.c
  * @brief          : gpio_default Peripheral initialization
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the mx_stm32c5xx_hal_drivers_license.md file
  * in the same directory as the generated code.
  * If no mx_stm32c5xx_hal_drivers_license.md file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mx_gpio_default.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported variables by reference -------------------------------------------*/

/******************************************************************************/
/* Exported functions for GPIO in HAL layer                                   */
/******************************************************************************/
system_status_t mx_gpio_default_init(void)
{
  hal_gpio_config_t  gpio_config;

  HAL_RCC_GPIOA_EnableClock();

  HAL_RCC_GPIOB_EnableClock();

  HAL_RCC_GPIOE_EnableClock();

  /*
    GPIO pin labels :
    PA0   ---------> PA0
    */
  /* Configure PA0 GPIO pin in input mode */
  gpio_config.mode            = HAL_GPIO_MODE_INPUT;
  gpio_config.pull            = HAL_GPIO_PULL_NO;
  if (HAL_GPIO_Init(PA0_PORT, PA0_PIN, &gpio_config) != HAL_OK)
  {
    return SYSTEM_PERIPHERAL_ERROR;
  }

  /*
    GPIO pin labels :
    PA1   ---------> PA1
    PA4   ---------> PA4
    */
  /* Configure PA1, PA4 GPIO pins in output mode */
  gpio_config.mode            = HAL_GPIO_MODE_OUTPUT;
  gpio_config.speed           = HAL_GPIO_SPEED_FREQ_LOW;
  gpio_config.pull            = HAL_GPIO_PULL_NO;
  gpio_config.output_type     = HAL_GPIO_OUTPUT_PUSHPULL;
  gpio_config.init_state      = HAL_GPIO_PIN_SET;
  if (HAL_GPIO_Init(HAL_GPIOA, PA1_PIN | PA4_PIN, &gpio_config) != HAL_OK)
  {
    return SYSTEM_PERIPHERAL_ERROR;
  }

  /*
    GPIO pin labels :
    PB0   ---------> PB0
    */
  /* Configure PB0 GPIO pin in output mode */
  gpio_config.mode            = HAL_GPIO_MODE_OUTPUT;
  gpio_config.speed           = HAL_GPIO_SPEED_FREQ_LOW;
  gpio_config.pull            = HAL_GPIO_PULL_NO;
  gpio_config.output_type     = HAL_GPIO_OUTPUT_PUSHPULL;
  gpio_config.init_state      = PB0_INIT_STATE;
  if (HAL_GPIO_Init(PB0_PORT, PB0_PIN, &gpio_config) != HAL_OK)
  {
    return SYSTEM_PERIPHERAL_ERROR;
  }

  /*
    GPIO pin labels :
    PE2   ---------> PE2
    */
  /* Configure PE2 GPIO pin in output mode */
  gpio_config.mode            = HAL_GPIO_MODE_OUTPUT;
  gpio_config.speed           = HAL_GPIO_SPEED_FREQ_LOW;
  gpio_config.pull            = HAL_GPIO_PULL_NO;
  gpio_config.output_type     = HAL_GPIO_OUTPUT_PUSHPULL;
  gpio_config.init_state      = PE2_INIT_STATE;
  if (HAL_GPIO_Init(PE2_PORT, PE2_PIN, &gpio_config) != HAL_OK)
  {
    return SYSTEM_PERIPHERAL_ERROR;
  }

  return SYSTEM_OK;
}

system_status_t mx_gpio_default_deinit(void)
{
  /* De-initialize pins of GPIOA port */
  HAL_GPIO_DeInit(HAL_GPIOA, PA0_PIN | PA1_PIN | PA4_PIN);

  /* De-initialize pins of GPIOB port */
  HAL_GPIO_DeInit(PB0_PORT, PB0_PIN);

  /* De-initialize pins of GPIOE port */
  HAL_GPIO_DeInit(PE2_PORT, PE2_PIN);

  return SYSTEM_OK;
}
