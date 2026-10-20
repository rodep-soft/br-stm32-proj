/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "bno08x.h"
#include "bno085_can.h"
#include "can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile bool bno_data_ready = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
//static void CAN_LoopbackTest(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static sh2_SensorValue_t sensor_value;
static sh2_SensorValue_t last_accel;
static sh2_SensorValue_t last_gyro;

/* ===== CANループバックテスト設定 ===== */
// #define CAN_TEST_STD_ID     0x100U   /* 送信CAN ID (DBCのBNO085_GyroZと同一) */
// #define CAN_TEST_DLC        8U       /* データ長 (byte) */
// #define CAN_TEST_DATA_BYTE  0xFFU    /* テストデータ (全バイト0xFF) */
// #define CAN_TEST_PERIOD_MS  1000U    /* whileループ内で再送する間隔 (ms) */
// #define CAN_TEST_PERIODIC   1        /* 1: 周期送信も行う / 0: 起動時1回のみ */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\n========================================\r\n");
  printf("  STM32F303K8 BNO085 Auto-Recovery Test\r\n");
  printf("========================================\r\n");

  /* ---- CAN開始 (フィルタ: 全ID受け入れ, ループバックモード) ---- */
  {
    CAN_FilterTypeDef can_filter = {0};
    can_filter.FilterBank           = 0;
    can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh         = 0x0000;
    can_filter.FilterIdLow          = 0x0000;
    can_filter.FilterMaskIdHigh     = 0x0000;   /* マスク0 = すべて通す */
    can_filter.FilterMaskIdLow      = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation     = ENABLE;
    can_filter.SlaveStartFilterBank = 14;       /* F3はシングルCANなのでダミー値 */

    if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK)
    {
      printf("[ERROR] HAL_CAN_ConfigFilter failed\r\n");
    }
    else if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
      printf("[ERROR] HAL_CAN_Start failed\r\n");
    }
    else
    {
      printf("[OK] CAN started \r\n");
      //CAN_LoopbackTest();   /* 起動時に1回テスト */
    }
  }

  bool bno_initialized = false;

  printf("Initializing BNO085...\r\n");
  int init_status = bno08x_init();
  if (init_status == 0)
  {
    printf("[OK] BNO085 initialized successfully!\r\n");
    bno08x_enable_report(SH2_ACCELEROMETER, 5000);
    bno08x_enable_report(SH2_GYROSCOPE_CALIBRATED, 5000);

    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));
    sh2_getSensorConfig(SH2_ACCELEROMETER, &config);

    printf("!!!!!!![OK] Accelerometer config: sensitivity=%f, reportInterval_us=%u\r\n", config.changeSensitivity, config.reportInterval_us);

    //printf("[OK] Enabled Accelerometer & Gyroscope reports (5ms)\r\n");
    bno_initialized = true;
  }
  else
  {
    printf("[WARN] BNO085 init failed (code: %d), will retry in main loop...\r\n", init_status);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */




    // 【自動再起処理】初期化できていない場合は1秒おきにリトライする
    if (!bno_initialized)
    {
      static uint32_t last_retry = 0;
      uint32_t now = HAL_GetTick();
      if ((now - last_retry) >= 1000)
      {
        last_retry = now;
        printf("Retrying BNO085 initialization...\r\n");
        int retry_status = bno08x_init();
        if (retry_status == 0)
        {
          printf("[OK] BNO085 Recovered & Initialized!\r\n");
          bno08x_enable_report(SH2_ACCELEROMETER, 5000);
          bno08x_enable_report(SH2_GYROSCOPE_CALIBRATED, 5000);
          printf("[OK] Enabled Accelerometer & Gyroscope reports (5ms)\r\n");
          bno_initialized = true;
        }
        else
        {
          printf("[ERROR] Retry failed (code: %d), will try again in 1s.\r\n", retry_status);
        }
      }
      continue;
    }

    // --- ここからは初期化成功後の通常処理 ---

    if (bno08x_was_reset())
    {
      printf("[WARN] BNO085 reset detected! Re-enabling reports...\r\n");
      bno08x_enable_report(SH2_ACCELEROMETER, 5000);
      bno08x_enable_report(SH2_GYROSCOPE_CALIBRATED, 5000);
    }

    static bool has_accel = false;
    static bool has_gyro = false;

    if (bno_data_ready) {
      bno_data_ready = false;
    
      if (bno08x_get_sensor_event(&sensor_value))
      {

        if (sensor_value.sensorId == SH2_ACCELEROMETER)
        {
          last_accel = sensor_value;
          has_accel = true;
        }
        else if (sensor_value.sensorId == SH2_GYROSCOPE_CALIBRATED)
        {
          last_gyro = sensor_value;
          has_gyro = true;
        }
      }
    }
    
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tick >= BNO085_CAN_BNO085_DATA_CYCLE_TIME_MS && (has_accel || has_gyro))
    {
        last_tick = now;
        can_send(&last_gyro, &last_accel, &hcan);
        has_accel = false;
        has_gyro = false;
        //CAN_LoopbackRecieve(&hcan);
// //メールボックスに空きがあるか確認 
//       if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
//       {
//         last_tick = now;
//         can_send(&last_gyro, &last_accel, &hcan);
//         has_accel = false;
//         has_gyro = false;
//         //CAN_LoopbackRecieve(&hcan);
//       }
//       else
//       {
//         // 送信メールボックスが詰まっている（相手がACKを返していない等）場合の警告 
//         printf("[WARN] CAN Tx Mailbox Full!\r\n");
//       }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* EXTI0 interrupt init for H_INTN */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
  if (ch == '\n')
  {
    uint8_t cr = '\r';
    HAL_UART_Transmit(&huart2, &cr, 1, 10);
  }
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
  return ch;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
    bno_data_ready = true;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
