/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "sct.h"
#include "1wire.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTERT_T_DELAY 750
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static int16_t adc_values[] = {168.9,166.9,164.9,163,161.1,159.3,157.5,155.8,154,152.4,150.7,149.1,147.5,146,144.5,143,141.5,140.1,138.7,137.3,136,134.7,133.4,132.2,130.9,129.7,128.5,127.4,126.3,125.1,124.1,123,122,120.9,119.9,119,118,117.1,116.1,115.2,114.4,113.5,112.6,111.8,111,110.2,109.4,108.6,107.9,107.1,106.4,105.7,105,104.3,103.6,103,102.3,101.7,101.1,100.5,99.9,99.3,98.7,98.1,97.6,97,96.5,96,95.4,94.9,94.4,93.9,93.4,93,92.5,92,91.6,91.1,90.7,90.2,89.8,89.4,89,88.6,88.2,87.8,87.4,87,86.6,86.2,85.9,85.5,85.1,84.8,84.4,84.1,83.7,83.4,83.1,82.7,82.4,82.1,81.8,81.4,81.1,80.8,80.5,80.2,79.9,79.6,79.3,79,78.8,78.5,78.2,77.9,77.6,77.4,77.1,76.8,76.6,76.3,76,75.8,75.5,75.3,75,74.8,74.5,74.3,74,73.8,73.5,73.3,73.1,72.8,72.6,72.3,72.1,71.9,71.7,71.4,71.2,71,70.7,70.5,70.3,70.1,69.9,69.6,69.4,69.2,69,68.8,68.6,68.4,68.2,67.9,67.7,67.5,67.3,67.1,66.9,66.7,66.5,66.3,66.1,65.9,65.7,65.5,65.3,65.1,64.9,64.7,64.5,64.3,64.1,64,63.8,63.6,63.4,63.2,63,62.8,62.6,62.5,62.3,62.1,61.9,61.7,61.5,61.4,61.2,61,60.8,60.7,60.5,60.3,60.1,59.9,59.8,59.6,59.4,59.3,59.1,58.9,58.7,58.6,58.4,58.2,58.1,57.9,57.7,57.6,57.4,57.3,57.1,56.9,56.8,56.6,56.5,56.3,56.1,56,55.8,55.7,55.5,55.4,55.2,55,54.9,54.7,54.6,54.4,54.3,54.1,54,53.8,53.7,53.6,53.4,53.3,53.1,53,52.8,52.7,52.5,52.4,52.3,52.1,52,51.8,51.7,51.6,51.4,51.3,51.1,51,50.9,50.7,50.6,50.5,50.3,50.2,50.1,49.9,49.8,49.7,49.6,49.4,49.3,49.2,49,48.9,48.8,48.7,48.5,48.4,48.3,48.2,48,47.9,47.8,47.7,47.5,47.4,47.3,47.2,47.1,46.9,46.8,46.7,46.6,46.5,46.4,46.2,46.1,46,45.9,45.8,45.7,45.5,45.4,45.3,45.2,45.1,45,44.9,44.8,44.6,44.5,44.4,44.3,44.2,44.1,44,43.9,43.8,43.7,43.5,43.4,43.3,43.2,43.1,43,42.9,42.8,42.7,42.6,42.5,42.4,42.3,42.2,42.1,41.9,41.8,41.7,41.6,41.5,41.4,41.3,41.2,41.1,41,40.9,40.8,40.7,40.6,40.5,40.4,40.3,40.2,40.1,40,39.9,39.8,39.7,39.6,39.5,39.4,39.3,39.2,39.1,39,38.9,38.8,38.7,38.6,38.5,38.4,38.3,38.2,38.1,38,37.9,37.8,37.7,37.6,37.5,37.4,37.3,37.2,37.1,37,36.9,36.8,36.7,36.6,36.5,36.4,36.3,36.2,36.1,36,35.9,35.8,35.8,35.7,35.6,35.5,35.4,35.3,35.2,35.1,35,34.9,34.8,34.7,34.6,34.5,34.4,34.3,34.2,34.1,34,33.9,33.8,33.7,33.6,33.5,33.4,33.3,33.2,33.1,33,33,32.9,32.8,32.7,32.6,32.5,32.4,32.3,32.2,32.1,32,31.9,31.8,31.7,31.6,31.5,31.4,31.3,31.2,31.1,31,30.9,30.9,30.8,30.7,30.6,30.5,30.4,30.3,30.2,30.1,30,29.9,29.8,29.7,29.6,29.5,29.4,29.3,29.3,29.2,29.1,29,28.9,28.8,28.7,28.6,28.5,28.4,28.3,28.2,28.1,28,27.9,27.9,27.8,27.7,27.6,27.5,27.4,27.3,27.2,27.1,27,26.9,26.8,26.8,26.7,26.6,26.5,26.4,26.3,26.2,26.1,26,25.9,25.8,25.8,25.7,25.6,25.5,25.4,25.3,25.2,25.1,25,25,24.9,24.8,24.7,24.6,24.5,24.4,24.3,24.2,24.2,24.1,24,23.9,23.8,23.7,23.6,23.5,23.5,23.4,23.3,23.2,23.1,23,22.9,22.8,22.8,22.7,22.6,22.5,22.4,22.3,22.2,22.2,22.1,22,21.9,21.8,21.7,21.7,21.6,21.5,21.4,21.3,21.2,21.1,21.1,21,20.9,20.8,20.7,20.6,20.6,20.5,20.4,20.3,20.2,20.1,20.1,20,19.9,19.8,19.7,19.6,19.6,19.5,19.4,19.3,19.2,19.2,19.1,19,18.9,18.8,18.7,18.7,18.6,18.5,18.4,18.3,18.2,18.2,18.1,18,17.9,17.8,17.8,17.7,17.6,17.5,17.4,17.3,17.3,17.2,17.1,17,16.9,16.9,16.8,16.7,16.6,16.5,16.4,16.4,16.3,16.2,16.1,16,16,15.9,15.8,15.7,15.6,15.5,15.5,15.4,15.3,15.2,15.1,15.1,15,14.9,14.8,14.7,14.6,14.6,14.5,14.4,14.3,14.2,14.1,14.1,14,13.9,13.8,13.7,13.6,13.5,13.5,13.4,13.3,13.2,13.1,13,13,12.9,12.8,12.7,12.6,12.5,12.4,12.4,12.3,12.2,12.1,12,11.9,11.8,11.7,11.7,11.6,11.5,11.4,11.3,11.2,11.1,11,11,10.9,10.8,10.7,10.6,10.5,10.4,10.3,10.2,10.2,10.1,10,9.9,9.8,9.7,9.6,9.5,9.4,9.3,9.2,9.2,9.1,9,8.9,8.8,8.7,8.6,8.5,8.4,8.3,8.2,8.1,8,7.9,7.8,7.8,7.7,7.6,7.5,7.4,7.3,7.2,7.1,7,6.9,6.8,6.7,6.6,6.5,6.4,6.3,6.2,6.1,6,5.9,5.8,5.7,5.6,5.5,5.4,5.3,5.2,5.1,5,4.9,4.8,4.7,4.6,4.5,4.4,4.3,4.2,4.1,4,3.9,3.8,3.7,3.6,3.5,3.4,3.3,3.2,3.1,3,2.9,2.8,2.7,2.6,2.5,2.4,2.3,2.2,2.1,2,1.9,1.8,1.7,1.6,1.5,1.4,1.3,1.2,1.1,1,0.9,0.7,0.6,0.5,0.4,0.3,0.2,0.1,0,-0.1,-0.2,-0.3,-0.4,-0.5,-0.6,-0.7,-0.8,-0.9,-1.1,-1.2,-1.3,-1.4,-1.5,-1.6,-1.7,-1.8,-1.9,-2,-2.1,-2.2,-2.4,-2.5,-2.6,-2.7,-2.8,-2.9,-3,-3.1,-3.2,-3.3,-3.5,-3.6,-3.7,-3.8,-3.9,-4,-4.1,-4.2,-4.4,-4.5,-4.6,-4.7,-4.8,-4.9,-5,-5.2,-5.3,-5.4,-5.5,-5.6,-5.7,-5.9,-6,-6.1,-6.2,-6.3,-6.4,-6.6,-6.7,-6.8,-6.9,-7.1,-7.2,-7.3,-7.4,-7.5,-7.7,-7.8,-7.9,-8,-8.2,-8.3,-8.4,-8.6,-8.7,-8.8,-8.9,-9.1,-9.2,-9.3,-9.5,-9.6,-9.7,-9.9,-10,-10.1,-10.3,-10.4,-10.6,-10.7,-10.8,-11,-11.1,-11.3,-11.4,-11.6,-11.7,-11.9,-12,-12.2,-12.3,-12.5,-12.6,-12.8,-12.9,-13.1,-13.2,-13.4,-13.6,-13.7,-13.9,-14,-14.2,-14.4,-14.5,-14.7,-14.9,-15,-15.2,-15.4,-15.6,-15.7,-15.9,-16.1,-16.3,-16.5,-16.6,-16.8,-17,-17.2,-17.4,-17.6,-17.8,-18,-18.2,-18.4,-18.6,-18.8,-19,-19.2,-19.4,-19.6,-19.8,-20,-20.2,-20.4,-20.7,-20.9,-21.1,-21.3,-21.5,-21.8,-22,-22.2,-22.4,-22.7,-22.9,-23.1,-23.4,-23.6,-23.9,-24.1,-24.3,-24.6,-24.8,-25.1,-25.3,-25.6,-25.8,-26.1,-26.4,-26.6,-26.9,-27.1,-27.4,-27.7,-27.9,-28.2,-28.5,-28.7,-29,-29.3,-29.5,-29.8,-30.1,-30.4,-30.7,-30.9,-31.2,-31.5,-31.8,-32.1,-32.4,-32.6,-32.9,-33.2,-33.5,-33.8,-34.1,-34.4,-34.7,-34.9,-35.2,-35.5,-35.8,-36.1,-36.4,-36.7,-37,-37.2,-37.5,-37.8,-38.1,-38.4,-38.7,-38.9,-39.2,-39.5,-39.8,-40.1,-40.3,-40.6,-40.9,-41.1,-41.4,-41.6,-41.9,-42.2,-42.4,-42.7,-42.9,-43.1,-43.4};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  OWInit();
  sct_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_ADC_Start(&hadc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //OWConvertAll();
	  //HAL_Delay(CONVERT_T_DELAY);
	  //int16_t temperature = 0;
      //OWReadTemperature(&temperature);

	  //sct_value(temperature / 100, 0, 0);

      uint16_t adc_val = HAL_ADC_GetValue(&hadc);
      sct_value(adc_values[adc_val], 0, 0);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_10B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PB0_Pin|SCT_NOE_Pin|SCT_CLK_Pin|SCT_SDI_Pin
                          |SCT_NLA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DQ_GPIO_Port, DQ_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_Pin S1_Pin */
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0_Pin SCT_NOE_Pin SCT_CLK_Pin SCT_SDI_Pin
                           SCT_NLA_Pin */
  GPIO_InitStruct.Pin = PB0_Pin|SCT_NOE_Pin|SCT_CLK_Pin|SCT_SDI_Pin
                          |SCT_NLA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DQ_Pin */
  GPIO_InitStruct.Pin = DQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DQ_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
