/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t sifir[7] = {1,1,1,1,1,1,0};
uint8_t bir[7] =   {0,1,1,0,0,0,0};
uint8_t iki[7] =   {1,1,0,1,1,0,1};
uint8_t uc[7] =    {1,1,1,1,0,0,1};
uint8_t dort[7] =  {0,1,1,0,0,1,1};
uint8_t bes[7] =   {1,0,1,1,0,1,1};
uint8_t alti[7] =  {1,0,1,1,1,1,1};
uint8_t yedi[7] =  {1,1,1,0,0,0,0};
uint8_t sekiz[7] = {1,1,1,1,1,1,1};
uint8_t dokuz[7] = {1,1,1,1,0,1,1};
uint32_t deger = 0;

char XBEE_TX[11] = "", BOS[4] = "    ", hiz_0[2];
char	komut1[]="   ", komut2[]="   ", *token;
char gelenData[10];
char komut_RAPOR[] =       "RPR\0";
uint8_t TEST_FLAG = 0;
uint16_t komut2_int=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void display(uint8_t sayi[7] , uint8_t basamak);
void sep_digits(uint8_t number, uint8_t basamak[2]);
uint8_t display_2dig(uint8_t sayi);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	  HAL_ADC_PollForConversion(&hadc1, 100);
	  deger = ((HAL_ADC_GetValue(&hadc1)/(41.0f)));
	  if(deger > 98) deger = 99;
	  else deger = 5*floor(deger/5.0f);
	  deger = (deger - 80)*5;
	  if(deger == 95) deger = 99;
	  itoa(deger, hiz_0, 10);
	  if(HAL_GPIO_ReadPin(GPIOA, STOP_Pin) == GPIO_PIN_SET){
		  TEST_FLAG = 0;
		  HAL_UART_Transmit(&huart1, (uint8_t *)"STP       ", strlen("STP       "), 100);
	  }
	  else if(HAL_GPIO_ReadPin(GPIOA, ILERI_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOA, SAG_Pin) == GPIO_PIN_SET){
		  TEST_FLAG = 0;
		  snprintf(XBEE_TX, sizeof(XBEE_TX), "%s%s%s", "ISAG ", hiz_0, BOS);
		  HAL_UART_Transmit(&huart1, (uint8_t *)XBEE_TX, strlen(XBEE_TX), 100);
		  // HAL_UART_Transmit(&huart1, (uint8_t *)"OK        ", strlen("OK        "), 100);

	  }
	  else if(HAL_GPIO_ReadPin(GPIOA, ILERI_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOA, SOL_Pin) == GPIO_PIN_SET){
		  TEST_FLAG = 0;
		  snprintf(XBEE_TX, sizeof(XBEE_TX), "%s%s%s", "ISOL ", hiz_0, BOS);
		  HAL_UART_Transmit(&huart1, (uint8_t *)XBEE_TX, strlen(XBEE_TX), 100);
		  // HAL_UART_Transmit(&huart1, (uint8_t *)"OK        ", strlen("OK        "), 100);

	  }
	  else if(HAL_GPIO_ReadPin(GPIOA, GERI_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOA, SAG_Pin) == GPIO_PIN_SET){
		  TEST_FLAG = 0;
		  snprintf(XBEE_TX, sizeof(XBEE_TX), "%s%s%s", "GSAG ", hiz_0, BOS);
		  HAL_UART_Transmit(&huart1, (uint8_t *)XBEE_TX, strlen(XBEE_TX), 100);
		  //HAL_UART_Transmit(&huart1, (uint8_t *)"OK        ", strlen("OK        "), 100);

	  }
	  else if(HAL_GPIO_ReadPin(GPIOA, GERI_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOA, SOL_Pin) == GPIO_PIN_SET){
		  TEST_FLAG = 0;
		  snprintf(XBEE_TX, sizeof(XBEE_TX), "%s%s%s", "GSOL ", hiz_0, BOS);
		  HAL_UART_Transmit(&huart1, (uint8_t *)XBEE_TX, strlen(XBEE_TX), 100);
		  //HAL_UART_Transmit(&huart1, (uint8_t *)"OK        ", strlen("OK        "), 100);

	  }
	  else if(HAL_GPIO_ReadPin(GPIOA, ILERI_Pin) == GPIO_PIN_SET){
		  TEST_FLAG = 0;
		  snprintf(XBEE_TX, sizeof(XBEE_TX), "%s%s%s", "ILR ", hiz_0, BOS);
		  HAL_UART_Transmit(&huart1, (uint8_t *)XBEE_TX, strlen(XBEE_TX), 100);
		  //HAL_UART_Transmit(&huart1, (uint8_t *)"OK        ", strlen("OK        "), 100);

	  }
	  else if(HAL_GPIO_ReadPin(GPIOA, GERI_Pin) == GPIO_PIN_SET){
		  TEST_FLAG = 0;
		  snprintf(XBEE_TX, sizeof(XBEE_TX), "%s%s%s", "GRI ", hiz_0, BOS);
		  HAL_UART_Transmit(&huart1, (uint8_t *)XBEE_TX, strlen(XBEE_TX), 100);
		  //HAL_UART_Transmit(&huart1, (uint8_t *)"OK        ", strlen("OK        "), 100);

	  }
	  else if(HAL_GPIO_ReadPin(GPIOA, SOL_Pin) == GPIO_PIN_SET){
		  TEST_FLAG = 0;
		  snprintf(XBEE_TX, sizeof(XBEE_TX), "%s%s%s", "RSOL ", hiz_0, BOS);
		  HAL_UART_Transmit(&huart1, (uint8_t *)XBEE_TX, strlen(XBEE_TX), 100);
		  //HAL_UART_Transmit(&huart1, (uint8_t *)"OK        ", strlen("OK        "), 100);

	  }
	  else if(HAL_GPIO_ReadPin(GPIOA, SAG_Pin) == GPIO_PIN_SET){
		  TEST_FLAG = 0;
		  snprintf(XBEE_TX, sizeof(XBEE_TX), "%s%s%s", "RSAG ", hiz_0, BOS);
		  HAL_UART_Transmit(&huart1, (uint8_t *)XBEE_TX, strlen(XBEE_TX), 100);
		  //HAL_UART_Transmit(&huart1, (uint8_t *)"OK        ", strlen("OK        "), 100);

	  }
	  else if(HAL_GPIO_ReadPin(GPIOA, TEST_Pin) == GPIO_PIN_SET){
		  //TEST_FLAG = 1;
		  //HAL_UART_Transmit(&huart1, (uint8_t *)"IMU ALL   ", strlen("IMU ALL   "), 100);
		  //HAL_UART_Transmit(&huart1, (uint8_t *)"ILR 100   ", strlen("ILR 100   "), 100);
		  HAL_UART_Transmit(&huart1, (uint8_t *)"OK        ", strlen("OK        "), 100);
	  }
	  /*
	  else if(TEST_FLAG){

	  }
	  */
	  else {
		  HAL_UART_Transmit(&huart1, (uint8_t *)"DUR       ", strlen("DUR       "), 100);
	  }

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  	  /*  if(!strcmp(gelenData,sag))
  	  HAL_UART_Transmit(&huart5, "2000\r", 10, 100);*/
  token = strtok(gelenData, " ");
  strcpy(komut1, token);
  token = strtok(NULL, " ");
  strcpy(komut2, token);
  komut2_int = atoi(komut2);
  if(!strcmp(komut1,komut_RAPOR)){
	  HAL_GPIO_WritePin(GPIOB, LED1_Pin, 1);
	  HAL_GPIO_WritePin(GPIOB, LED2_Pin, 1);
  }
}
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
  MX_TIM1_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_DMA(&huart1, gelenData, 10);

  HAL_UART_Transmit(&huart1, (uint8_t *)"**", strlen("**"), 100);
  //HAL_UART_Transmit(&huart1, (uint8_t *)"**********", strlen("**********"), 100);
  HAL_Delay(100);
  //HAL_UART_Transmit(&huart1, (uint8_t *)"OK        ", strlen("OK        "), 100);
  HAL_GPIO_WritePin(GPIOB, D3_Pin, 1);
  HAL_GPIO_WritePin(GPIOB, D4_Pin, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)"RPR       ", strlen("RPR       "), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  display_2dig(deger);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A_Pin|B_Pin|LED1_Pin|LED2_Pin
                          |D1_Pin|D2_Pin|D3_Pin|D4_Pin
                          |D_Pin|E_Pin|F_Pin|G_Pin
                          |C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ILERI_Pin GERI_Pin SOL_Pin SAG_Pin
                           STOP_Pin TEST_Pin */
  GPIO_InitStruct.Pin = ILERI_Pin|GERI_Pin|SOL_Pin|SAG_Pin
                          |STOP_Pin|TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A_Pin B_Pin D_Pin E_Pin
                           F_Pin G_Pin C_Pin */
  GPIO_InitStruct.Pin = A_Pin|B_Pin|D_Pin|E_Pin
                          |F_Pin|G_Pin|C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin D2_Pin D3_Pin D4_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D2_Pin|D3_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void display(uint8_t sayi[7] , uint8_t basamak){
	if(basamak == 1){
		HAL_GPIO_WritePin(GPIOB, D1_Pin, 1);
		HAL_GPIO_WritePin(GPIOB, D2_Pin, 0);
	}
	else if(basamak == 0){
		HAL_GPIO_WritePin(GPIOB, D1_Pin, 0);
		HAL_GPIO_WritePin(GPIOB, D2_Pin, 1);
	}
	HAL_GPIO_WritePin(GPIOB, A_Pin, sayi[0]); //A
	HAL_GPIO_WritePin(GPIOB, B_Pin, sayi[1]); //B
	HAL_GPIO_WritePin(GPIOB, C_Pin, sayi[2]); //C
	HAL_GPIO_WritePin(GPIOB, D_Pin, sayi[3]); //D
	HAL_GPIO_WritePin(GPIOB, E_Pin, sayi[4]); //E
	HAL_GPIO_WritePin(GPIOB, F_Pin, sayi[5]); //F
	HAL_GPIO_WritePin(GPIOB, G_Pin, sayi[6]); //G
}
void sep_digits(uint8_t number, uint8_t basamak[2]){
	basamak[0] = number%10;
	basamak[1] = number - basamak[0];
	basamak[1] = basamak[1]/10;
}
uint8_t display_2dig(uint8_t sayi){
	uint8_t basamak[2];
	sep_digits(sayi, basamak);
	HAL_Delay(1);
	switch (basamak[0]) {
		case 0:
			display(sifir, 0);
			break;
		case 1:
			display(bir, 0);
			break;
		case 2:
			display(iki, 0);
			break;
		case 3:
			display(uc, 0);
			break;
		case 4:
			display(dort, 0);
			break;
		case 5:
			display(bes, 0);
			break;
		case 6:
			display(alti, 0);
			break;
		case 7:
			display(yedi, 0);
			break;
		case 8:
			display(sekiz, 0);
			break;
		case 9:
			display(dokuz, 0);
			break;
		default:
			break;
	}
	HAL_Delay(1);
	switch (basamak[1]) {
		case 0:
			display(sifir, 1);
			break;
		case 1:
			display(bir, 1);
			break;
		case 2:
			display(iki, 1);
			break;
		case 3:
			display(uc, 1);
			break;
		case 4:
			display(dort, 1);
			break;
		case 5:
			display(bes, 1);
			break;
		case 6:
			display(alti, 1);
			break;
		case 7:
			display(yedi, 1);
			break;
		case 8:
			display(sekiz, 1);
			break;
		case 9:
			display(dokuz, 1);
			break;
		default:
			break;
	}
	HAL_Delay(1);

	return ((basamak[1]*10) + basamak[0]);


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
