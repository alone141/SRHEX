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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "MPU_9255.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {ALL, OnSol, OnSag, OrtaSol, OrtaSag, ArkaSol, ArkaSag}motorlar;
typedef enum{ileri, geri, dur}yonler;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_rx;

/* USER CODE BEGIN PV */
MPU_DataStruct MPU_Degerler;
uint8_t durum = 0, komut2_int = 0, IMU_ALL_FLAG = 0 , IMU_ACC_FLAG = 0, IMU_GYRO_FLAG = 0 , IMU_ANGLE_FLAG=0 , OK_FLAG = 0;
;
char gelenData[10], komut1[]="   ", komut2[]="   ", *token;
char komut_ILERI[] = "ILR\0", komut_GERI[] = "GRI\0", komut_DUR[] = "DUR\0", komut_DON[] = "DON\0", komut_SAG[] = "SAG\0",komut_SOL[] = "SOL\0",
komut_DEMO[] = "DEMO\0", komut_IMU[] = "IMU\0", komut_IMU_ALL[] = "ALL\0",  komut_IMU_GYRO[] = "GYR\0",  komut_IMU_ACC[] = "ACC\0",  komut_IMU_ANG[] = "ANG\0",
komut_TIM8_Period[] = "PER\0" , komut_OK[] = "OK\0\0" , komut_STOP[]= "STP\0" , komut_OnSol[] = "FLL\0" , komut_OrtaSag[] = "MRR\0",
komut_OnSag[] = "FRR\0", komut_OrtaSol[] = "MLL\0" , komut_ArkaSol[] = "BLL\0", komut_ArkaSag[] = "BRR\0";
char rpr[] = "rpr\0" , acc[] = "ACC\0";
yonler DIREC_SAG=dur, DIREC_SOL=dur, DIREC=dur;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMA_Init(void);
static void MX_UART7_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
void motokontrol(motorlar motor_poz, yonler yon, uint8_t hiz);
void ftoa(float n, char* res, int afterpoint);
int intToStr(int x, char str[], int d);
void reverse(char* str, int len);
void XBEE_TransmitDouble(UART_HandleTypeDef *_UART, double deger); //  Double bir degiskeni _UART ile XBEE ye gonderir.

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
  durum++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  token = strtok(gelenData, " ");
  strcpy(komut1, token);
  token = strtok(NULL, " ");
  strcpy(komut2, token);
  komut2_int = atoi(komut2);
  //HAL_UART_Transmit(&huart7, (uint8_t *)"\n ", strlen("\n"), 100);

  if(!strcmp(komut2, komut_ILERI)){
	  DIREC = ileri;
  }
  if(!strcmp(komut2,komut_GERI)){
	  DIREC = geri;
  }
  if(!strcmp(komut1,komut_OnSol)){
	  motokontrol(OnSol, DIREC, 50);
  }
  if(!strcmp(komut1,komut_OnSag)){
	  motokontrol(OnSag, DIREC, 50);
  }
  if(!strcmp(komut1,komut_OrtaSol)){
	  motokontrol(OrtaSol, DIREC, 50);
  }
  if(!strcmp(komut1,komut_OrtaSag)){
	  motokontrol(OrtaSag, DIREC, 50);
  }
  if(!strcmp(komut1,komut_ArkaSag)){
	  motokontrol(ArkaSag, DIREC, 50);
  }
  if(!strcmp(komut1,komut_ArkaSol)){
	  motokontrol(ArkaSol, DIREC, 50);
  }

  if(!strcmp(komut1,rpr)){
	  HAL_UART_Transmit(&huart7, (uint8_t *)"RAPOR ALINDI \n", strlen("RAPOR ALINDI \n"), 100);
	  XBEE_TransmitDouble(&huart7, MPU_Degerler.pitch);
	  durum++;
  }

  else if(!strcmp(komut1,komut_STOP)) {
	  OK_FLAG = 0;
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	  motokontrol(ALL, dur, 0);
  }

  else if(!strcmp(komut1,komut_ILERI)) motokontrol(OnSag, ileri, komut2_int);

  else if(!strcmp(komut1, komut_GERI)) motokontrol(OnSag, geri, komut2_int);

  else if(!strcmp(komut1, komut_DON)){
	  if(!strcmp(komut2, komut_SAG)){
		  DIREC_SAG = ileri;
		  DIREC_SOL = geri;
	  }
	  if(!strcmp(komut2, komut_SOL)){
		  DIREC_SAG = geri;
		  DIREC_SOL = ileri;
	  }
	  motokontrol(OnSol, DIREC_SOL, 50);
	  motokontrol(OnSag, DIREC_SAG, 50);
  }

  else if(!strcmp(komut1,komut_IMU)){

	  if(!strcmp(komut2,komut_IMU_ALL)) IMU_ALL_FLAG = 1;

	  if(!strcmp(komut2,komut_IMU_ACC)) IMU_ACC_FLAG = 1;

	  if(!strcmp(komut2,komut_IMU_GYRO)) IMU_GYRO_FLAG = 1;

	  if(!strcmp(komut2,komut_IMU_ANG)) IMU_ANGLE_FLAG = 1;
  }

  else if(!strcmp(komut1,komut_OK)){
	  OK_FLAG = 1;
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
  }

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
  MPU_ALL(&MPU_Degerler);
  if(OK_FLAG){
	if (IMU_ALL_FLAG) {
		HAL_UART_Transmit(&huart7, (uint8_t *)" Ax: ", strlen(" Ax: "), 100);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.Ax);
		HAL_UART_Transmit(&huart7, (uint8_t *)" Ay: ", strlen(" Ay: "), 100);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.Ay);
		HAL_UART_Transmit(&huart7, (uint8_t *)" Az: ", strlen(" Az: "), 100);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.Az);
		HAL_UART_Transmit(&huart7, (uint8_t *)" Gx: ", strlen(" Gx: "), 100);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.Gx);
		HAL_UART_Transmit(&huart7, (uint8_t *)" Gy: ", strlen(" Gy: "), 100);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.Gy);
		HAL_UART_Transmit(&huart7, (uint8_t *)" Gz: ", strlen(" Gz: "), 100);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.Gz);
		HAL_UART_Transmit(&huart7, (uint8_t *)" pitch: ", strlen(" pitch: "), 100);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.pitch);
		HAL_UART_Transmit(&huart7, (uint8_t *)" roll: ", strlen(" roll: "), 100);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.roll);
	}
	if(IMU_ACC_FLAG){
		HAL_UART_Transmit(&huart7, (uint8_t *)" Ax: ", strlen(" Ax: "), 10);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.Ax);
		HAL_UART_Transmit(&huart7, (uint8_t *)" Ay: ", strlen(" Ay: "), 10);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.Ay);
		HAL_UART_Transmit(&huart7, (uint8_t *)" Az: ", strlen(" Az: "), 10);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.Az);
	}
	if(IMU_GYRO_FLAG){
		HAL_UART_Transmit(&huart7, (uint8_t *)" Gx: ", strlen(" Gx: "), 10);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.Gx);
		HAL_UART_Transmit(&huart7, (uint8_t *)" Gy: ", strlen(" Gy: "), 10);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.Gy);
		HAL_UART_Transmit(&huart7, (uint8_t *)" Gz: ", strlen(" Gz: "), 10);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.Gz);
	}
	if(IMU_ANGLE_FLAG){
		HAL_UART_Transmit(&huart7, (uint8_t *)" pitch: ", strlen(" pitch: "), 10);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.pitch);
		HAL_UART_Transmit(&huart7, (uint8_t *)" roll: ", strlen(" roll: "), 10);
		XBEE_TransmitDouble(&huart7, MPU_Degerler.roll);
	}
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
  MX_UART7_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  HAL_UART_Receive_DMA(&huart7, gelenData , 10);
  HAL_UART_Transmit(&huart7, "BAGLANTI KURULDU! Baudrate: 115200\n", strlen("BAGLANTI KURULDU! Baudrate: 115200\n"), 100);
  HAL_UART_Transmit(&huart7, (uint8_t *)"SIRALAMA: \n Ax \n Ay \n Az \n", strlen("SIRALAMA: \n Ax \n Ay \n Az \n"), 100);
  HAL_UART_Transmit(&huart7, (uint8_t *)" Gx \n Gy \n Gz \n", strlen(" Gx \n Gy \n Gz \n"), 100);
  HAL_UART_Transmit(&huart7, (uint8_t *)" pitch \n roll \n", strlen(" pitch \n roll \n"), 100);
  HAL_Delay(100); //I2C ICIN DELAY (ARTIK GEREKSIZ AMA DURSUN YINE)

  HAL_TIM_Base_Start_IT(&htim8);
  MPU9255_Init(&hi2c1);


/*
  htim1.Instance -> CCR1 = 50 ;
  htim1.Instance->CCR2 = 50;
  htim1.Instance -> CCR3 = 50 ;
  htim1.Instance->CCR4 = 50;
  htim3.Instance -> CCR1 = 50 ;
  htim3.Instance->CCR2 = 50;
*/
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	if (durum && OK_FLAG) {

		  motokontrol(ALL, dur, 0);
		  HAL_Delay(500);
		  motokontrol(OnSol, ileri, 100);
		  HAL_Delay(500);
		  motokontrol(ALL, dur, 0);
		  HAL_Delay(500);
		  motokontrol(OnSag, ileri, 100);
		  HAL_Delay(500);
		  motokontrol(ALL, dur, 0);
		  HAL_Delay(500);
		  motokontrol(OrtaSol, ileri, 100);
		  HAL_Delay(500);
		  motokontrol(ALL, dur, 0);
		  HAL_Delay(500);
		  motokontrol(OrtaSag, ileri, 100);
		  HAL_Delay(500);
		  motokontrol(ALL, dur, 0);
		  HAL_Delay(500);
		  motokontrol(ArkaSol, ileri, 100);
		  HAL_Delay(500);
		  motokontrol(ALL, dur, 0);
		  HAL_Delay(500);
		  motokontrol(ArkaSag, ileri, 100);
		  HAL_Delay(500);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

		/*
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_Delay(500);

		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_Delay(500);

		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_Delay(500);

		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_Delay(500);

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_Delay(500);

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_Delay(500);
		*/
	}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1680;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1680;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16799;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF11 PF12 PF13 PF14 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG13 PG14
                           PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE10 PE12
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void motokontrol(motorlar motor_poz, yonler yon, uint8_t hiz){
	if(motor_poz == OnSol){
		htim1.Instance -> CCR1 = hiz;
		if(yon == ileri){
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET); // AIN2
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET); // AIN1
		}
		else if(yon == geri){
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);
		}
		else if(yon == dur){
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);
		}
	}

	else if(motor_poz == OnSag){
		htim1.Instance -> CCR2 = hiz;
		if(yon == ileri){
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		else if(yon == geri){
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
		}
		else if(yon == dur){
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
		}
	}

	else if(motor_poz == OrtaSol){
		htim1.Instance -> CCR3 = hiz;
		if(yon == ileri){
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);
		}
		else if(yon == geri){
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_SET);
		}
		else if(yon == dur){
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);
		}
	}

	else if(motor_poz == OrtaSag){
		htim1.Instance -> CCR4 = hiz;
		if(yon == ileri){
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
		}
		else if(yon == geri){
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
		}
		else if(yon == dur){
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
		}
	}

	else if(motor_poz == ArkaSol){
		htim3.Instance -> CCR1 = hiz;
		if(yon == ileri){
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
		}
		else if(yon == geri){
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
		}
		else if(yon == dur){
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
		}
	}

	else if(motor_poz == ArkaSag){
		htim3.Instance -> CCR2 = hiz;
		if(yon == ileri){
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		else if(yon == geri){
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
		}
		else if(yon == dur){
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
		}
	}
	else if(motor_poz == ALL){

		motokontrol(OnSol, yon, hiz);
		motokontrol(OnSag, yon, hiz);
		motokontrol(OrtaSol, yon, hiz);
		motokontrol(OrtaSag, yon, hiz);
		motokontrol(ArkaSol, yon, hiz);
		motokontrol(ArkaSag, yon, hiz);
		/*
		if(yon == ileri){

			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET); // AIN2
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET); // AIN1
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);

		}
		else if(yon == geri){

			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET); // AIN2
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET); // AIN1
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);

		}
		else{

			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET); // AIN2
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET); // AIN1
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);

		}
	*/

	}
	else {
		  // (FL)
		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);
		  // (FR)
		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);

		  // (ML)
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);
		  // (MR)
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);

		  // (BL)
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
		  // (BR)
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	}
}
void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
void XBEE_TransmitDouble(UART_HandleTypeDef *_UART, double deger){
	char integerBuffer[10]; //BUFFER
	//itoa(deger,integerBuffer,10); // UNUTTUM BUNU AMA DURSUN BURDA

	// 0a ESIT VEYA 1 DEN BUYUKSE DEGISTIRME
	ftoa(deger, integerBuffer, 5); // DOUBLE TO STRING 4 DIGITS
	if(deger >= 1 || deger == 0){
		 ftoa(deger, integerBuffer, 5); // DOUBLE TO STRING 4 DIGITS
		 HAL_UART_Transmit(_UART, (uint8_t *)integerBuffer, strlen(integerBuffer), 10);
		 HAL_UART_Transmit(_UART, (uint8_t *)"\n", strlen("\n"), 100);

	 }
	else if(deger < 1 && deger > 0){
		HAL_UART_Transmit(_UART, (uint8_t *)"0", strlen("0"), 100);
		 HAL_UART_Transmit(_UART, (uint8_t *)integerBuffer, strlen(integerBuffer), 10);
		 HAL_UART_Transmit(_UART, (uint8_t *)"\n", strlen("\n"), 100);
	}
	else if(deger > -1 && deger < 0){
		deger = -deger;
		ftoa(deger, integerBuffer, 5); // DOUBLE TO STRING 4 DIGITS
		 HAL_UART_Transmit(_UART, (uint8_t *)"-", strlen("-"), 100);
		 HAL_UART_Transmit(_UART, (uint8_t *)"0", strlen("0"), 100);
		 HAL_UART_Transmit(_UART, (uint8_t *)integerBuffer, strlen(integerBuffer), 10);
		 HAL_UART_Transmit(_UART, (uint8_t *)"\n", strlen("\n"), 100);

	}
	// 0 DAN KUCUKSE BASINA - KOYUP GONDER
	 else if(deger <= -1){
		 deger = -deger;
		 ftoa(deger, integerBuffer, 5); // DOUBLE TO STRING 4 DIGITS
		 HAL_UART_Transmit(_UART, (uint8_t *)"-", strlen("-"), 100);
		 HAL_UART_Transmit(_UART, (uint8_t *)integerBuffer, strlen(integerBuffer), 10);
		 HAL_UART_Transmit(_UART, (uint8_t *)"\n", strlen("\n"), 100);

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
