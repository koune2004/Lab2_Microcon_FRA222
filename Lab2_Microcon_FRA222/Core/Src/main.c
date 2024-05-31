/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "arm_math.h"
#include "math.h"
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
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
//int set_pos = 0;
float pi = 3.1415926535897932384626433832795;
//float di_pulley = 24.95;

arm_pid_instance_f32 PID = { 0 };
uint32_t QEI_raw = 0;
float QEI_mm = 0;
float set_point = 0;
float en_motor = 0;
float PWMset = 0;
float diffpos = 0;
uint32_t Vfeedback = 0;
uint8_t condi = 0;

uint16_t ADC[2] = { 0 };

uint16_t sen_top = 0;
uint16_t sen_bot = 0;

uint32_t time = 0;



/*-----------------------------------*/
float32_t kp = 0;
float32_t ki = 0;
float32_t kd = 0;
uint32_t Now = 0;
uint32_t Lastime = 0;
float error = 0;
float errorsum = 0;
float Iterm = 0;
float dinput = 0;
float Lasterror = 0;
float speed = 0;
float LastPos = 0;
float Dis_down = 0;



/*--------------------------------------------------*/
float Vmax = 600.0; // mm/s
float Amax = 500.0; // mm/s^2
float Distance = 0;
float t = 0;
float temp_pos_acc = 0;
float temp_pos_const = 0;
float temp_v_acc = 0;
int temp = 0;
float Distance_Velo_Max = 0;
float traj[3];
float pos = 0;
float trajec_target = 0;

float Time_acc = 0;
float Time_acc_tri = 0;
float Time_dec = 0;
float Time_const = 0;
float time_now = 0;
float Timestamp = 0;
float QEI_start = 0;
uint8_t trajec_state = 0;

uint8_t temp_check = 0;
//uint32_t Position[2];
//uint64_t TimeStamp[2];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void PID_Tuning();
void sensor();
void reset_pos();
void HomemadePID();
void Trajectory();
void changeUnit();
void speedread();
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//	PID.Kp = 1;
//	PID.Ki = 0.04291992;
//	PID.Kd = 0;
//	arm_pid_init_f32(&PID, 0);

//	kp = 510;
//	ki = 10;
//	kd = 0.01;

	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
	HAL_TIM_Base_Start(&htim2);

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, ADC, 2);
	reset_pos();

	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	trajec_target = -10;

	HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		if(temp == 10){
//			set_point = 300;
//		}
		QEI_raw = __HAL_TIM_GET_COUNTER(&htim1);
		changeUnit();
		sensor();
//		if(sen_top < 2048 || sen_bot < 2048){
//			if(trajec_target >= 0 && trajec_state == 0){
//				QEI_start = QEI_mm;
//			}
//			Trajectory();
//			HomemadePID();
//			speedread();
//		}
//		else if(sen_top > 2048 || sen_bot > 2048){
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,0);
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);
//			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,10000);
//		}

		if(trajec_target >= 0 && trajec_state == 0){
			QEI_start = QEI_mm;
		}
		Trajectory();
		HomemadePID();
//		PID_Tuning();
		speedread();
		LastPos = QEI_mm;




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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4.294967295E9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 3;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 29999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA15 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
void PID_Tuning(){
		set_point = traj[0];
		Now = __HAL_TIM_GET_COUNTER(&htim2);
		diffpos = set_point - QEI_mm;
		if(diffpos > 32768)
			diffpos -= 65536;
		if(diffpos < -32768)
			diffpos += 65536;

		if(diffpos > 0){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,1);
		}
		else if(diffpos < 0){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);
			diffpos = diffpos*-1;
		}
		else if(diffpos == 0){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);
		}



		Vfeedback = arm_pid_f32(&PID, diffpos);

		PWMset = (Vfeedback/65536.0)*30000;
//		if(diffpos > -60 && diffpos < 60 && time > 0){
//			PWMset = PWMset/((Now-time)/1000000);
//		}
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PWMset);
}
//*/

void sensor() {
	sen_top = ADC[0];
	sen_bot = ADC[1];
}

void reset_pos(){
	set_point = 0;
	time = 0;
	while(sen_bot < 2048){
		sensor();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,7000);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,30000);
	HAL_Delay(250);

	sensor();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,1);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,10000);
	HAL_Delay(1500);

	sensor();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,7000);
	HAL_Delay(500);
	while(sen_bot < 2048){
		sensor();
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,4300);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,10000);
	HAL_Delay(1000);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	if(GPIO_Pin == GPIO_PIN_13)
//	{
//	}
}

void changeUnit(){
	QEI_mm = (QEI_raw/8192.0)*25.326*pi;
}

void HomemadePID(){
//	Now Lastime error errorsum kp ki kd Iterm dinput Lasterror
	set_point = traj[0];
	static uint32_t timestamp = 0;
	if (timestamp < __HAL_TIM_GET_COUNTER(&htim2)) {
		timestamp = __HAL_TIM_GET_COUNTER(&htim2) + 1000;
		Now = __HAL_TIM_GET_COUNTER(&htim2);
		error = set_point - QEI_mm;

		if(error < 1 && error > -1){
			errorsum = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,30000);
		} else if(error < 10 && error > -10){
			errorsum += 10;
		} else if(error < 15 && error > -15){
			errorsum += 15;
		} else if(error < 20 && error > -20){
			errorsum += 20;
		}

		if(error > 32768)
			error -= 65536;
		if(error < -32768)
			error += 65536;

		if(error > 0){
			kp = 520;
			ki = 2;
			kd = 1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,1);
			errorsum = errorsum + (error*(Now-Lastime)/10000);
			Iterm = ki*errorsum;
			if(Iterm < -65535 || Iterm > 65535){
				errorsum = (Iterm/ki) - (error*(Now-Lastime));
			}
			dinput = (error-Lasterror)/(Now-Lastime);
			Vfeedback = (kp*error)+(ki*errorsum)+(kd*dinput);
			if(Vfeedback > 65536){
				Vfeedback = 65536;
			}

			PWMset = (Vfeedback/65536.0)*30000;
		}
		else if(error < 0){
			kp = 20;
			ki = 50;
			kd = 8;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);
			error = error*(-1);

//			if(t < Time_acc_tri){
//				PWMset = 8000;
//			} else{
				errorsum = errorsum + (error*(Now-Lastime)/10000);
				Iterm = ki*errorsum;
				if(Iterm < -65535 || Iterm > 65535){
					errorsum = (Iterm/ki) - (error*(Now-Lastime));
				}
				dinput = (error-Lasterror)/(Now-Lastime);
				Vfeedback = (kp*error)+(ki*errorsum)+(kd*dinput);
				if(Vfeedback > 65536){
					Vfeedback = 65536;
				}

				PWMset = (Vfeedback/65536.0)*30000;
				if(PWMset > 8000){
					PWMset = 8000;
				}
//				PWMset = 4000;
//			}
		}
		else if(error == 0){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,0);
		}

		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PWMset);

		Lastime = Now;
		Lasterror = error;
	}
}


void speedread(){
	Now = __HAL_TIM_GET_COUNTER(&htim2);
	speed = ((QEI_mm - LastPos)*1000000.0)/(Now - Lastime);
	Lastime = Now;
}

void Trajectory(){
	Distance = trajec_target - QEI_start;
	Time_acc = Vmax / Amax;
	Time_dec = Time_acc;


	time_now = __HAL_TIM_GET_COUNTER(&htim2);

	if(trajec_target >= 0 && trajec_state == 0){
		trajec_state = 1;
		Timestamp = time_now;
		QEI_start = QEI_mm;
		pos = QEI_start;
	}
	else if(Distance > 0 && trajec_state == 1){	//Run Up
		Distance_Velo_Max = -(Vmax*Time_acc) + Distance ;
		t = (time_now - Timestamp)/1000000.0;  //s
		if(Distance_Velo_Max > 0){						//Trapezoi
			temp_check = 1;
			Time_const = Distance_Velo_Max/Vmax;
			if(t < Time_acc){
				traj[2] = Amax;
				traj[1] = Amax*t;
				traj[0] = (Amax/2.0)*t*t + pos;
				temp_pos_acc = traj[0];
			}
			else if(t < Time_const+Time_acc){
				traj[2] = 0;
				traj[1] = Vmax;
				traj[0] = (Vmax*(t-Time_acc)) + temp_pos_acc;
				temp_pos_const = traj[0];
			}
			else if(t < Time_const+Time_acc+Time_dec){
				traj[2] = -Amax;
				traj[1] = (traj[2]*(t-Time_const-Time_acc)) + Vmax;
				traj[0] = ((Amax/2)*(t-Time_const-Time_acc)*(t-Time_const-Time_acc))+(traj[1]*(t-Time_const-Time_acc))+temp_pos_const;
			}
			else{
				trajec_state = 0;
				trajec_target = -10;
				traj[2] = 0;
				temp_pos_acc = 0;
				temp_pos_const = 0;
			}
		}
		else if(Distance_Velo_Max <= 0){				//Triangle
			temp_check = 2;
			Time_acc_tri = sqrt(Distance/Amax);
			if(t < Time_acc_tri){
				traj[2] = Amax;
				traj[1] = Amax*t;
				traj[0] = (Amax/2.0)*t*t + pos;
				temp_pos_acc = traj[0];
				temp_v_acc = traj[1];
			}
			else if(t < Time_acc_tri*2){
				traj[2] = -Amax;
				traj[1] = temp_v_acc - (Amax*(t-Time_acc_tri));
				traj[0] = ((Amax/2.0)*(t-Time_acc_tri)*(t-Time_acc_tri))+(traj[1]*(t-Time_acc_tri))+temp_pos_acc;
			}
			else{
				trajec_state = 0;
				trajec_target = -10;
				traj[2] = 0;
				Time_acc_tri = 0;
				temp_pos_acc = 0;
				temp_v_acc = 0;
			}
		}
	}
	else if(Distance < 0 && trajec_state == 1){        		// Run Down
		Distance_Velo_Max = (Vmax*Time_acc) + Distance ;
		t = (time_now - Timestamp)/1000000.0;  //s
		if(Distance_Velo_Max < 0){							//Trapezoi
			temp_check = 3;
			Time_const = Distance_Velo_Max/Vmax;
			if(t < Time_acc){
				traj[2] = -Amax;
				traj[1] = -Amax*t;
				traj[0] = (-Amax/2.0)*t*t + pos;
				temp_pos_acc = traj[0];
			}
			else if(t < Time_const+Time_acc){
				traj[2] = 0;
				traj[1] = -Vmax;
				traj[0] = (-Vmax*(t-Time_acc)) + temp_pos_acc;
				temp_pos_const = traj[0];
			}
			else if(t < Time_const+Time_acc+Time_dec){
				traj[2] = Amax;
				traj[1] = (traj[2]*(t-Time_const-Time_acc)) - Vmax;
				traj[0] = ((Amax/2)*(t-Time_const-Time_acc)*(t-Time_const-Time_acc))+(traj[1]*(t-Time_const-Time_acc))+temp_pos_const;
			}
			else{
				trajec_state = 0;
				trajec_target = -10;
				traj[2] = 0;
				temp_pos_acc = 0;
				temp_pos_const = 0;
			}
		}
		else if(Distance_Velo_Max >= 0){					//Triangle
			temp_check = 4;
			Time_acc_tri = sqrt(-Distance/Amax);
			if(t < Time_acc_tri){
				traj[2] = -Amax;
				traj[1] = -Amax*t;
				traj[0] = (-Amax/2.0)*t*t + pos;
				temp_pos_acc = traj[0];
				temp_v_acc = traj[1];
			}
			else if(t < Time_acc_tri*2){
				traj[2] = Amax;
				traj[1] = temp_v_acc + (Amax*(t-Time_acc_tri));
				traj[0] = ((-Amax/2.0)*(t-Time_acc_tri)*(t-Time_acc_tri))+(traj[1]*(t-Time_acc_tri))+temp_pos_acc;
			}
			else{
				trajec_state = 0;
				trajec_target = -10;
				traj[2] = 0;
				Time_acc_tri = 0;
				temp_pos_acc = 0;
				temp_v_acc = 0;
			}
		}
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
	while (1) {
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
