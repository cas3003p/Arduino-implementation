/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "ssd1306.h"
#include "stdlib.h"

#include <stdio.h>

#include <stdlib.h>

#include <time.h>

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

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
int counttime=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t adcValues[128] = {0};
int tempVar=1;
int judgestart=0;
int temp=0;

void print_start()
{
 char str[13]="Tap To Start";
 SSD1306_GotoXY(25,25);
 SSD1306_Puts(str, &Font_7x10, 1);
 SSD1306_UpdateScreen();
 HAL_Delay(250);
 SSD1306_Init();

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, adcValues, 128);
  SSD1306_Init();
  SSD1306_Fill(SSD1306_COLOR_BLACK);
  SSD1306_UpdateScreen();

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  if(judgestart==0){
	  print_start();
  }

  while (1)
  {
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim1.Init.Prescaler = 999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1496;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 254;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 124;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint16_t tones[7]={1496,1333,1187,1123,999,890,793};
int bee[64]={5,3,3,4,2,2,1,2,3,4,5,5,5,5,3,3,4,2,2,1,3,5,5,3,2,2,2,2,2,
		3,4,3,3,3,3,3,4,5,5,3,3,4,2,2,1,3,5,5,1};

int airplane=13;
int wall=1;
int oldfy;

uint16_t GPIO_Pin;
float down=0;
float x=128,x2=178,y=10,y2=20,x3=228,y3=40,x4=178,y4=40;
float move=10;
int L=28,O=31,S=35,T=36,I=25,M=29,E=21,colon=10,A=17,R=34;
int a,b;
//int counttime=0;
int judgeend=0;
int timea,timeb,ftime;
float speed=1;
int st[3]={3,2,1};



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)&&tempVar&&temp==0){
		 	  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		 	  judgestart=1;
		 	  tempVar=0;
		 	  temp=1;
	}
	if(judgeend==0&&judgestart==1){
	HAL_UART_Transmit_DMA(&huart2,adcValues,256);
	static int i=0;
	SSD1306_Fill(SSD1306_COLOR_BLACK);

	//設定上下邊線
	for(i = 0; i<128; i++){
		SSD1306_DrawPixel((uint16_t)i , (uint16_t)0 , SSD1306_COLOR_WHITE);
		SSD1306_DrawPixel((uint16_t)i , (uint16_t)60 , SSD1306_COLOR_WHITE);
	}

	//設定障礙物
	SSD1306_GotoXY(x,y);
	SSD1306_Putc(0x30 +wall, &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_GotoXY(x,y+5);
	SSD1306_Putc(0x30 +wall, &Font_7x10, SSD1306_COLOR_WHITE);

	SSD1306_GotoXY(x2,y2);
	SSD1306_Putc(0x30 +2, &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_GotoXY(x2,y2+5);
	SSD1306_Putc(0x30 +2, &Font_7x10, SSD1306_COLOR_WHITE);

	if(counttime==20){
			x3=128;
	}
	if(counttime>=20){
		SSD1306_GotoXY(x3,y3);
		SSD1306_Putc(0x30 +3, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(x3,y3+5);
		SSD1306_Putc(0x30 +3, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(x3,y3+10);
		SSD1306_Putc(0x30 +3, &Font_7x10, SSD1306_COLOR_WHITE);
	}
	if(counttime==40){
		x4=128;
	}
	if(counttime>=40){
		SSD1306_GotoXY(x4,y4);
		SSD1306_Putc(0x30 +4, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(x4,y4+5);
		SSD1306_Putc(0x30 +4, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(x4,y4+10);
		SSD1306_Putc(0x30 +4, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(x4,y4+15);
		SSD1306_Putc(0x30 +4, &Font_7x10, SSD1306_COLOR_WHITE);
	}


	float fy = ((float) adcValues[i] / 4095) *64;
	if(fy > 63){
		fy = 63;
	}
	if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)&&tempVar){
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		down-=3;
		SSD1306_GotoXY(5,down);
		SSD1306_Putc(0x30 + airplane, &Font_7x10, SSD1306_COLOR_WHITE);
		tempVar=0;
	}
	//預設飛機沒控制時會直落下
	if(down>=50){
		down=50;
		SSD1306_GotoXY(5,down);
		SSD1306_Putc(0x30 + airplane, &Font_7x10, SSD1306_COLOR_WHITE);
		if(judgeend==1){
			tempVar=0;
		}
		else{
			tempVar=1;
		}
	}
	else if(down<0){
		down=0;
		SSD1306_GotoXY(5,down);
		SSD1306_Putc(0x30 + airplane, &Font_7x10, SSD1306_COLOR_WHITE);
		if(judgeend==1){
			tempVar=0;
		}
		else{
			tempVar=1;
		}
	}
	else {
		down+=2;
		SSD1306_GotoXY(5,down);
		SSD1306_Putc(0x30 + airplane, &Font_7x10, SSD1306_COLOR_WHITE);
		if(judgeend==1){
			tempVar=0;
		}
		else{
			tempVar=1;
		}
	}


	//判斷飛機撞到障礙物
	if(x==5&&down>=(y-8)&&down<=y+15){
		tempVar=0;
		judgeend=1;

		x-=0;
		y+=0;
		x2-=0;
		y2+=0;
	}
	else if(x2==5&&down>=(y2-8)&&down<=y2+15){
		tempVar=0;
		judgeend=1;

		x-=0;
		y+=0;
		x2-=0;
		y2+=0;
	}
	else if(x3==5&&down>=(y3-8)&&down<=y3+15&&counttime>=20){
			tempVar=0;
			judgeend=1;

			x-=0;
			y+=0;
			x2-=0;
			y2+=0;
			x3-=0;
			y3+=0;
	}
	else if(x4==5&&down>=(y4-8)&&down<=y4+25&&counttime>=40){
			tempVar=0;
			judgeend=1;

			x-=0;
			y+=0;
			x2-=0;
			y2+=0;
			x3-=0;
			y3+=0;
			x4-=0;
			y4+=0;
	}
	//控制障礙物移動(橫向)，到0刷新
	else{
		if(counttime>=20){
			x3-=speed;
		}
		if(counttime>=40){
			x4-=speed;
		}
		x-=speed;
		x2-=speed;
		if (x==0){
			x=128;
			y=(rand()%50)+1;
		}
		if(x2==0){
			x2=128;
			y2=(rand()%50)+1;
			}
		}
		if(x3==0){
			x3=128;
			y3=(rand()%50)+1;
		}
		if(x4==0){
			x4=128;
			y4=(rand()%50)+1;
		}
	}
	else{//失敗時LOST 和時間
		if(judgestart==1){
			SSD1306_GotoXY(64,32);
			SSD1306_Putc(0x30 +L, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(71,32);
			SSD1306_Putc(0x30 +0, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(78,32);
			SSD1306_Putc(0x30 +S, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(85,32);
			SSD1306_Putc(0x30 +T, &Font_7x10, SSD1306_COLOR_WHITE);

			timea=counttime/10;
			timeb=counttime%10;
			SSD1306_GotoXY(64,43);
			SSD1306_Putc(0x30 +T, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(71,43);
			SSD1306_Putc(0x30 +I, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(78,43);
			SSD1306_Putc(0x30 +M, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(85,43);
			SSD1306_Putc(0x30 +E, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(92,43);
			SSD1306_Putc(0x30 +colon, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(99,43);
			SSD1306_Putc(0x30 +timea, &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(106,43);
			SSD1306_Putc(0x30 +timeb, &Font_7x10, SSD1306_COLOR_WHITE);
		}
	}
	SSD1306_UpdateScreen();
}
//計算遊戲時間
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==htim1.Instance&&judgestart==1){
		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		if(judgeend==0){
			counttime++;
			//遊戲進行時播小蜜蜂
			static int c=0;
			if(c>=49){
				TIM2->CCR2=0;
				c=0;			//counter=0;
				}
			else{
				TIM2->PSC=tones[bee[c++]];
				TIM2->CCR2=127;
			}
		}
		else{
			//結束時蜂鳴器發聲
			static int counter=8;
			if(counter<=0){
				TIM2->CCR2=0;
				//counter=0;
			}
			else{
				TIM2->PSC=tones[counter--];
				TIM2->CCR2=127;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
