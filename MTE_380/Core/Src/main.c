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
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
//Creating a typedef for motor states
typedef enum {
	MOTOR_FORWARD = 0,
	MOTOR_BACKWARD,
	MOTOR_STOP
}MotorStates_t;

//Creating a typedef for Mission States
typedef enum {
	START_RECOGNITION = 0,
	START_TO_TARGET,
	ACQUIRE_TARGET,
	TARGET_ACQUIRED,
	TARGET_TO_START,
	SAFE_ZONE,
	TARGET_RELEASE,
	MISSION_SUCCESSFUL
}MissionStates_t;

//Creating a typedef for Surfaces
typedef enum {
	RED = 0,
	BLUE,
	GREEN,
	WOOD,
	NOT_APPLICABLE
}Surfaces_t;

//State trackers
bool AT_START_LINE = FALSE;
bool AT_TARGET = FALSE;
bool TARGET_PATH_REALIGNED = FALSE;

//Defining Min and Max values for Colour Sensor
int FCL_R_max = 57;
int FCL_R_min = 6;
int FCL_G_max = 55;
int FCL_G_min = 6;
int FCL_B_max = 39;
int FCL_B_min = 4;

int LCL_R_max = 51;
int LCL_R_min = 5;
int LCL_G_max =  53 ;
int LCL_G_min = 3;
int LCL_B_max =  37;
int LCL_B_min = 3;

int RCL_R_max = 41;
int RCL_R_min = 4;
int RCL_G_max = 41;
int RCL_G_min = 3;
int RCL_B_max = 30;
int RCL_B_min = 3;

//Defining all colour sensor parameters
//FRONT COLOUR SENSOR PARAMETERS
// FCL RED SURFACE PARAMETERS
int FCL_RED_R_AVG = 215;
int FCL_RED_R_M_10 = FCL_RED_R_AVG - 10;
int FCL_RED_R_P_10 = FCL_RED_R_AVG +10;

int FCL_RED_G_AVG = 71;
int FCL_RED_G_M_10 = FCL_RED_R_AVG - 10;
int FCL_RED_G_P_10 = FCL_RED_R_AVG +10;

int FCL_RED_B_AVG = 70;
int FCL_RED_B_M_10 = FCL_RED_R_AVG - 10;
int FCL_RED_B_P_10 = FCL_RED_R_AVG +10;

// FCL GREEN SURFACE PARAMETERS

int FCL_GREEN_R_AVG = 100;
int FCL_GREEN_R_M_10 = FCL_GREEN_R_AVG - 10;
int FCL_GREEN_R_P_10 = FCL_GREEN_R_AVG +10;

int FCL_GREEN_G_AVG = 108;
int FCL_GREEN_G_M_10 = FCL_GREEN_R_AVG - 10;
int FCL_GREEN_G_P_10 = FCL_GREEN_R_AVG +10;

int FCL_GREEN_B_AVG = 76;
int FCL_GREEN_B_M_10 = FCL_GREEN_R_AVG - 10;
int FCL_GREEN_B_P_10 = FCL_GREEN_R_AVG +10;

// FCL BLUE SURFACE PARAMETERS

int FCL_BLUE_R_AVG = 185;
int FCL_BLUE_R_M_10 = FCL_BLUE_R_AVG - 10;
int FCL_BLUE_R_P_10 = FCL_BLUE_R_AVG +10;

int FCL_BLUE_G_AVG = 197;
int FCL_BLUE_G_M_10 = FCL_BLUE_R_AVG - 10;
int FCL_BLUE_G_P_10 = FCL_BLUE_R_AVG +10;

int FCL_BLUE_B_AVG = 212;
int FCL_BLUE_B_M_10 = FCL_BLUE_R_AVG - 10;
int FCL_BLUE_B_P_10 = FCL_BLUE_R_AVG +10;

// FCL WOOD SURFACE PARAMETERS

int FCL_WOOD_R_AVG = 230;
int FCL_WOOD_R_M_10 = FCL_WOOD_R_AVG - 10;
int FCL_WOOD_R_P_10 = FCL_WOOD_R_AVG +10;

int FCL_WOOD_G_AVG = 203;
int FCL_WOOD_G_M_10 = FCL_WOOD_R_AVG - 10;
int FCL_WOOD_G_P_10 = FCL_WOOD_R_AVG +10;

int FCL_WOOD_B_AVG = 176;
int FCL_WOOD_B_M_10 = FCL_WOOD_R_AVG - 10;
int FCL_WOOD_B_P_10 = FCL_WOOD_R_AVG +10;


//LEFT COLOUR SENSOR PARAMETERS
// LCL RED SURFACE PARAMETERS

int LCL_RED_R_AVG = 215;
int LCL_RED_R_M_10 = LCL_RED_R_AVG - 10;
int LCL_RED_R_P_10 = LCL_RED_R_AVG +10;

int LCL_RED_G_AVG = 71;
int LCL_RED_G_M_10 = LCL_RED_R_AVG - 10;
int LCL_RED_G_P_10 = LCL_RED_R_AVG +10;

int LCL_RED_B_AVG = 70;
int LCL_RED_B_M_10 = LCL_RED_R_AVG - 10;
int LCL_RED_B_P_10 = LCL_RED_R_AVG +10;

// LCL GREEN SURFACE PARAMETERS

int LCL_GREEN_R_AVG = 100;
int LCL_GREEN_R_M_10 = LCL_GREEN_R_AVG - 10;
int LCL_GREEN_R_P_10 = LCL_GREEN_R_AVG +10;

int LCL_GREEN_G_AVG = 108;
int LCL_GREEN_G_M_10 = LCL_GREEN_R_AVG - 10;
int LCL_GREEN_G_P_10 = LCL_GREEN_R_AVG +10;

int LCL_GREEN_B_AVG = 76;
int LCL_GREEN_B_M_10 = LCL_GREEN_R_AVG - 10;
int LCL_GREEN_B_P_10 = LCL_GREEN_R_AVG +10;

// LCL BLUE SURFACE PARAMETERS

int LCL_BLUE_R_AVG = 185;
int LCL_BLUE_R_M_10 = LCL_BLUE_R_AVG - 10;
int LCL_BLUE_R_P_10 = LCL_BLUE_R_AVG +10;

int LCL_BLUE_G_AVG = 197;
int LCL_BLUE_G_M_10 = LCL_BLUE_R_AVG - 10;
int LCL_BLUE_G_P_10 = LCL_BLUE_R_AVG +10;

int LCL_BLUE_B_AVG = 212;
int LCL_BLUE_B_M_10 = LCL_BLUE_R_AVG - 10;
int LCL_BLUE_B_P_10 = LCL_BLUE_R_AVG +10;

// LCL WOOD SURFACE PARAMETERS

int LCL_WOOD_R_AVG = 230;
int LCL_WOOD_R_M_10 = LCL_WOOD_R_AVG - 10;
int LCL_WOOD_R_P_10 = LCL_WOOD_R_AVG +10;

int LCL_WOOD_G_AVG = 203;
int LCL_WOOD_G_M_10 = LCL_WOOD_R_AVG - 10;
int LCL_WOOD_G_P_10 = LCL_WOOD_R_AVG +10;

int LCL_WOOD_B_AVG = 176;
int LCL_WOOD_B_M_10 = LCL_WOOD_R_AVG - 10;
int LCL_WOOD_B_P_10 = LCL_WOOD_R_AVG +10;


//RCL COLOUR SENSOR PARAMETERS
// RCL RED SURFACE PARAMETERS

int RCL_RED_R_AVG = 215;
int RCL_RED_R_M_10 = RCL_RED_R_AVG - 10;
int RCL_RED_R_P_10 = RCL_RED_R_AVG +10;

int RCL_RED_G_AVG = 71;
int RCL_RED_G_M_10 = RCL_RED_R_AVG - 10;
int RCL_RED_G_P_10 = RCL_RED_R_AVG +10;

int RCL_RED_B_AVG = 70;
int RCL_RED_B_M_10 = RCL_RED_R_AVG - 10;
int RCL_RED_B_P_10 = RCL_RED_R_AVG +10;

// RCL GREEN SURFACE PARAMETERS

int RCL_GREEN_R_AVG = 100;
int RCL_GREEN_R_M_10 = RCL_GREEN_R_AVG - 10;
int RCL_GREEN_R_P_10 = RCL_GREEN_R_AVG +10;

int RCL_GREEN_G_AVG = 108;
int RCL_GREEN_G_M_10 = RCL_GREEN_R_AVG - 10;
int RCL_GREEN_G_P_10 = RCL_GREEN_R_AVG +10;

int RCL_GREEN_B_AVG = 76;
int RCL_GREEN_B_M_10 = RCL_GREEN_R_AVG - 10;
int RCL_GREEN_B_P_10 = RCL_GREEN_R_AVG +10;

// RCL BLUE SURFACE PARAMETERS

int RCL_BLUE_R_AVG = 185;
int RCL_BLUE_R_M_10 = RCL_BLUE_R_AVG - 10;
int RCL_BLUE_R_P_10 = RCL_BLUE_R_AVG +10;

int RCL_BLUE_G_AVG = 197;
int RCL_BLUE_G_M_10 = RCL_BLUE_R_AVG - 10;
int RCL_BLUE_G_P_10 = RCL_BLUE_R_AVG +10;

int RCL_BLUE_B_AVG = 212;
int RCL_BLUE_B_M_10 = RCL_BLUE_R_AVG - 10;
int RCL_BLUE_B_P_10 = RCL_BLUE_R_AVG +10;

// RCL WOOD SURFACE PARAMETERS

int RCL_WOOD_R_AVG = 230;
int RCL_WOOD_R_M_10 = RCL_WOOD_R_AVG - 10;
int RCL_WOOD_R_P_10 = RCL_WOOD_R_AVG +10;

int RCL_WOOD_G_AVG = 203;
int RCL_WOOD_G_M_10 = RCL_WOOD_R_AVG - 10;
int RCL_WOOD_G_P_10 = RCL_WOOD_R_AVG +10;

int RCL_WOOD_B_AVG = 176;
int RCL_WOOD_B_M_10 = RCL_WOOD_R_AVG - 10;
int RCL_WOOD_B_P_10 = RCL_WOOD_R_AVG +10;


//Identifying surface being seen by front colour sensor using getFCLcolour() function
int FCL_R[10] = 0;
int FCL_G[10] = 0;
int FCL_B[10] = 0;

int curr_R_sum = 0;
int curr_G_sum = 0;
int curr_B_sum = 0;

int curr_R_average = 0;
int curr_G_average = 0;
int curr_B_average = 0;

Surfaces_t FCL_Surface = NOT_APPLICABLE;

Surfaces_t getFCLcolour(){

	for (int i = 0; i<10; i++){
		curr_R_sum += FCL_R[i];
		curr_G_sum += FCL_G[i];
		curr_B_sum += FCL_B[i];

		if (i<9){
		FCL_R[i] = FCL_R[i+1];
		FCL_G[i] = FCL_G[i+1];
		FCL_B[i] = FCL_B[i+1];
		}
/*
		else{
		FCL_R[i] = ReadRInput from colour sensor mapped to RGB Scale using max and min;
		S2, S3 = RedFilter
		Frequency_PW = Reading from sensor (SOUT)
		Frequency_RGB = map(FCL_R_max, FCL_R_min, 0, 255)

		FCL_G[i] = ReadGInput from colour sensor mapped to RGB Scale using max and min;
		S2, S3 = GreenFilter
		Frequency_PW = Reading from sensor (SOUT)
		Frequency_RGB = map(FCL_G_max, FCL_G_min, 0, 255)

		FCL_B[i] = ReadBInput from colour sensor mapped to RGB Scale using max and min;
		S2, S3 = BlueFilter
		Frequency_PW = Reading from sensor (SOUT)
		Frequency_RGB = map(FCL_B_max, FCL_B_min, 0, 255)

		}
	}
*/
	curr_R_Average = curr_R_sum/10;
	curr_G_Average = curr_G_sum/10;
	curr_B_Average = curr_B_sum/10;

	if (((FCL_RED_R_M_10 <= curr_R_average) && (curr_R_average <= FCL_RED_R_P_10))
	&& ((FCL_RED_G_M_10 <= curr_G_average) && (curr_G_average <= FCL_RED_G_P_10))
	&& ((FCL_RED_B_M_10 <= curr_B_average) && (curr_B_average <= FCL_RED_B_P_10)))
		{
		FCL_Surface = RED;
		}

	else if (((FCL_GREEN_R_M_10 <= curr_R_average) && (curr_R_average <= FCL_GREEN_R_P_10))
		&& ((FCL_GREEN_G_M_10 <= curr_G_average) && (curr_G_average <= FCL_GREEN_G_P_10))
		&& ((FCL_GREEN_B_M_10 <= curr_B_average) && (curr_B_average <= FCL_GREEN_B_P_10)))
		{
		FCL_Surface = GREEN;
		}

	else if (((FCL_BLUE_R_M_10 <= curr_R_average) && (curr_R_average <= FCL_BLUE_R_P_10))
			&& ((FCL_BLUE_G_M_10 <= curr_G_average) && (curr_G_average <= FCL_BLUE_G_P_10))
			&& ((FCL_BLUE_B_M_10 <= curr_B_average) && (curr_B_average <= FCL_BLUE_B_P_10)))
		{
		FCL_Surface = BLUE;
		}

	else if (((FCL_WOOD_R_M_10 <= curr_R_average) && (curr_R_average <= FCL_WOOD_R_P_10))
				&& ((FCL_WOOD_G_M_10 <= curr_G_average) && (curr_G_average <= FCL_WOOD_G_P_10))
				&& ((FCL_WOOD_B_M_10 <= curr_B_average) && (curr_B_average <= FCL_WOOD_B_P_10)))
		{
		FCL_Surface = WOOD;
		}
	else
		FCL_Surface = NOT_APPLICABLE;

	return FCL_Surface;
}
GPIO_PinState PB5_IN1 = GPIO_PIN_RESET;
GPIO_PinState PB6_IN2 = GPIO_PIN_RESET;
GPIO_PinState PB7_IN3 = GPIO_PIN_RESET;
GPIO_PinState PB8_IN4 = GPIO_PIN_RESET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
void motorMove(MotorStates_t dirLeft, int32_t chLeft, MotorStates_t dirRight, int32_t chRight);

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

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM3_Init();

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	// PC7
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	// PC8
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);	// PC9
	// Timer 3 has a period size of 16 bits, 0 - 65535, ARR set to 65534 (max period)
	// TIM3->CCRX = PWM_Durity_Cycle = CCR/ARR

Surfaces_t MISIION_STATE;

MISSION_STATE = START_RECOGNITION;
SetLED(START_RECOGNITION);

While(At_start_line != 1){
	- Surface_FCL = getFCLcolour();
	- Surface_RCL = getRCLcolour();
	- Surface_LCL = getLCLcolour();
	- While (FCL != RED && LCL != RED && RCL != RED)
		○ Move Forward;
		○ FCL = getFCLcolour();
		○ RCL = getRCLcolour();
		○ LCL = getLCLcolour();
	- Set At Start Line = 1;
}

State = Start_to_Target
SetLED(Start_to_Target);

While(At_Target != 1){
	- FCL = getFCLcolour();
	- RCL = getRCLcolour();
	- LCL = getLCLcolour();
	- If (FCL == RED, RCL == WOOD, LCL == WOOD)
		○ MoveForward
	- Else
		○ Stop
		○ Turn_or_Realign(Forward)
	- If (FCL == BLUE)
		○ At_Target = 1;
}

Stop for 3 seconds

State = Target_to_Start
SetLED(Target_to_Start);

While(At_Start_Line != 1){
	- FCL = getFCLcolour();
	- RCL = getRCLcolour();
	- LCL = getLCLcolour();
	- If (FCL == RED, RCL == WOOD, LCL == WOOD)
		○ MoveReverse
	- Else
		○ Stop
Turn_or_Realign(Reverse)
	while (1)
	{
		motorMove(MOTOR_FORWARD, 65535, MOTOR_FORWARD, 65535);
//		HAL_Delay(1000);
//
//		motorMove(MOTOR_FORWARD, 60000, MOTOR_FORWARD, 30000);
//		HAL_Delay(1000);
//
//		motorMove(MOTOR_FORWARD, 60000, MOTOR_FORWARD, 60000);
//		HAL_Delay(1000);
//
//		motorMove(MOTOR_STOP, 30000, MOTOR_STOP, 30000);
//		HAL_Delay(1000);
//
//		motorMove(MOTOR_BACKWARD, 30000, MOTOR_BACKWARD, 30000);
//		HAL_Delay(1000);
//
//		motorMove(MOTOR_BACKWARD, 60000, MOTOR_BACKWARD, 30000);
//		HAL_Delay(1000);
//
//		motorMove(MOTOR_BACKWARD, 60000, MOTOR_BACKWARD, 60000);
//		HAL_Delay(1000);
//
//		motorMove(MOTOR_STOP, 30000, MOTOR_STOP, 30000);
//		HAL_Delay(1000);

	}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
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
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void motorMove(MotorStates_t dirLeft, int32_t chLeft, MotorStates_t dirRight, int32_t chRight)
{
	// Left Motor
	if(dirLeft == MOTOR_FORWARD){
		PB5_IN1	= GPIO_PIN_SET;
		PB6_IN2 = GPIO_PIN_RESET;
	} else if(dirLeft == MOTOR_BACKWARD){
		PB5_IN1	= GPIO_PIN_RESET;
		PB6_IN2 = GPIO_PIN_SET;
	} else if(dirLeft == MOTOR_STOP){
		PB5_IN1	= GPIO_PIN_RESET;
		PB6_IN2 = GPIO_PIN_RESET;
		chLeft = 0;
	}

	// Right Motor
	if(dirRight == MOTOR_FORWARD){
		PB7_IN3	= GPIO_PIN_SET;
		PB8_IN4 = GPIO_PIN_RESET;
	} else if(dirRight == MOTOR_BACKWARD){
		PB7_IN3	= GPIO_PIN_RESET;
		PB8_IN4 = GPIO_PIN_SET;
	} else if(dirRight == MOTOR_STOP){
		PB7_IN3	= GPIO_PIN_RESET;
		PB8_IN4 = GPIO_PIN_RESET;
		chRight = 0;
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, PB5_IN1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, PB6_IN2);
	TIM3->CCR2 = chLeft;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, PB7_IN3);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, PB8_IN4);
	TIM3->CCR3 = chRight;

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
