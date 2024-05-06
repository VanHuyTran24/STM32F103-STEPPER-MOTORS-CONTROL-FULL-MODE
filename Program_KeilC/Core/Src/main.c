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
uint8_t full_step_1_1[4] = {0x08,0x02,0x04,0x01};
uint8_t full_step_1_2[4] = {0x08,0x01,0x04,0x02};
uint8_t full_step_2_1[4] = {0x0A,0x06,0x05,0x09};
uint8_t full_step_2_2[4] = {0x0A,0x09,0x05,0x06};
uint8_t half_step_1[8]= {0x08,0x0A,0x02,0x06,0x04,0x05,0x01,0x09};
uint8_t half_step_2[8]= {0x08,0x09,0x01,0x05,0x04,0x06,0x02,0x0A};
uint8_t micro_step_1[8] = {20,38,56,70,83,92,98,100};
uint8_t micro_step_2[8] = {98,92,83,70,56,38,20,0};
uint8_t GPIO[4] = {GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_3,GPIO_PIN_5};
float cnt = 0;
float pos = 5;
float rpm = 200;
float t_delay;
int mode = 4;
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void duty_cycles_max(void)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,100);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,100);
}

void delay1_s(float time)
{
	__HAL_TIM_SetCounter(&htim2,0);
	while (__HAL_TIM_GetCounter(&htim2)<time*100000);
	cnt += 1;
}

void delay2_s(float time)
{
	__HAL_TIM_SetCounter(&htim2,0);
	while (__HAL_TIM_GetCounter(&htim2)<(time/2)*100000);
	cnt += 0.5;
}

void delay3_s(float time)
{
	__HAL_TIM_SetCounter(&htim2,0);
	while (__HAL_TIM_GetCounter(&htim2)<(time/16)*100000);
}

void full_step_1_cw(float rpm)
{
	duty_cycles_max();
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{ 
			HAL_GPIO_WritePin(GPIOB,GPIO[j],(full_step_1_1[i]<<j)&0x08);
		}
		delay1_s(t_delay);
	}
}

void full_step_1_ccw(float rpm)
{
	duty_cycles_max();
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0;j < 4; j++)
		{ 
			HAL_GPIO_WritePin(GPIOB,GPIO[j],(full_step_1_2[i]<<j)&0x08);
		}
		delay1_s(t_delay);
	}
}

void full_step_2_cw(float rpm)
{
	duty_cycles_max();
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{ 
			HAL_GPIO_WritePin(GPIOB,GPIO[j],(full_step_2_1[i]<<j)&0x08);
		}
		delay1_s(t_delay);
	}
}

void full_step_2_ccw(float rpm)
{
	duty_cycles_max();
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{ 
			HAL_GPIO_WritePin(GPIOB,GPIO[j],(full_step_2_2[i]<<j)&0x08);
		}
		delay1_s(t_delay);
	}
}

void half_step_cw(float rpm)
{
	duty_cycles_max();
	for(int i = 0; i < 8; i++)
	{
		for(int j = 0; j < 4; j++)
		{ 
			HAL_GPIO_WritePin(GPIOB,GPIO[j],(half_step_1[i]<<j)&0x08);
		}
		delay2_s(t_delay);
	}
}

void half_step_ccw(float rpm)
{
	duty_cycles_max();
	for(int i = 0; i < 8; i++)
	{
		for(int j = 0; j < 4; j++)
		{ 
			HAL_GPIO_WritePin(GPIOB,GPIO[j],(half_step_2[i]<<j)&0x08);
		}
		delay2_s(t_delay);
	}
}

void micro_step_cw(float rpm)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,100);	
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO[j],(full_step_1_1[i]<<j)&0x08);
		}
		if (i%2==0)
		{
			for(int i = 0; i < 8; i++)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,micro_step_1[i]);
				delay3_s(t_delay);
			}
			for(int i = 0; i < 8 ; i++)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,micro_step_2[i]);
				delay3_s(t_delay);
			}
		}
		else
		{
				for(int i = 0; i < 8; i++)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,micro_step_1[i]);
				delay3_s(t_delay);
			}
			for(int i = 0; i < 8 ; i++)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,micro_step_2[i]);
				delay3_s(t_delay);
			}
		}
		cnt += 1;
	}
}

void micro_step_ccw(float rpm)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,100);		
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO[j],(full_step_1_2[i]<<j)&0x08);
		}
		if (i%2==0)
		{
			for(int i = 0; i < 8; i++)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,micro_step_1[i]);
				delay3_s(t_delay);
			}
			for(int i = 0; i < 8 ; i++)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,micro_step_2[i]);
				delay3_s(t_delay);
			}
		}
		else
		{
				for(int i = 0; i < 8; i++)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,micro_step_1[i]);
				delay3_s(t_delay);
			}
			for(int i = 0; i < 8 ; i++)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,micro_step_2[i]);
				delay3_s(t_delay);
			}
		}
		cnt += 1;
	}
}

void position(float pos)
{
	if (cnt == 200*pos)
	{
		cnt = 0;
		mode = 0;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim1);
	
	HAL_TIM_Base_Start(&htim2);
	
	t_delay = 60/(rpm*(360/1.8));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */ 
		switch (mode)
		{
		case 0:
			for(int i = 0; i < 4; i++) 
			{
				HAL_GPIO_WritePin(GPIOB,GPIO[i],0);
			}
			break;
		case 1:
			full_step_1_cw(rpm);
			position(pos);
			break;
		case 2:
			full_step_2_cw(rpm);
			position(pos);
			break;
		case 3:				
			half_step_cw(rpm);
			position(pos);
			break;
		case 4:
			micro_step_cw(rpm);
			position(pos);
			break;
		case 5:
			full_step_1_ccw(rpm);
			position(pos);
			break;
		case 6:
			full_step_2_ccw(rpm);
			position(pos);
			break;
		case 7:				
			half_step_ccw(rpm);
			position(pos);
			break;
		case 8:
			micro_step_ccw(rpm);
			position(pos);
			break;
		}

		/* USER CODE END 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
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
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP ;// GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4
                           PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
