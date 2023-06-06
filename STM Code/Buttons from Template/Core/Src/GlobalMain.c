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

#include <LCD1602.h>	//AD
#include <stdio.h>		//AD

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*					IMPORTANT
 * The arrays create a set of options (3 memory slots, 4 active digipots to control) for the user to control
 * Pointers are used to move through the arrays
 * The position of the pointers directs where the ADC's current value is stored and
 * what memory slot (combination of 3 digipot values + current ADC for selected digipot) is active
 */

char Pot_Choice[5] = {'B', 'L', 'H', 'T', 'N'}; // selects the digital pot to be augmented
volatile char *Pot_pointer = Pot_Choice;
char Memory_Choice[3] = {'A', 'B', 'C'};		// selects the active memory slot
volatile char *Memory_pointer = Memory_Choice;

char Buffer[16];	//the string for pot settings sent to lcd

/*
 * Lines 55-64 declare & initialize memory slots and placeholders for the pot values
 */
uint16_t Current_Value = 30;
uint16_t A_Settings[5] = {30, 30, 30, 30, 30};
uint16_t B_Settings[5] = {30, 30, 30, 30, 30};
uint16_t C_Settings[5] = {30, 30, 30, 30, 30};

volatile int Ba;
volatile int Lm;
volatile int Hm;
volatile int Tr;
volatile int N;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */

  /*
   * This code starts the timer that controls the lcd,
   * initializes the lcd and writes to it before clearing so that the main code controls the lcd
   */
  HAL_TIM_Base_Start(&htim1);

  lcd_init();
  lcd_put_cur(0,0);
  lcd_send_string("Loading...");
  HAL_Delay(1000);
  lcd_clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_ADC_Start(&hadc1);	//starts ADC
	  HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);	//ADC waits for and captures single measurement

	  Current_Value = HAL_ADC_GetValue(&hadc1);	//reads ADC and assigns it to a variable

/*
 * Lines 161-242 directs the ADC reading to a pot position within a memory slot
 * and shows what memory is active
 */

	  if(*Memory_pointer == 'A')
	  {
		  lcd_put_cur(0, 0);
		  lcd_send_string("A:Ba Lm Hm Tr N");	//shows that memeory A is active
		  HAL_Delay(100);

		  if(*Pot_pointer == 'B')
		  {
			  A_Settings[0] = Current_Value;
		  }
		  if(*Pot_pointer == 'L')
		  {
			  A_Settings[1] = Current_Value;
		  }
		  if(*Pot_pointer == 'H')
		  {
			  A_Settings[2] = Current_Value;
		  }
		  if(*Pot_pointer == 'T')
		  {
			  A_Settings[3] = Current_Value;
		  }
		  if(*Pot_pointer == 'N')
		  {
			  A_Settings[4] = Current_Value;
		  }
	  }
	  if(*Memory_pointer == 'B')
	  {
		  lcd_put_cur(0, 0);
		  lcd_send_string("B:Ba Lm Hm Tr N");	//shows that memeory B is active
		  HAL_Delay(100);

	  	  if(*Pot_pointer == 'B')
	  	  {
	  		  B_Settings[0] = Current_Value;
	  	  }
	  	  if(*Pot_pointer == 'L')
	  	  {
	  		  B_Settings[1] = Current_Value;
	  	  }
	  	  if(*Pot_pointer == 'H')
	  	  {
	  		  B_Settings[2] = Current_Value;
	  	  }
	  	  if(*Pot_pointer == 'T')
	  	  {
	  		  B_Settings[3] = Current_Value;
	  	  }
	  	  if(*Pot_pointer == 'N')
	  	  {
	  		  B_Settings[4] = Current_Value;
	  	  }
	  }
	  if(*Memory_pointer == 'C')
	  {
		  lcd_put_cur(0, 0);
		  lcd_send_string("C:Ba Lm Hm Tr N");	//shows that memeory C is active
		  HAL_Delay(100);

	  	  if(*Pot_pointer == 'B')
	  	  {
	  		  C_Settings[0] = Current_Value;
	  	  }
	  	  if(*Pot_pointer == 'L')
	  	  {
	  		  C_Settings[1] = Current_Value;
	  	  }
	  	  if(*Pot_pointer == 'H')
	  	  {
	  		  C_Settings[2] = Current_Value;
	  	  }
	  	  if(*Pot_pointer == 'T')
	  	  {
	  		  C_Settings[3] = Current_Value;
	  	  }
	  	  if(*Pot_pointer == 'N')
	  	  {
	  		  C_Settings[4] = Current_Value;

	  	  }
	  }
	 /*Lines 247-285
*Reads which memory slot is active then then transmits the values for each pot to the lcd
*Accepts pot values as ints then converts them to a string with 'sprintf' before sending that string to lcd
	  */
	  if(*Memory_pointer == 'A')
	  {
		  Ba = A_Settings[0];
		  Lm = A_Settings[1];
		  Hm = A_Settings[2];
		  Tr = A_Settings[3];
		  N = A_Settings[4];

		  sprintf(Buffer, "  %d %d %d %d %d", Ba,Lm,Hm,Tr,N);

		  lcd_put_cur(1, 0);
		  lcd_send_string(Buffer);

	  } else if(*Memory_pointer == 'B')
	  {
		  Ba = B_Settings[0];
		  Lm = B_Settings[1];
		  Hm = B_Settings[2];
		  Tr = B_Settings[3];
		  N = B_Settings[4];

		  sprintf(Buffer, "  %d %d %d %d %d", Ba,Lm,Hm,Tr,N);
		  lcd_put_cur(1, 0);
		  lcd_send_string(Buffer);

	  }else
	  {
		  Ba = C_Settings[0];
		  Lm = C_Settings[1];
		  Hm = C_Settings[2];
		  Tr = C_Settings[3];
		  N = C_Settings[4];

		  sprintf(Buffer, "  %d %d %d %d %d", Ba,Lm,Hm,Tr,N);

		  lcd_put_cur(1, 0);
		  lcd_send_string(Buffer);
	  }
	  HAL_Delay(100);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_6B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
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
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_RS_Pin|LCD_RW_Pin|LCD_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D7_Pin LCD_D6_Pin LCD_D5_Pin LCD_D4_Pin */
  GPIO_InitStruct.Pin = LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_RW_Pin LCD_E_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_RW_Pin|LCD_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_1_Pin Button_2_Pin Button_3_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin|Button_2_Pin|Button_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/*BUTTONS 	ISRs allow the user to cycle through memory slots and pot selections, while a 3rd button
 *			moves pot choice to a null position to avoid unintentional overwrites
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	/*
	the if else statements check whether the pointer is at the end of the array
	to reset its position to the beginning
	*/
	if(GPIO_Pin == Button_1_Pin)
	{
		if(Pot_pointer > Pot_Choice +  40)
		{
			Pot_pointer = Pot_Choice;
		}
		else
		{
			Pot_pointer++;	//increments pointer position to next digital potentiometer
		}
	}
	if(GPIO_Pin == Button_2_Pin)
	{
		if(Memory_pointer > Memory_Choice + 24)
		{
			Memory_pointer = Memory_Choice;
			Pot_pointer = Pot_Choice + 32;	// Moves the pot select to 'N' which is inactive
		}
		else
		{
			Memory_pointer++;	//increments the memory slot
			Pot_pointer = Pot_Choice + 32;	// Moves the pot select to 'N' which is inactive
		}
	}
	if(GPIO_Pin == Button_3_Pin)
	{
		Pot_pointer = Pot_Choice + 32;	// Moves the pot select to 'N' which is inactive
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
