/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_1
#define LCD_D7_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_3
#define LCD_D6_GPIO_Port GPIOA
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOA
#define LCD_D4_Pin GPIO_PIN_7
#define LCD_D4_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_7
#define LCD_RS_GPIO_Port GPIOE
#define LCD_RW_Pin GPIO_PIN_9
#define LCD_RW_GPIO_Port GPIOE
#define LCD_E_Pin GPIO_PIN_11
#define LCD_E_GPIO_Port GPIOE
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define Button_1_Pin GPIO_PIN_10
#define Button_1_GPIO_Port GPIOA
#define Button_1_EXTI_IRQn EXTI15_10_IRQn
#define Button_2_Pin GPIO_PIN_13
#define Button_2_GPIO_Port GPIOA
#define Button_2_EXTI_IRQn EXTI15_10_IRQn
#define Button_3_Pin GPIO_PIN_15
#define Button_3_GPIO_Port GPIOA
#define Button_3_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
