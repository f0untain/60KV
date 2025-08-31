/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LcdBackLight_Pin GPIO_PIN_13
#define LcdBackLight_GPIO_Port GPIOC
#define KeyStart_Pin GPIO_PIN_14
#define KeyStart_GPIO_Port GPIOC
#define Lcd4_Pin GPIO_PIN_0
#define Lcd4_GPIO_Port GPIOC
#define Lcd5_Pin GPIO_PIN_1
#define Lcd5_GPIO_Port GPIOC
#define Lcd6_Pin GPIO_PIN_2
#define Lcd6_GPIO_Port GPIOC
#define Lcd7_Pin GPIO_PIN_3
#define Lcd7_GPIO_Port GPIOC
#define LcdEn_Pin GPIO_PIN_2
#define LcdEn_GPIO_Port GPIOA
#define Ntc1_Pin GPIO_PIN_3
#define Ntc1_GPIO_Port GPIOA
#define LcdRs_Pin GPIO_PIN_7
#define LcdRs_GPIO_Port GPIOA
#define VoltageRead_Pin GPIO_PIN_4
#define VoltageRead_GPIO_Port GPIOC
#define Ntc2_Pin GPIO_PIN_1
#define Ntc2_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_10
#define Buzzer_GPIO_Port GPIOB
#define ShortCircuitDetect_Pin GPIO_PIN_14
#define ShortCircuitDetect_GPIO_Port GPIOB
#define Relay_Pin GPIO_PIN_8
#define Relay_GPIO_Port GPIOC
#define RotaryA_Pin GPIO_PIN_8
#define RotaryA_GPIO_Port GPIOA
#define RotaryB_Pin GPIO_PIN_9
#define RotaryB_GPIO_Port GPIOA
#define KeyRotaryMiddle_Pin GPIO_PIN_10
#define KeyRotaryMiddle_GPIO_Port GPIOA
#define KeyStandby_Pin GPIO_PIN_11
#define KeyStandby_GPIO_Port GPIOA
#define KeyBack_Pin GPIO_PIN_12
#define KeyBack_GPIO_Port GPIOA
#define Led1_Pin GPIO_PIN_10
#define Led1_GPIO_Port GPIOC
#define Led2_Pin GPIO_PIN_11
#define Led2_GPIO_Port GPIOC
#define Led3_Pin GPIO_PIN_12
#define Led3_GPIO_Port GPIOC
#define Led4_Pin GPIO_PIN_2
#define Led4_GPIO_Port GPIOD
#define PwmGenerate_Pin GPIO_PIN_3
#define PwmGenerate_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
