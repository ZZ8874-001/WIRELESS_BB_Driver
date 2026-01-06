/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define L_duty 0.05f
#define deadtime 100
#define H_duty 0.95f
#define Hrtim_Period 46080
#define ADC1_VOUT_Pin GPIO_PIN_0
#define ADC1_VOUT_GPIO_Port GPIOA
#define ADC1_VIN_Pin GPIO_PIN_1
#define ADC1_VIN_GPIO_Port GPIOA
#define ADC_CURR_P_Pin GPIO_PIN_4
#define ADC_CURR_P_GPIO_Port GPIOA
#define ADC_CURR_N_Pin GPIO_PIN_5
#define ADC_CURR_N_GPIO_Port GPIOA
#define VCC_Indicator_Pin GPIO_PIN_6
#define VCC_Indicator_GPIO_Port GPIOA
#define BBEN_Indicator_Pin GPIO_PIN_7
#define BBEN_Indicator_GPIO_Port GPIOA
#define BUCK_L_IN_Pin GPIO_PIN_8
#define BUCK_L_IN_GPIO_Port GPIOA
#define BUCK_H_IN_Pin GPIO_PIN_9
#define BUCK_H_IN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
