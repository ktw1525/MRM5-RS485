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
#include "stm32g0xx_hal.h"

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
#define MCU_NRST_Pin GPIO_PIN_2
#define MCU_NRST_GPIO_Port GPIOF
#define ADC_V_Pin GPIO_PIN_0
#define ADC_V_GPIO_Port GPIOA
#define ADC_1V6_Pin GPIO_PIN_1
#define ADC_1V6_GPIO_Port GPIOA
#define ADC_Zf_Pin GPIO_PIN_2
#define ADC_Zf_GPIO_Port GPIOA
#define SYNC_INT_Pin GPIO_PIN_3
#define SYNC_INT_GPIO_Port GPIOA
#define DAC_Z_Pin GPIO_PIN_4
#define DAC_Z_GPIO_Port GPIOA
#define IFLAG_Z_Pin GPIO_PIN_5
#define IFLAG_Z_GPIO_Port GPIOA
#define TFLAG_Z_Pin GPIO_PIN_6
#define TFLAG_Z_GPIO_Port GPIOA
#define IFLAG_1V6_Pin GPIO_PIN_7
#define IFLAG_1V6_GPIO_Port GPIOA
#define TFLAG_1V6_Pin GPIO_PIN_0
#define TFLAG_1V6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
