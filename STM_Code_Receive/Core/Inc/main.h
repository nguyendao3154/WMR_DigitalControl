/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define LED_PIN_Pin GPIO_PIN_13
#define LED_PIN_GPIO_Port GPIOC
#define ENCODER_R_Pin GPIO_PIN_3
#define ENCODER_R_GPIO_Port GPIOA
#define ENCODER_R_EXTI_IRQn EXTI3_IRQn
#define ENCODER_L_Pin GPIO_PIN_4
#define ENCODER_L_GPIO_Port GPIOA
#define ENCODER_L_EXTI_IRQn EXTI4_IRQn
#define NRF_CSN_Pin GPIO_PIN_0
#define NRF_CSN_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_10
#define NRF_CE_GPIO_Port GPIOB
#define RIGHT_2_Pin GPIO_PIN_3
#define RIGHT_2_GPIO_Port GPIOB
#define RIGHT_PWM_Pin GPIO_PIN_4
#define RIGHT_PWM_GPIO_Port GPIOB
#define LEFT_2_Pin GPIO_PIN_5
#define LEFT_2_GPIO_Port GPIOB
#define LEFT_PWM_Pin GPIO_PIN_6
#define LEFT_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
