/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define GREEN_DBG_LED_1_Pin GPIO_PIN_0
#define GREEN_DBG_LED_1_GPIO_Port GPIOF
#define GREEN_DBG_LED_2_Pin GPIO_PIN_1
#define GREEN_DBG_LED_2_GPIO_Port GPIOF
#define MOT2_PWM_1_Pin GPIO_PIN_0
#define MOT2_PWM_1_GPIO_Port GPIOA
#define MOT2_PWM_2_Pin GPIO_PIN_1
#define MOT2_PWM_2_GPIO_Port GPIOA
#define MOT3_PWM_1_Pin GPIO_PIN_2
#define MOT3_PWM_1_GPIO_Port GPIOA
#define TRW_PWM_1_Pin GPIO_PIN_3
#define TRW_PWM_1_GPIO_Port GPIOA
#define RED_DBG_LED_1_Pin GPIO_PIN_4
#define RED_DBG_LED_1_GPIO_Port GPIOA
#define RED_DBG_LED_2_Pin GPIO_PIN_5
#define RED_DBG_LED_2_GPIO_Port GPIOA
#define HOLD_servo_PWM_Pin GPIO_PIN_6
#define HOLD_servo_PWM_GPIO_Port GPIOA
#define AIM_servo_PWM_Pin GPIO_PIN_7
#define AIM_servo_PWM_GPIO_Port GPIOA
#define MOT_SLEEP_Pin GPIO_PIN_0
#define MOT_SLEEP_GPIO_Port GPIOB
#define MOT1_PWM_1_Pin GPIO_PIN_8
#define MOT1_PWM_1_GPIO_Port GPIOA
#define MOT1_PWM_2_Pin GPIO_PIN_9
#define MOT1_PWM_2_GPIO_Port GPIOA
#define MOT3_PWM_2_Pin GPIO_PIN_10
#define MOT3_PWM_2_GPIO_Port GPIOA
#define MOT3_ENC_1_Pin GPIO_PIN_15
#define MOT3_ENC_1_GPIO_Port GPIOA
#define MOT_OFF_Pin GPIO_PIN_3
#define MOT_OFF_GPIO_Port GPIOB
#define MOT1_ENC_1_Pin GPIO_PIN_4
#define MOT1_ENC_1_GPIO_Port GPIOB
#define MOT1_ENC_2_Pin GPIO_PIN_5
#define MOT1_ENC_2_GPIO_Port GPIOB
#define MOT2_ENC_1_Pin GPIO_PIN_6
#define MOT2_ENC_1_GPIO_Port GPIOB
#define MOT2_ENC_2_Pin GPIO_PIN_7
#define MOT2_ENC_2_GPIO_Port GPIOB
#define MOT3_ENC_2_Pin GPIO_PIN_8
#define MOT3_ENC_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
