/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_Pin GPIO_PIN_13
#define DEBUG_GPIO_Port GPIOC
#define FAN_PWM_Pin GPIO_PIN_1
#define FAN_PWM_GPIO_Port GPIOD
#define BMS_CS_Pin GPIO_PIN_4
#define BMS_CS_GPIO_Port GPIOA
#define BMS_SCK_Pin GPIO_PIN_5
#define BMS_SCK_GPIO_Port GPIOA
#define BMS_MISO_Pin GPIO_PIN_6
#define BMS_MISO_GPIO_Port GPIOA
#define BMS_MOSI_Pin GPIO_PIN_7
#define BMS_MOSI_GPIO_Port GPIOA
#define CURR_Pin GPIO_PIN_0
#define CURR_GPIO_Port GPIOB
#define CURR_HR_Pin GPIO_PIN_1
#define CURR_HR_GPIO_Port GPIOB
#define TRIGGER_Pin GPIO_PIN_8
#define TRIGGER_GPIO_Port GPIOA
#define CHARGE_EN_Pin GPIO_PIN_3
#define CHARGE_EN_GPIO_Port GPIOB
#define AIR_Pin GPIO_PIN_5
#define AIR_GPIO_Port GPIOB
#define BMS_FLT_Pin GPIO_PIN_9
#define BMS_FLT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
