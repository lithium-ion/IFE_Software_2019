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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CURRENT_VAL_Pin GPIO_PIN_0
#define CURRENT_VAL_GPIO_Port GPIOA
#define CUSTOM_VAL_Pin GPIO_PIN_1
#define CUSTOM_VAL_GPIO_Port GPIOA
#define CUR_LED_Pin GPIO_PIN_3
#define CUR_LED_GPIO_Port GPIOA
#define BMS_LED_ON_Pin GPIO_PIN_4
#define BMS_LED_ON_GPIO_Port GPIOA
#define IMD_LED_ON_Pin GPIO_PIN_5
#define IMD_LED_ON_GPIO_Port GPIOA
#define BSPD_LED_ON_Pin GPIO_PIN_6
#define BSPD_LED_ON_GPIO_Port GPIOA
#define TC_VAL_Pin GPIO_PIN_7
#define TC_VAL_GPIO_Port GPIOA
#define DRS_VAL_Pin GPIO_PIN_1
#define DRS_VAL_GPIO_Port GPIOB
#define DRS_LED_Pin GPIO_PIN_8
#define DRS_LED_GPIO_Port GPIOA
#define TC_LED_Pin GPIO_PIN_9
#define TC_LED_GPIO_Port GPIOA
#define RGB_GREEN_Pin GPIO_PIN_5
#define RGB_GREEN_GPIO_Port GPIOB
#define RGB_RED_Pin GPIO_PIN_6
#define RGB_RED_GPIO_Port GPIOB
#define RGB_BLUE_Pin GPIO_PIN_7
#define RGB_BLUE_GPIO_Port GPIOB
#define CUST_LED_Pin GPIO_PIN_8
#define CUST_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
