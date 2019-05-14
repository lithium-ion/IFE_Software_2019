/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

uint32_t millisTimer;
uint32_t RTDS_Timer;
uint32_t CAN_Timer;

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

/* USER CODE BEGIN Private defines */

/* Private defines -----------------------------------------------------------*/
#define FAULT_IMD_STATUS_Pin GPIO_PIN_14
#define FAULT_IMD_STATUS_GPIO_Port GPIOC
#define FAULT_BMS_STATUS_Pin GPIO_PIN_15
#define FAULT_BMS_STATUS_GPIO_Port GPIOC
#define FAULT_BSPD_STATUS_Pin GPIO_PIN_0
#define FAULT_BSPD_STATUS_GPIO_Port GPIOD
#define BRAKE_POSITION_Pin GPIO_PIN_0
#define BRAKE_POSITION_GPIO_Port GPIOA
#define STEERING_POSITION_Pin GPIO_PIN_1
#define STEERING_POSITION_GPIO_Port GPIOA
#define BRAKE_PRESSURE_2_Pin GPIO_PIN_2
#define BRAKE_PRESSURE_2_GPIO_Port GPIOA
#define BRAKE_PRESSURE_1_Pin GPIO_PIN_3
#define BRAKE_PRESSURE_1_GPIO_Port GPIOA
#define THROTTLE_A_Pin GPIO_PIN_0
#define THROTTLE_A_GPIO_Port GPIOB
#define THROTTLE_B_Pin GPIO_PIN_1
#define THROTTLE_B_GPIO_Port GPIOB
#define HV_CONNECTED_Pin GPIO_PIN_10
#define HV_CONNECTED_GPIO_Port GPIOB
#define BRAKE_LIGHT_EN_Pin GPIO_PIN_13
#define BRAKE_LIGHT_EN_GPIO_Port GPIOB
#define ENABLE_IN_Pin GPIO_PIN_15
#define ENABLE_IN_GPIO_Port GPIOB
#define RTDS_EN_Pin GPIO_PIN_3
#define RTDS_EN_GPIO_Port GPIOB
#define APPS_EN_Pin GPIO_PIN_4
#define APPS_EN_GPIO_Port GPIOB
#define RTD_EN_Pin GPIO_PIN_5
#define RTD_EN_GPIO_Port GPIOB
#define PRECHARGE_COMPLETE_Pin GPIO_PIN_6
#define PRECHARGE_COMPLETE_GPIO_Port GPIOB
#define BTSF_EN_Pin GPIO_PIN_8
#define BTSF_EN_GPIO_Port GPIOB
#define BRAKE_POS_ADC_CHANNEL 0
#define STEERING_POS_ADC_CHANNEL 1
#define BRAKE_PRESSURE_1_ADC_CHANNEL 3
#define BRAKE_PRESSURE_2_ADC_CHANNEL 2
#define THROTTLE_A_ADC_CHANNEL 8
#define THROTTLE_B_ADC_CHANNEL 9



char checkBTSF();
char checkAPPS();
char APPS_Diff();
void sendFaultMsg();
void sendCar_state();
void readFaults();
void car_state_machine(char STATE);
uint16_t updateADC(int channel); 

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
