/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DASH_CAN_ID			0x00F

#define FAULTS				0x0D0
#define PRECHARGE			0x0D1
#define ENABLE_SIG		0x0D2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

CAN_RxHeaderTypeDef   	RxHeader;
uint8_t               	RxData[8];

CAN_TxHeaderTypeDef   	POT_TxHeader;
uint8_t               	POT_data[8];
uint32_t              	TxMailbox;

volatile char					CAN_flag;

uint16_t				pot_threshold[11] = {0, 615, 1025, 1435, 1845, 2255, 2665, 3075, 3485, 3895, 4095};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void POT_read(uint16_t pot_values[4]);
void CAN_interpret(void);
void  POT_interpret(uint16_t pot_values[4]);
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
	if (CAN_flag == 0xFF)
		CAN_interpret();

	//send POT positions CAN message
	
	uint16_t pot_position[4];
  POT_read(pot_position);
	POT_interpret(pot_position);

  HAL_CAN_AddTxMessage(&hcan, &POT_TxHeader, POT_data, &TxMailbox);

	
	//HAL_Delay();

	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef     sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	POT_TxHeader.StdId = DASH_CAN_ID; 						// CAN standard ID
	POT_TxHeader.RTR = CAN_RTR_DATA; 						// CAN frame type
	POT_TxHeader.IDE = CAN_ID_STD; 							// CAN ID type
	POT_TxHeader.DLC = 4; 									// CAN frame length in bytes
	POT_TxHeader.TransmitGlobalTime = DISABLE;				// CAN timestamp in TxData[6] and TxData[7]
  
	sFilterConfig.FilterBank = 0;							// filter number (0-13)
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;		// mask mode or identifier mode		
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;		
	sFilterConfig.FilterIdHigh = 0x0000;					// received ID must match filter ID for each bit specified by filter mask
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;				// specifies which bits of the received ID to compare to the filter ID
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;		// receive FIFO (0 or 1, must match chosen interrupt!)
	sFilterConfig.FilterActivation = ENABLE;
	
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	
	HAL_CAN_Start(&hcan);
	
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); 
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CUR_LED_Pin|BMS_LED_ON_Pin|IMD_LED_ON_Pin|BSPD_LED_ON_Pin 
                          |DRS_LED_Pin|TC_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin|RGB_RED_Pin|RGB_BLUE_Pin|CUST_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CUR_LED_Pin BMS_LED_ON_Pin IMD_LED_ON_Pin BSPD_LED_ON_Pin 
                           DRS_LED_Pin TC_LED_Pin */
  GPIO_InitStruct.Pin = CUR_LED_Pin|BMS_LED_ON_Pin|IMD_LED_ON_Pin|BSPD_LED_ON_Pin 
                          |DRS_LED_Pin|TC_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_GREEN_Pin RGB_RED_Pin RGB_BLUE_Pin CUST_LED_Pin */
  GPIO_InitStruct.Pin = RGB_GREEN_Pin|RGB_RED_Pin|RGB_BLUE_Pin|CUST_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void POT_read(uint16_t pot_values[4]) {
	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);		//change timeout to a HAL define
	pot_values[0] = HAL_ADC_GetValue(&hadc1);
	
	HAL_ADC_PollForConversion(&hadc1, 1000);
	pot_values[1] = HAL_ADC_GetValue(&hadc1);
	
	HAL_ADC_PollForConversion(&hadc1, 1000);
	pot_values[2] = HAL_ADC_GetValue(&hadc1);
	
	HAL_ADC_PollForConversion(&hadc1, 1000);
	pot_values[3] = HAL_ADC_GetValue(&hadc1);
	
	HAL_ADC_Stop(&hadc1);
	
}

void POT_interpret(uint16_t pot_values[4]) {
	
	uint8_t pot_pos[4];
	
	for (uint8_t i = 0; i < 4; i++) {
		
		for (uint8_t j = 0; j < 10; j++) {
			
			//uint16_t pot_threshold[12] = {0, 615, 1025, 1435, 1845, 2255, 2665, 3075, 3485, 3895, 4095};
			if ((pot_values[i] > pot_threshold[j]) && (pot_values[i] <= pot_threshold[j + 1]))
				pot_pos[i] = 10 - (j + 1); // pot_pos[i] = 0 is ~3.3V, pot_pos[i] = 1 is ~3V, etc.
			}

		POT_data[i] = pot_pos[i];
			
	}
	
	if (pot_pos[0] != 0) // if CURRENT_POT is in any position other than first, turn on CUR_LED
		HAL_GPIO_WritePin(GPIOA, CUR_LED_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, CUR_LED_Pin, GPIO_PIN_RESET);
	
	if (pot_pos[1] != 0) // if CUSTOM_POT is in any position other than first, turn on CUST_LED
		HAL_GPIO_WritePin(GPIOB, CUST_LED_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, CUST_LED_Pin, GPIO_PIN_RESET);
	
	if (pot_pos[2] != 0) // if TC_POT is in any position other than first, turn on TC_LED
		HAL_GPIO_WritePin(GPIOA, TC_LED_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, TC_LED_Pin, GPIO_PIN_RESET);
	
	if (pot_pos[3] != 0) // if DRS_POT is in any position other than first, turn on DRS_LED
		HAL_GPIO_WritePin(GPIOA, DRS_LED_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, DRS_LED_Pin, GPIO_PIN_RESET);
	
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
	  CAN_flag = 0xFF;
}

void CAN_interpret(void) {
	
	uint16_t received_ID;
	received_ID = RxHeader.StdId;
	
	if (received_ID == FAULTS) {
		
		uint8_t BMS_fault;
		uint8_t IMD_fault;
		uint8_t BSPD_fault;
		bool any_fault;

		BMS_fault = RxData[0];
		IMD_fault = RxData[1];
		BSPD_fault = RxData[2];

		if (BMS_fault == 0xFF) {
			HAL_GPIO_WritePin(GPIOA, BMS_LED_ON_Pin, GPIO_PIN_SET);
			any_fault = true;
		}
		else if (BMS_fault == 0x00)
			HAL_GPIO_WritePin(GPIOA, BMS_LED_ON_Pin, GPIO_PIN_RESET);	
			
		if (IMD_fault == 0xFF) {
			HAL_GPIO_WritePin(GPIOA, IMD_LED_ON_Pin, GPIO_PIN_SET);
			any_fault = true;
		}
		else if (IMD_fault == 0x00)
			HAL_GPIO_WritePin(GPIOA, IMD_LED_ON_Pin, GPIO_PIN_RESET);
			
		if (BSPD_fault == 0xFF) {
			HAL_GPIO_WritePin(GPIOA, BSPD_LED_ON_Pin, GPIO_PIN_SET);
			any_fault = true;
		}
		else if (BSPD_fault == 0x00)
			HAL_GPIO_WritePin(GPIOA, BSPD_LED_ON_Pin, GPIO_PIN_RESET);
			
		if (any_fault == true) {
			// if there is any fault
			HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, GPIO_PIN_RESET); // set RGB LED red
			HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, GPIO_PIN_RESET);
			any_fault = false;
		}

	}

	if (received_ID == PRECHARGE) {
		
		uint8_t Precharge_state;
		Precharge_state = RxData[0];
		
		if (Precharge_state == 0xFF) {
			// if precharge is complete 
			HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, GPIO_PIN_SET); // set RGB LED green
			HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, GPIO_PIN_RESET);
		}
		
		if (Precharge_state == 0x00) {
			// if precharge is not complete 
			HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, GPIO_PIN_RESET); // set RGB LED blue
			HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, GPIO_PIN_SET);
			
		}
		
	}
	
	CAN_flag = 0x00;
	
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
