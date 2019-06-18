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

#define SOFT_FAULTS				         0x0C1
#define STATES			               0x00E
#define RINEHARTCUR_CAN_ID         0x202
#define RINEHART_FAULTS            0x0AB
#define TC_CONTROL_ID              0x0D3

#define FAULT_CLEAR_TIME          100
//if we are in clear mode, how much time to wait until we fault

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_ChannelConfTypeDef sConfig = {0};
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

CAN_RxHeaderTypeDef     RxHeader;
uint8_t                 RxData[8];

CAN_TxHeaderTypeDef     RIENHART_SOFT_FAULT;
uint8_t                 RIENHART_SOFT_FAULT_Data[8] = {0x14,0x0,0x01,0x0,0x0,0x0,0x0,0x0};

CAN_TxHeaderTypeDef     RIENHART_CURRENT_Tx;
uint8_t                 RIENHART_CURRENT_Data[8] = {0,0,0,0,0,0,0,0};

CAN_TxHeaderTypeDef     TC_CONTROL;
uint8_t                 TC_CONTROL_Data[1] = {0};


uint32_t                TxMailbox;

volatile char					CAN_flag;

uint16_t				pot_threshold[11] = {0, 615, 1025, 1435, 1845, 2255, 2665, 3075, 3485, 3895, 4095};
uint16_t        current_limits[11] = {200,200,175,150,125,100,75,50,30,15,10};
uint8_t         Soft_Fault_Clear = 0x00;

uint8_t         KNOB_FAULT_CLEAR_POS = 0;
uint8_t         LED_SIGN = 0;
uint8_t         BOOT_UP = 0;

uint8_t         FULL_SEND_MODE = 0x00;
uint8_t         CLEARABLE_FAULT = 0x00;





extern int32_t soft_fault_check;
extern int32_t fault_clear_mode;
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
  uint16_t pot_position[4];
  soft_fault_check = 1000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	
	POT_read(pot_position);
	POT_interpret(pot_position);
  HAL_CAN_AddTxMessage(&hcan,&RIENHART_CURRENT_Tx,RIENHART_CURRENT_Data,&TxMailbox);

  HAL_Delay(50);
  if(soft_fault_check <= 0)
    {
      MX_CAN_Init();
      soft_fault_check = 1000;
    }
  if((fault_clear_mode <= 0) && (FULL_SEND_MODE) && (CLEARABLE_FAULT))
  {
    HAL_CAN_AddTxMessage(&hcan,&RIENHART_SOFT_FAULT,RIENHART_SOFT_FAULT_Data,&TxMailbox);
  }

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

  //ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  hcan.Init.Prescaler = 2; //500kbit/s
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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

  RIENHART_CURRENT_Tx.StdId = RINEHARTCUR_CAN_ID; 						// CAN standard ID
	RIENHART_CURRENT_Tx.RTR = CAN_RTR_DATA; 						// CAN frame type
	RIENHART_CURRENT_Tx.IDE = CAN_ID_STD; 							// CAN ID type
	RIENHART_CURRENT_Tx.DLC = 8; 									// CAN frame length in bytes
	RIENHART_CURRENT_Tx.TransmitGlobalTime = DISABLE;

  TC_CONTROL.StdId = TC_CONTROL_ID;             // CAN standard ID
  TC_CONTROL.RTR = CAN_RTR_DATA;             // CAN frame type
  TC_CONTROL.IDE = CAN_ID_STD;               // CAN ID type
  TC_CONTROL.DLC = 1;                  // CAN frame length in bytes
  TC_CONTROL.TransmitGlobalTime = DISABLE;

  RIENHART_SOFT_FAULT.StdId = SOFT_FAULTS;            // CAN standard ID
  RIENHART_SOFT_FAULT.RTR = CAN_RTR_DATA;             // CAN frame type
  RIENHART_SOFT_FAULT.IDE = CAN_ID_STD;               // CAN ID type
  RIENHART_SOFT_FAULT.DLC = 8;                  // CAN frame length in bytes
  RIENHART_SOFT_FAULT.TransmitGlobalTime = DISABLE; 				// CAN timestamp in TxData[6] and TxData[7]

	sFilterConfig.FilterBank = 0;							// filter number (0-13)
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;		// mask mode or identifier mode
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	if(soft_fault_check <= 0)
  {
    sFilterConfig.FilterIdHigh = (0x0AB << 5);
  }
  else
  {
    sFilterConfig.FilterIdHigh = (0x000E << 5);					// received ID must match filter ID for each bit specified by filter mask
	}
  sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0xFFE0;				// specifies which bits of the received ID to compare to the filter ID
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
	//0 1 7 9

	sConfig.Channel = ADC_CHANNEL_0; //
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);		//CURRENT_VAL, PA0
	pot_values[0] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	sConfig.Channel = ADC_CHANNEL_1;   // CUSTOM VAL PA1
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	pot_values[1] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	sConfig.Channel = ADC_CHANNEL_7;         //TC_VAL PA7
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);


	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	pot_values[2] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	sConfig.Channel = ADC_CHANNEL_9;         //DRS_VAL PA9
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);


	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	pot_values[3] = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

}

void POT_interpret(uint16_t pot_values[4]) {

	uint8_t pot_pos[4];
	uint8_t i, j;

	for (i = 0; i < 4; i++) {

		for (j = 0; j < 10; j++) {

			//uint16_t pot_threshold[12] = {0, 615, 1025, 1435, 1845, 2255, 2665, 3075, 3485, 3895, 4095};
			if ((pot_values[i] > pot_threshold[j]) && (pot_values[i] <= pot_threshold[j + 1]))
      {
				pot_pos[i] = 10 - (j + 1); // pot_pos[i] = 0 is ~3.3V, pot_pos[i] = 1 is ~3V, etc.
        if(i == 2)
        {
            RIENHART_CURRENT_Data[0] = current_limits[j];
            RIENHART_CURRENT_Data[1] = current_limits[j] >> 8;
        }
			}
    }
  }

	if (pot_pos[0] != 0) // if CURRENT_POT is in any position other than first, turn on CUR_LED
	{
  	HAL_GPIO_WritePin(GPIOB, CUST_LED_Pin, GPIO_PIN_SET);
  }
  else
  {
  	HAL_GPIO_WritePin(GPIOB, CUST_LED_Pin, GPIO_PIN_RESET);
  }
	if (pot_pos[1] != 0) // if CUSTOM_POT is in any position other than first, turn on CUST_LED
	{	
    HAL_GPIO_WritePin(GPIOA, CUR_LED_Pin, GPIO_PIN_SET);
  }
	else
  {
		HAL_GPIO_WritePin(GPIOA, CUR_LED_Pin, GPIO_PIN_RESET);
  }

	if (pot_pos[2] != 0) // if TC_POT is in any position other than first, turn on TC_LED
		HAL_GPIO_WritePin(GPIOA, TC_LED_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, TC_LED_Pin, GPIO_PIN_RESET);

	if (pot_pos[3] != 0) // if DRS_POT is in any position other than first, turn on DRS_LED
		HAL_GPIO_WritePin(GPIOA, DRS_LED_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, DRS_LED_Pin, GPIO_PIN_RESET);
  if((pot_pos[0] != KNOB_FAULT_CLEAR_POS) && (BOOT_UP == 0xFF))
  {
      HAL_CAN_AddTxMessage(&hcan,&RIENHART_SOFT_FAULT,RIENHART_SOFT_FAULT_Data,&TxMailbox);
  }
  if(pot_pos[0] >= 7)
  {
    FULL_SEND_MODE = 0xFF;
  }
  else if(pot_pos[0] < 7)
  {
    FULL_SEND_MODE = 0x00;
  }
  BOOT_UP = 0xFF;
  KNOB_FAULT_CLEAR_POS = pot_pos[0];

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  //HAL_GPIO_TogglePin(GPIOB, TC_LED_Pin);

  if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
   { 
      if(RxHeader.StdId == STATES || RxHeader.StdId == RINEHART_FAULTS)
        CAN_interpret();
  }
}

void CAN_interpret(void) {

	uint16_t received_ID;
	received_ID = RxHeader.StdId;
  uint8_t BMS_fault;
  uint8_t IMD_fault;
  uint8_t BSPD_fault;
  uint8_t Hold_Array[8];
  Hold_Array[0] = RxData[0];
  Hold_Array[1] = RxData[1];
  Hold_Array[2] = RxData[2];
  Hold_Array[3] = RxData[3];
  Hold_Array[4] = RxData[4];
  Hold_Array[5] = RxData[5];
  Hold_Array[6] = RxData[6];
  Hold_Array[7] = RxData[7];
  uint8_t Precharge_state;
  Precharge_state = RxData[0];

    BMS_fault = RxData[1];
    IMD_fault = RxData[2];
    BSPD_fault = RxData[3];

	if (received_ID == STATES) {


		

		if (Precharge_state == 0x01) {
			// if precharge is complete
			HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, GPIO_PIN_RESET); // set RGB LED green
			HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, GPIO_PIN_RESET);
		}

		if (Precharge_state == 0x02) {
			// if precharge is not complete
			HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, GPIO_PIN_RESET); // set RGB LED blue
			HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, GPIO_PIN_SET);
       //Soft_Fault_Clear = 0x00;
		}

  if (Precharge_state == 0x04) {
      // if precharge is not complete
      HAL_GPIO_TogglePin(GPIOB, RGB_GREEN_Pin); // set RGB LED blue
      HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_TogglePin(GPIOB, RGB_BLUE_Pin);
      //if(Soft_Fault_Clear)
      //{
      //  Soft_Fault_Clear = 0x00;
       // HAL_CAN_AddTxMessage(&hcan,&RIENHART_SOFT_FAULT,RIENHART_SOFT_FAULT_Data,&TxMailbox);
      //}
    }
    if (Precharge_state == 0x08 && (!Soft_Fault_Clear)) {
      // if precharge is not complete
      HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, GPIO_PIN_SET); // set RGB LED blue
      HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, GPIO_PIN_RESET);

    }
    if (Precharge_state == 0x10 && (!Soft_Fault_Clear)) {
      // if precharge is not complete
      HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, GPIO_PIN_SET); // set RGB LED blue
      HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, GPIO_PIN_RESET);

    }
    if (Precharge_state == 0x20 && (!Soft_Fault_Clear)) {
      // if precharge is not complete
       // set RGB LED blue
      HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, GPIO_PIN_SET); // set RGB LED blue
      if(Hold_Array[5] == 0x0A)
      {
      HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, GPIO_PIN_RESET);
      HAL_GPIO_TogglePin(GPIOB, RGB_BLUE_Pin);
      }
      else
      {
        HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, GPIO_PIN_RESET);
        HAL_GPIO_TogglePin(GPIOB, RGB_GREEN_Pin);
      }

    }


    if (BMS_fault == 0xFF) {
      HAL_GPIO_WritePin(GPIOA, BMS_LED_ON_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, GPIO_PIN_RESET); // set RGB LED red
      HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, GPIO_PIN_RESET);
    }
    else if (BMS_fault == 0x00)
      HAL_GPIO_WritePin(GPIOA, BMS_LED_ON_Pin, GPIO_PIN_RESET);

    if (IMD_fault == 0xFF) {
      HAL_GPIO_WritePin(GPIOA, IMD_LED_ON_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, GPIO_PIN_RESET); // set RGB LED red
      HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, GPIO_PIN_RESET);
    }
    else if (IMD_fault == 0x00)
      HAL_GPIO_WritePin(GPIOA, IMD_LED_ON_Pin, GPIO_PIN_RESET);

    if (BSPD_fault == 0xFF) {
      HAL_GPIO_WritePin(GPIOA, BSPD_LED_ON_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, GPIO_PIN_RESET); // set RGB LED red
      HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, GPIO_PIN_RESET);
    }
    else if (BSPD_fault == 0x00)
      HAL_GPIO_WritePin(GPIOA, BSPD_LED_ON_Pin, GPIO_PIN_RESET);
	}

  if(received_ID == RINEHART_FAULTS)
  {

      if((Hold_Array[4] & 0x06) || (Hold_Array[5] & 0x03))
      {
          HAL_GPIO_WritePin(GPIOB,RGB_RED_Pin | RGB_GREEN_Pin | RGB_BLUE_Pin,LED_SIGN);
          if(LED_SIGN == 0)
          {
            LED_SIGN = 1;
          }
          else
          { 
            LED_SIGN = 0;
          }
          Soft_Fault_Clear = 0xFF;
          if(FULL_SEND_MODE)
          {
            CLEARABLE_FAULT = 0xFF;
            fault_clear_mode = FAULT_CLEAR_TIME;
          }
      }
      else if(Hold_Array[0] || Hold_Array[1] || Hold_Array[2] || Hold_Array[3] || Hold_Array[4] || Hold_Array[5] || Hold_Array[7] || Hold_Array[7])
      {
      HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, GPIO_PIN_RESET); // set RGB LED red
      HAL_GPIO_TogglePin(GPIOB, RGB_RED_Pin);
      HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, GPIO_PIN_RESET); 
      Soft_Fault_Clear = 0xFF;
      }
      else
      {
        Soft_Fault_Clear = 0x00;
      }
      soft_fault_check = 1000;
      MX_CAN_Init();
      //HAL_GPIO_TogglePin(GPIOB, RGB_RED_Pin);
  }

  //HAL_CAN_AddTxMessage(&hcan,&TC_CONTROL,TC_CONTROL_Data,&TxMailbox);

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
