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
#include "LTC6811.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CELLVAL_ID      0x007
#define BMSSTAT_ID      0x008
#define BMSVINF_ID      0x009
#define BMSTINF_ID      0x00A
#define PACKSTAT_ID     0x00B
#define CHARGER_ID      0x1806E5F4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
CommandCodeTypedef      CommandCode;
BMSconfigStructTypedef  BMSconfig;

ADC_ChannelConfTypeDef sConfig = {0};

CAN_TxHeaderTypeDef     TxHeader;
CAN_RxHeaderTypeDef     RxHeader;
uint8_t                 TxData[8];
uint8_t                 RxData[8];
uint32_t                TxMailbox;

CAN_TxHeaderTypeDef     ChargerTxHeader;
uint8_t                 ChargerTxData[8];

uint8_t                 CELLVAL_DATA[6];
uint8_t                 BMSSTAT_DATA[6];

uint16_t                minimum;
uint32_t                sumOfCells;
uint8_t                 chargeRate = 2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
bool FAULT_check(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6], uint8_t bmsStatus[6]);
void setDischarge(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6], bool cellDischarge[12][8], bool bmsFault, bool fullDischarge[12][8]);
void checkDischarge(BMSconfigStructTypedef cfg, bool fullDischarge[12][8], uint8_t bmsData[96][6]);
uint16_t balancingThreshold(BMSconfigStructTypedef cfg);
void setChargerTxData(BMSconfigStructTypedef cfg);
void CELLVAL_message(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6]);
void BMSVINF_message(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6]);
void BMSTINF_message(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6]);
void BMSSTAT_message(BMSconfigStructTypedef cfg, uint8_t bmsStatus[6]);
void PACKSTAT_message(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6]);
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
  //MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  SPI_Init();
  initPECTable();
  loadConfig(&BMSconfig);

  uint8_t BMS_DATA[96][6];
  uint8_t BMS_STATUS[6];
  bool discharge[12][8];
  bool full_discharge[12][8];

  bool AIR = 0;
  bool CHARGE_EN = 0;
  bool BMS_FAULT = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //reset config registers
    writeConfigAll(BMSconfig);

    //wait 100ms
    HAL_Delay(100);

    //read all cell voltages, send BMSVINF message
    readAllCellVoltages(BMSconfig, BMS_DATA);
    BMSVINF_message(BMSconfig, BMS_DATA);

    //read all cell temps, send BMSTINF message
    readAllCellTemps(BMSconfig, BMS_DATA);
    BMSTINF_message(BMSconfig, BMS_DATA);

    //check for faults (cell DC, cell OV, cell UV, cell OT, cell TDC, invalid PEC, no charger comm)
    checkAllCellConnections(BMSconfig, BMS_DATA);
    BMS_FAULT = FAULT_check(BMSconfig, BMS_DATA, BMS_STATUS);

    //if AIR and CHARGE_EN are low, pack is charging
    AIR = HAL_GPIO_ReadPin(GPIOB, AIR_Pin);
    CHARGE_EN = HAL_GPIO_ReadPin(GPIOB, CHARGE_EN_Pin);

    if (/*(AIR == 0) && */(CHARGE_EN == 0)) {

      if (chargeRate != 0)
        setDischarge(BMSconfig, BMS_DATA, discharge, BMS_FAULT, full_discharge);
      
      setChargerTxData(BMSconfig);

      if (chargeRate != 0) {
        dischargeCellGroups(BMSconfig, discharge);
        HAL_Delay(BMSconfig.dischargeTime);
      }
      else {
        checkDischarge(BMSconfig, full_discharge, BMS_DATA);
        dischargeCellGroups(BMSconfig, full_discharge);
        HAL_Delay(BMSconfig.dischargeTime);
      }
    }

    //send remaining CAN messages
    CELLVAL_message(BMSconfig, BMS_DATA);
    BMSSTAT_message(BMSconfig, BMS_STATUS);
    PACKSTAT_message(BMSconfig, BMS_DATA);

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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */
  CAN_FilterTypeDef     sFilterConfig;
  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 1; //500kbit/s
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
  TxHeader.StdId = 0x321; 				// CAN standard ID
	TxHeader.ExtId = 0x01; 					// CAN extended ID
	TxHeader.RTR = CAN_RTR_DATA; 			// CAN frame type
	TxHeader.IDE = CAN_ID_STD; 				// CAN ID type
	TxHeader.DLC = 8; 						// CAN frame length in bytes
	TxHeader.TransmitGlobalTime = DISABLE;	// CAN timestamp in TxData[6] and TxData[7]

	ChargerTxHeader.ExtId = CHARGER_ID; 					// CAN extended ID
	ChargerTxHeader.RTR = CAN_RTR_DATA; 			// CAN frame type
	ChargerTxHeader.IDE = CAN_ID_EXT; 				// CAN ID type
	ChargerTxHeader.DLC = 8; 						// CAN frame length in bytes
	ChargerTxHeader.TransmitGlobalTime = DISABLE;	// CAN timestamp in TxData[6] and TxData[7]

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BMS_CS_Pin|TRIGGER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMS_FLT_GPIO_Port, BMS_FLT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DEBUG_Pin */
  GPIO_InitStruct.Pin = DEBUG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_CS_Pin TRIGGER_Pin */
  GPIO_InitStruct.Pin = BMS_CS_Pin|TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CHARGE_EN_Pin AIR_Pin */
  GPIO_InitStruct.Pin = CHARGE_EN_Pin|AIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BMS_FLT_Pin */
  GPIO_InitStruct.Pin = BMS_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMS_FLT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);

  uint16_t ID = RxHeader.StdId;
  uint32_t ExtID = RxHeader.ExtId;

  //TESTING
  if (ID == 0x123) {

    //HAL_GPIO_TogglePin(GPIOC, DEBUG_Pin);
    voltageState = RxData[0];
  }
  //TESTING

  if (ExtID == CHARGER_ID) {
    TIM2->CNT = 0; // reset timer
    HAL_TIM_Base_Start(&htim2);
    RxHeader.ExtId = 0;
    //HAL_GPIO_TogglePin(GPIOC, DEBUG_Pin);
    HAL_GPIO_WritePin(GPIOC, DEBUG_Pin, GPIO_PIN_SET);
  }
}*/

bool FAULT_check(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6], uint8_t bmsStatus[6]) {

  bool BMS_FAULT = false;
  uint16_t cellVoltage;
  bool cellConnection;
  bool dataValid;
  bool OT_fault;
  bool DC_fault;
  uint8_t board;
  uint8_t error_count[cfg.numOfICs];

  bmsStatus[0] = 0;

  for (uint8_t cell = 0; cell < 96; cell++) {

    cellVoltage = 0;
    cellVoltage = (uint16_t) (bmsData[cell][2]);
    cellVoltage = cellVoltage << 8;
		cellVoltage += (uint16_t) (bmsData[cell][3]);

    cellConnection = (bool) (bmsData[cell][1] & 0x01);
    dataValid = (bool) ((bmsData[cell][1] & 0x02) >> 1);
    OT_fault = (bool) ((bmsData[cell][1] & 0x10) >> 4);
    DC_fault = (bool) ((bmsData[cell][1] & 0x08) >> 3);

    //OV fault
    if (cellVoltage > cfg.OV_threshold) {
      BMS_FAULT = true;
      bmsStatus[0] |= 0x01; //fault byte
      bmsStatus[1] = cell + 1; //OV cell number
    }

    //UV fault
    if (cellVoltage < cfg.UV_threshold) {
      BMS_FAULT = true;
      bmsStatus[0] |= 0x02; //fault byte
      bmsStatus[2] = cell + 1; //UV cell number
    }

    //Cell DC fault
    if (cellConnection == 0) {
      BMS_FAULT = true;
      bmsStatus[0] |= 0x80; //fault byte
      bmsStatus[4] = cell + 1; //DC cell number
    }

    //OT fault
    if (OT_fault == 1) {
      BMS_FAULT = true;
      bmsStatus[0] |= 0x04; //fault byte
      bmsStatus[3] = cell + 1; //OT cell number
    }

    //Temp DC fault
    if (DC_fault == 1) {
      BMS_FAULT = true;
      bmsStatus[0] |= 0x10;
      bmsStatus[5] = cell + 1;
    }

    board = cell / cfg.numOfCellsPerIC;

    //Board DC fault
    if (dataValid == 0) {
      error_count[board]++;

      //if data is invalid for a board, every cell will report invalid
      if (error_count[board] > (cfg.numOfCellsPerIC * cfg.invalidPECcount)) {
        BMS_FAULT = true;
        bmsStatus[0] |= 0x20;
      }
    }
    if (dataValid == 1) {
      error_count[board] = 0;
    }

    //Charger comm fault
    /*count = TIM2->CNT;

    if (count > 20000) {
      BMS_FAULT = true;
      HAL_TIM_Base_Stop(&htim2);
    }*/
  }

  if (BMS_FAULT == false)
    HAL_GPIO_WritePin(GPIOB, BMS_FLT_Pin, GPIO_PIN_RESET);
  if (BMS_FAULT == true)
    HAL_GPIO_WritePin(GPIOB, BMS_FLT_Pin, GPIO_PIN_SET);
  
  return BMS_FAULT;
}

void setDischarge(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6], bool cellDischarge[12][8], bool bmsFault, bool fullDischarge[12][8]) {

  uint16_t threshold;
  uint16_t cellVoltage;
  uint8_t board;
  uint8_t cell;
	chargeRate = 2; // initialize the charging current to normal operation

  // if there is any type of fault, stop charging
  // if (bmsFault) {
  //   chargeRate = 0;
  //   //return 0;
  // }

  for (uint8_t i = 0; i < 96; i++) {
    
    cellVoltage = 0;
    cellVoltage = (uint16_t) (bmsData[i][2]);
    cellVoltage = cellVoltage << 8;
		cellVoltage += (uint16_t) (bmsData[i][3]);
    if (cellVoltage == 65535) continue;

    board = i / cfg.numOfCellsPerIC;
    cell = i % cfg.numOfCellsPerIC;
    cellDischarge[board][cell] = 1;

    bmsData[i][1] &= 0x5F; //charging state = 2

    // // if any cell voltage is much greater than the minimum (>200mV), stop charging and discharge that cell to the minimum
    // if (cellVoltage > (minimum + cfg.max_difference)) {
    //   chargeRate = 0;
    //   fullDischarge[board][cell] = 1;
    //   bmsData[i][1] &= 0x7F; //charging state = 3
    // }
    // // if any cell voltage is greater than some absolute threshold (4.18V), stop charging and discharge that cell to the minimum
    // // could discharge to a fixed value (4.15V) instead
    // if (cellVoltage > cfg.stopCharge_threshold) {
    //   chargeRate = 0;
    //   fullDischarge[board][cell] = 1;
    //   bmsData[i][1] &= 0x9F; //charging state = 4
    // }

    // // if still charging AND cells above ~3V
    // if ((chargeRate != 0) && (minimum > 30000)) {

    //   // if any cell is above some absolute threshold, charge slower 
    //   if (cellVoltage > cfg.slowCharge_threshold)
    //     chargeRate = 1;

    //   // determine the relative balancing threshold based on minimum voltage
    //   threshold = balancingThreshold(cfg);

    //   if (cellVoltage > (minimum + threshold)) {
    //     cellDischarge[board][cell] = 1;
    //     bmsData[i][1] &= 0x2F; //charging state = 1
    //   }
    //   else {
    //     cellDischarge[board][cell] = 0;
    //     bmsData[i][1] &= 0x1F; //charging state = 0
    //   }
    // }
	}
}

uint16_t balancingThreshold(BMSconfigStructTypedef cfg) {

  if (minimum > cfg.start_scaling) {
    // min1 = 4.1V   threshold1 = 50mV
    // min2 = 4.16V  threshold2 = 10mV

    // m = (threshold2 - threshold1) / (min2 - min1)
    // linear = m * (minimum - min1) + threshold1

    float m;
    uint16_t linear;

    //  ()
    m = ((float) (cfg.scale_to - cfg.balancing_difference)) / (cfg.stop_scaling - cfg.start_scaling);
    linear = (uint16_t) m * (minimum - cfg.start_scaling) + cfg.balancing_difference;

    return linear;

  }
  if (minimum <= cfg.start_scaling)
    return cfg.balancing_difference; 

  return 0;

}

void checkDischarge(BMSconfigStructTypedef cfg, bool fullDischarge[12][8], uint8_t bmsData[96][6]) {

  uint8_t sum = 0;
  uint16_t cellVoltage;
  uint8_t board;
  uint8_t cell;

  for (uint8_t i = 0; i < 96; i++) {

    cellVoltage = 0;
    cellVoltage = (uint16_t) (bmsData[i][2]);
    cellVoltage = cellVoltage << 8;
		cellVoltage += (uint16_t) (bmsData[i][3]);

    board = i / cfg.numOfCellsPerIC;
    cell = i % cfg.numOfCellsPerIC;

    if (fullDischarge[board][cell] == 1) {
      if (cellVoltage <= minimum) {
        fullDischarge[board][cell] = 0;
        bmsData[i][1] &= 0x5F; //reset charging state to 2
      }
      else
        sum += 1;
    }
  }

  if (sum == 0)
    chargeRate = 2;

}

void setChargerTxData(BMSconfigStructTypedef cfg) {

  TxHeader.ExtId = 0x1806E5F4;
  TxHeader.DLC = 8;

	/* voltage data (hex value of desired voltage (V) times 10)*/
	ChargerTxData[0] = (uint8_t)((cfg.chargerVoltage & 0xFF) >> 8);
	ChargerTxData[1] = (uint8_t)(cfg.chargerVoltage & 0xFF);

	/* set the current data (hex value of desired current (A) times 10) */
	switch (chargeRate) {
		case 1:
			/* lower current */
			ChargerTxData[2] = (uint8_t)((cfg.lowerCurrent & 0xFF) >> 8);
			ChargerTxData[3] = (uint8_t)(cfg.lowerCurrent & 0xFF);
			break;

		case 2:
			/* normal current */
			ChargerTxData[2] = (uint8_t)((cfg.normalCurrent & 0xFF) >> 8);
			ChargerTxData[3] = (uint8_t)(cfg.normalCurrent & 0xFF);
			break;

		default:
			/* no current */
			ChargerTxData[2] = 0x00;
			ChargerTxData[3] = 0x00;
	}

	/* these data bytes are not used */
	ChargerTxData[4] = 0x00;
	ChargerTxData[5] = 0x00;
	ChargerTxData[6] = 0x00;
	ChargerTxData[7] = 0x00;

  HAL_CAN_AddTxMessage(&hcan, &ChargerTxHeader, ChargerTxData, &TxMailbox);
}

void CELLVAL_message(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6]) {
	
  TxHeader.StdId = CELLVAL_ID;
  TxHeader.DLC = 6;

  for (uint8_t cell = 0; cell < 96; cell++) {
    CELLVAL_DATA[0] = bmsData[cell][0];
    CELLVAL_DATA[1] = bmsData[cell][1];
    CELLVAL_DATA[2] = bmsData[cell][2];
    CELLVAL_DATA[3] = bmsData[cell][3];
    CELLVAL_DATA[4] = bmsData[cell][4];
    CELLVAL_DATA[5] = bmsData[cell][5];

    HAL_CAN_AddTxMessage(&hcan, &TxHeader, CELLVAL_DATA, &TxMailbox);
    HAL_Delay(10);

  }
}

void BMSSTAT_message(BMSconfigStructTypedef cfg, uint8_t bmsStatus[6]) {

  TxHeader.StdId = BMSSTAT_ID;
  TxHeader.DLC = 6;

  HAL_CAN_AddTxMessage(&hcan, &TxHeader, bmsStatus, &TxMailbox);
}

void BMSVINF_message(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6]) {

  uint16_t cellVoltage;
  uint16_t minV = 0xFFFF;
  uint8_t minCell;
  uint16_t maxV = 0x0000;
  uint8_t maxCell;
  uint16_t averageV;
  uint32_t sum = 0;

  for (uint8_t cell = 0; cell < 96; cell++) {

    cellVoltage = 0;
    cellVoltage = (uint16_t) (bmsData[cell][2]);
    cellVoltage = cellVoltage << 8;
		cellVoltage += (uint16_t) (bmsData[cell][3]);

    if (cellVoltage < minV) {
      minV = cellVoltage;
      minCell = cell + 1;
    }

    if (cellVoltage > maxV) {
      maxV = cellVoltage;
      maxCell = cell + 1;
    }

    sum += cellVoltage;
  }

  sumOfCells = sum;
  minimum = minV;
  averageV = (uint16_t) (sum / (cfg.numOfICs * cfg.numOfCellsPerIC));

  TxHeader.StdId = BMSVINF_ID;
  TxHeader.DLC = 8;
  uint8_t BMSVINF_DATA[8];

  BMSVINF_DATA[0] = (uint8_t) ((maxV >> 8) & 0xFF);
  BMSVINF_DATA[1] = (uint8_t) (maxV & 0xFF);
  BMSVINF_DATA[2] = maxCell;
  BMSVINF_DATA[3] = (uint8_t) ((minV >> 8) & 0xFF);
  BMSVINF_DATA[4] = (uint8_t) (minV & 0xFF);
  BMSVINF_DATA[5] = minCell;
  BMSVINF_DATA[6] = (uint8_t) ((averageV >> 8) & 0xFF);
  BMSVINF_DATA[7] = (uint8_t) (averageV & 0xFF);

  HAL_CAN_AddTxMessage(&hcan, &TxHeader, BMSVINF_DATA, &TxMailbox);

}

void BMSTINF_message(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6]) {

  uint16_t cellTemp;
  uint16_t minT = 0xFFFF;
  uint8_t minCell;
  uint16_t maxT = 0x0000;
  uint8_t maxCell;
  uint16_t averageT;
  uint32_t sum = 0;

  for (uint8_t cell = 0; cell < 96; cell++) {

    cellTemp = 0;
    cellTemp = (uint16_t) (bmsData[cell][4]);
    cellTemp = cellTemp << 8;
		cellTemp += (uint16_t) (bmsData[cell][5]);

    if (cellTemp < minT) {
      minT = cellTemp;
      minCell = cell + 1;
    }

    if (cellTemp > maxT) {
      maxT = cellTemp;
      maxCell = cell + 1;
    }

    sum += cellTemp;
  }

  averageT = (uint16_t) (sum / (cfg.numOfICs * cfg.numOfTempPerIC));

  TxHeader.StdId = BMSTINF_ID;
  TxHeader.DLC = 8;
  uint8_t BMSTINF_DATA[8];

  BMSTINF_DATA[0] = (uint8_t) ((maxT >> 8) & 0xFF);
  BMSTINF_DATA[1] = (uint8_t) (maxT & 0xFF);
  BMSTINF_DATA[2] = maxCell;
  BMSTINF_DATA[3] = (uint8_t) ((minT >> 8) & 0xFF);
  BMSTINF_DATA[4] = (uint8_t) (minT & 0xFF);
  BMSTINF_DATA[5] = minCell;
  BMSTINF_DATA[6] = (uint8_t) ((averageT >> 8) & 0xFF);
  BMSTINF_DATA[7] = (uint8_t) (averageT & 0xFF);

  HAL_CAN_AddTxMessage(&hcan, &TxHeader, BMSTINF_DATA, &TxMailbox);  

};

void PACKSTAT_message(BMSconfigStructTypedef cfg, uint8_t bmsData[96][6]) {

  uint16_t channel1;
  uint16_t channel2;
  uint16_t pack_voltage;

  // +/- 20A (high res, channel 1)
  sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	channel1 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

  // +/- 500A (full range, channel 2)
  sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	channel2 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

  pack_voltage = (uint16_t) sumOfCells / 64;

  TxHeader.StdId = PACKSTAT_ID;
  TxHeader.DLC = 6;
  uint8_t PACKSTAT_DATA[6];

  PACKSTAT_DATA[0] = (uint8_t) ((pack_voltage >> 8) & 0xFF);
  PACKSTAT_DATA[1] = (uint8_t) (pack_voltage & 0xFF);
  PACKSTAT_DATA[2] = (uint8_t) ((channel1 >> 8) & 0xFF);
  PACKSTAT_DATA[3] = (uint8_t) (channel1 & 0xFF);
  PACKSTAT_DATA[4] = (uint8_t) ((channel2 >> 8) & 0xFF);
  PACKSTAT_DATA[5] = (uint8_t) (channel2 & 0xFF);

  HAL_CAN_AddTxMessage(&hcan, &TxHeader, PACKSTAT_DATA, &TxMailbox);

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
