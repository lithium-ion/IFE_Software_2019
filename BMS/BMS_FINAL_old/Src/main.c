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
#define CHARGER_ID      0x1806E5F4

#define normalCurrent   0x003E		// 6.2 A

/* hex value of ten times the current (A) when any cell exceeds lowerVoltage_Threshold */
#define lowerCurrent    0x000A			// 1 A

/* hex value of ten times the voltage (V) of the charger */
#define chargerVoltage  0x0FA0		// 400 V
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
uint8_t                 PEC_counter = 0;
uint8_t                 chargeRate = 2;

uint8_t voltageState = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void VOLTAGE_sort(BMSconfigStructTypedef cfg, uint16_t voltage[12][12]);
void CONNECTION_sort(BMSconfigStructTypedef cfg, bool connection[12][12]);
bool FAULT_check(BMSconfigStructTypedef cfg, uint16_t cellVoltage[12][12], bool tempFault[12][4], bool dcFault[12][4], bool cellConnection[12][12], uint8_t cellNumber[5], bool faultType[5], bool vPEC, bool tPEC);
void setDischarge(BMSconfigStructTypedef cfg, uint16_t cellVoltage[12][12], bool cellDischarge[12][8], bool bmsFault, bool fullDischarge[12][8]);
void checkDischarge(BMSconfigStructTypedef cfg, bool fullDischarge[12][8], uint16_t cellVoltage[12][12]);
uint16_t balancingThreshold(BMSconfigStructTypedef cfg);
void CELLVAL_message(BMSconfigStructTypedef cfg, uint16_t cellVoltage[12][12], uint16_t cellTemp[12][4], bool cellConnection[12][12], bool cellDischarge[12][8], bool dcFault[12][4], bool vreturn);
void BMSVINF_message(BMSconfigStructTypedef cfg, uint16_t cellVoltage[12][12]);
void BMSTINF_message(BMSconfigStructTypedef cfg, uint16_t cellTemp[12][4]);
void BMSSTAT_message(BMSconfigStructTypedef cfg, uint8_t cellNumber[5], bool faultType[5]);
void setChargerTxData();
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
  writeConfigAll(BMSconfig);

  uint16_t voltage[BMSconfig.numOfICs][BMSconfig.numOfCellInputs];        //12, 12
  uint16_t temp[BMSconfig.numOfICs][BMSconfig.numOfTempPerIC];            //12, 4
  bool tempdisconnect[BMSconfig.numOfICs][BMSconfig.numOfTempPerIC];      //12, 4
  bool overtemp[BMSconfig.numOfICs][BMSconfig.numOfTempPerIC];            //12, 4
  bool connection[BMSconfig.numOfICs][BMSconfig.numOfCellInputs];         //12, 12
  bool discharge[BMSconfig.numOfICs][BMSconfig.numOfCellsPerIC];          //12, 8
  bool fulldischarge[BMSconfig.numOfICs][BMSconfig.numOfCellsPerIC];    //12, 8

  fulldischarge[0][0] = 0;
  fulldischarge[0][1] = 0;
  fulldischarge[0][2] = 0;
  fulldischarge[0][3] = 0;
  fulldischarge[0][4] = 0;
  fulldischarge[0][5] = 0;
  fulldischarge[0][6] = 0;
  fulldischarge[0][7] = 0;
  fulldischarge[0][8] = 0;
  fulldischarge[0][9] = 0;
  fulldischarge[0][10] = 0;
  fulldischarge[0][11] = 0;

  bool vreturn = false;
  bool treturn = false;
  bool disconnectFault = false;
  uint8_t disconnectedCell;

  uint8_t number[5];
  bool faults[5];

  bool AIR = 0;
  bool CHARGE_EN = 0;

  /* TEST CHARGING */

  bool button = 0;
  bool lastButton = 0;
  bool fault = false;

  /* TEST CHARGING */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


    /* TEST CHARGING */

    //button = HAL_GPIO_ReadPin(GPIOB, AIR_Pin); //PB5

    /*if ((button == 1) && (lastButton == 0))
      voltageState++;

    if (voltageState > 1)
      voltageState = 0;

    lastButton = button;*/

    /*if (voltageState == 0) {
      for (uint8_t i = 0; i < 12; i++)
        voltage[0][i] = 40000;
      fault = false;
    }

    if (voltageState == 1) {
      for (uint8_t i = 0; i < 12; i++)
        voltage[0][i] = 40000;
      voltage[0][0] = 40600;
      voltage[0][1] = 40600;
      voltage[0][3] = 40600;
      fault = false;
    }

    if (voltageState == 2) {
      fault = true;
    }

    if (voltageState == 3) {

      for (uint8_t i = 0; i < 12; i++)
        voltage[0][i] = 41401;
      fault = false;

    }

    // Charger Comm Fault
    uint16_t count = TIM2->CNT;

    if (count > 20000) {
      HAL_GPIO_WritePin(GPIOC, DEBUG_Pin, GPIO_PIN_RESET);
      HAL_TIM_Base_Stop(&htim2);
    }

    BMSVINF_message(BMSconfig, voltage);

    setDischarge(BMSconfig, voltage, discharge, fault, fulldischarge);

    setChargerTxData();*/

    /*discharge[0][0] = 0;
    discharge[0][1] = 0;
    discharge[0][2] = 0;
    discharge[0][3] = 0;
    discharge[0][4] = 0;
    discharge[0][5] = 0;
    discharge[0][6] = 0;
    discharge[0][7] = 0;
    discharge[0][8] = 0;
    discharge[0][9] = 0;
    discharge[0][10] = 0;
    discharge[0][11] = 0;*/

    /*vreturn = 0;

    tempdisconnect[0][0] = 0;
    tempdisconnect[0][1] = 0;
    tempdisconnect[0][2] = 0;
    tempdisconnect[0][3] = 0;

    connection[0][0] = 1;
    connection[0][1] = 1;
    connection[0][2] = 1;
    connection[0][3] = 1;
    connection[0][4] = 1;
    connection[0][5] = 1;
    connection[0][6] = 1;
    connection[0][7] = 1;
    connection[0][8] = 1;
    connection[0][9] = 1;
    connection[0][10] = 1;
    connection[0][11] = 1;

    BMSSTAT_message(BMSconfig, number, faults);
    CELLVAL_message(BMSconfig, voltage, temp, connection, discharge, tempdisconnect, vreturn);

    checkDischarge(BMSconfig, fulldischarge, voltage);    

    HAL_Delay(500);*/

    //set faults to 0 by default, have one of the states be faulted
    //setDischarge, blink if still charging
    //check to see if every stop charging condition works correctly


    /* TEST CHARGING */







    vreturn = readAllCellVoltages(BMSconfig, voltage);
    VOLTAGE_sort(BMSconfig, voltage);
    BMSVINF_message(BMSconfig, voltage);
    
    treturn = readAllCellTemps(BMSconfig, temp, tempdisconnect, overtemp);
    BMSTINF_message(BMSconfig, temp);
    
    checkAllCellConnections(BMSconfig, voltage, connection);
    CONNECTION_sort(BMSconfig, connection);
    AIR = HAL_GPIO_ReadPin(GPIOB, AIR_Pin);
    CHARGE_EN = HAL_GPIO_ReadPin(GPIOB, CHARGE_EN_Pin);

    /*if ((AIR == 0) && (CHARGE_EN == 0)) {

      if (chargeRate != 0)
        setDischarge(BMSconfig, voltage, discharge, chargerate, faults);

      //send charger CAN message
      setChargerTxData();
    
      if (chargeRate != 0) {
        dischargeCellGroups(BMSconfig, discharge);
        //blink LED to test
      }

      else if (chargeRate == 0) {
        checkDischarge(BMSconfig, fulldischarge, voltage);
        dischargeCellGroups(BMSconfig, fulldischarge);
      }
    }
    else {

      //stop discharging for every cell

    }*/

    FAULT_check(BMSconfig, voltage, overtemp, tempdisconnect, connection, number, faults, vreturn, treturn);
    BMSSTAT_message(BMSconfig, number, faults);

    CELLVAL_message(BMSconfig, voltage, temp, connection, discharge, tempdisconnect, vreturn);

    //if (vreturn == 1)
    
    HAL_GPIO_TogglePin(GPIOC, DEBUG_Pin);

    HAL_Delay(100);

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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get RX message */
  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);

  uint16_t ID = RxHeader.StdId;
  uint32_t ExtID = RxHeader.ExtId;

  /* TESTING */
  if (ID == 0x123) {

    //HAL_GPIO_TogglePin(GPIOC, DEBUG_Pin);
    voltageState = RxData[0];
  }
  /* TESTING */

  if (ExtID == CHARGER_ID) {
    TIM2->CNT = 0; // reset timer
    HAL_TIM_Base_Start(&htim2);
    RxHeader.ExtId = 0;
    //HAL_GPIO_TogglePin(GPIOC, DEBUG_Pin);
    HAL_GPIO_WritePin(GPIOC, DEBUG_Pin, GPIO_PIN_SET);
  }
}

void VOLTAGE_sort(BMSconfigStructTypedef cfg, uint16_t voltage[12][12]) {
  for (uint8_t i = 0; i < cfg.numOfICs; i++) {
    voltage[i][4] = voltage[i][6];
    voltage[i][5] = voltage[i][7];
    voltage[i][6] = voltage[i][8];
    voltage[i][7] = voltage[i][9];
  }
}

void CONNECTION_sort(BMSconfigStructTypedef cfg, bool connection[12][12]) {
  for (uint8_t i = 0; i < cfg.numOfICs; i++) {
    connection[i][4] = connection[i][6];
    connection[i][5] = connection[i][7];
    connection[i][6] = connection[i][8];
    connection[i][7] = connection[i][9];
  }
}

bool FAULT_check(BMSconfigStructTypedef cfg, uint16_t cellVoltage[12][12], bool tempFault[12][4], bool dcFault[12][4], bool cellConnection[12][12], uint8_t cellNumber[5], bool faultType[5], bool vPEC, bool tPEC) {

  bool BMS_FAULT = false;
  
  cellNumber[0] = 0; // OV cell number
  cellNumber[1] = 0; // UV cell number
  cellNumber[2] = 0; // OT cell number
  cellNumber[3] = 0; // disconnected cell cell number
  cellNumber[4] = 0; // disconnected temp sensor cell number

  faultType[0] = 0; // disconnected temp sensor fault
  faultType[1] = 0; // disconnected cell fault
  faultType[2] = 0; // OT fault
  faultType[3] = 0; // UV fault
  faultType[4] = 0; // OV fault
      
  for (uint8_t i = 0; i < cfg.numOfICs; i++) {
    for (uint8_t j = 0; j < cfg.numOfCellsPerIC; j++) {

      // OV Fault
      if (cellVoltage[i][j] > cfg.OV_threshold) {
        BMS_FAULT = true;
        faultType[4] = true;
        cellNumber[0] = i * cfg.numOfCellsPerIC + j + 1;
      }

      // UV Fault
      if (cellVoltage[i][j] < cfg.UV_threshold) {
        BMS_FAULT = true;
        faultType[3] = true;
        cellNumber[1] = i * cfg.numOfCellsPerIC + j + 1;
      }

      // Disconnected Cell Fault
      if (cellConnection[i][j] == 0) {
        BMS_FAULT = true;
        faultType[1] = true;
        cellNumber[3] = i * cfg.numOfCellsPerIC + j + 1;
      }
    }
  }

  for (uint8_t i = 0; i < cfg.numOfICs; i++) {
    for (uint8_t j = 0; j < cfg.numOfTempPerIC; j++) {

      // OT Fault
      if (tempFault[i][j] == 1) {
        BMS_FAULT = true;
        faultType[2] = true;
        cellNumber[2] = i * cfg.numOfCellsPerIC + j*2 + 1;
      }

      // Disconnected Temp Sensor Fault
      if (dcFault[i][j] == 1) {
        BMS_FAULT = true;
        faultType[0] = true;
        cellNumber[4] = i * cfg.numOfCellsPerIC + j*2 + 1;
      }
    }
  }

  // Invalid PEC Fault
  if ((vPEC == false) || (tPEC == false))
    PEC_counter++;
  else
    PEC_counter = 0;
  
  if (PEC_counter >= cfg.invalidPECcount)
    BMS_FAULT = true;

  // Charger Comm Fault
  uint16_t count = TIM2->CNT;

  if (count > 20000) {
    HAL_GPIO_TogglePin(GPIOC, DEBUG_Pin);
    HAL_TIM_Base_Stop(&htim2);
  }

  if (BMS_FAULT == false)
    HAL_GPIO_WritePin(GPIOB, BMS_FLT_Pin, GPIO_PIN_RESET);
  if (BMS_FAULT == true)
    HAL_GPIO_WritePin(GPIOB, BMS_FLT_Pin, GPIO_PIN_SET);

  return BMS_FAULT;
}

void setDischarge(BMSconfigStructTypedef cfg, uint16_t cellVoltage[12][12], bool cellDischarge[12][8], bool bmsFault, bool fullDischarge[12][8]) {

  uint16_t threshold;
	chargeRate = 2; // initialize the charging current to normal operation

  // if any BMS fault(change this), set charge current to 0
  if (bmsFault == 1)
    chargeRate = 0;
	
	for (int board = 0; board < cfg.numOfICs; board++) {
		for (int cell = 0; cell < cfg.numOfCellsPerIC; cell++) {

      // if any cell voltage is much greater than the minimum (>200mV), stop charging and discharge that cell to the minimum
      if (cellVoltage[board][cell] > (minimum + cfg.max_difference)) {
        chargeRate = 0;
        fullDischarge[board][cell] = 1;
      }

      // if any cell voltage is greater than some absolute threshold (4.18V), stop charging and discharge that cell to the minimum
      // could discharge to a fixed value (4.15V) instead
      if (cellVoltage[board][cell] > cfg.stopCharge_threshold) {
        chargeRate = 0;
        fullDischarge[board][cell] = 1;
      }

      // if still charging AND cells above ~3V (add this)
      if (chargeRate != 0) {

        // if any cell is above some absolute threshold, charge slower 
        if (cellVoltage[board][cell] > cfg.slowCharge_threshold)
          chargeRate = 1;

        // determine the relative balancing threshold based on minimum voltage
        threshold = balancingThreshold(cfg);

        if (cellVoltage[board][cell] > (minimum + threshold))
          cellDischarge[board][cell] = 1;
        else
          cellDischarge[board][cell] = 0;
      }
    }
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

void checkDischarge(BMSconfigStructTypedef cfg, bool fullDischarge[12][8], uint16_t cellVoltage[12][12]) {

  uint8_t sum = 0;

  for (uint8_t board = 0; board < cfg.numOfICs; board++) {
    for (uint8_t cell = 0; cell < cfg.numOfCellsPerIC; cell++) {
      if (fullDischarge[board][cell] == 1) {
        if (cellVoltage[board][cell] <= minimum)
          fullDischarge[board][cell] = 0;
        else
          sum = sum + 1;
      }
    }
  }

  if (sum == 0)
    chargeRate = 2;

}

void setChargerTxData() {

	/* voltage data (hex value of desired voltage (V) times 10)*/
	ChargerTxData[0] = (uint8_t)(chargerVoltage >> 8);
	ChargerTxData[1] = (uint8_t)chargerVoltage;

	/* set the current data (hex value of desired current (A) times 10) */
	switch (chargeRate) {
		case 1:
			/* lower current */
			ChargerTxData[2] = (uint8_t)(lowerCurrent >> 8);
			ChargerTxData[3] = (uint8_t)lowerCurrent;
			break;

		case 2:
			/* normal current */
			ChargerTxData[2] = (uint8_t)(normalCurrent >> 8);
			ChargerTxData[3] = (uint8_t)normalCurrent;
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

void CELLVAL_message(BMSconfigStructTypedef cfg, uint16_t cellVoltage[12][12], uint16_t cellTemp[12][4], bool cellConnection[12][12], bool cellDischarge[12][8], bool dcFault[12][4], bool vreturn) {
	
  TxHeader.StdId = CELLVAL_ID;
  TxHeader.DLC = 6;

  for (uint8_t i = 0; i < cfg.numOfICs; i++) {
    for (uint8_t j = 0; j < cfg.numOfCellsPerIC; j++) {
      CELLVAL_DATA[0] = i * cfg.numOfCellsPerIC + j + 1; // cell number
      CELLVAL_DATA[1] = (uint8_t) (((dcFault[i][(uint8_t) j/2] << 3) & 0x08) | ((cellDischarge[i][j] << 2) & 0x04) | ((vreturn << 1) & 0x02) | ((cellConnection[i][j]) & 0x01));
      CELLVAL_DATA[2] = (uint8_t) ((cellVoltage[i][j] >> 8) & 0xFF);
      CELLVAL_DATA[3] = (uint8_t) (cellVoltage[i][j] & 0xFF);
      CELLVAL_DATA[4] = (uint8_t) ((cellTemp[i][(uint8_t) j/2] >> 8) & 0xFF);
      CELLVAL_DATA[5] = (uint8_t) (cellTemp[i][(uint8_t) j/2] & 0xFF);

      HAL_CAN_AddTxMessage(&hcan, &TxHeader, CELLVAL_DATA, &TxMailbox);
      HAL_Delay(10);
    }
  }
}

void BMSSTAT_message(BMSconfigStructTypedef cfg, uint8_t cellNumber[5], bool faultType[5]) {

  TxHeader.StdId = BMSSTAT_ID;
  TxHeader.DLC = 6;

  BMSSTAT_DATA[0] = (uint8_t) ((chargeRate << 5) | (faultType[0] << 4) | (faultType[1] << 3) | (faultType[2] << 2) | (faultType[3] << 1) | (faultType[4]));
  BMSSTAT_DATA[1] = cellNumber[0]; // OV cell number
  BMSSTAT_DATA[2] = cellNumber[1]; // UV cell number
  BMSSTAT_DATA[3] = cellNumber[2]; // OT cell number
  BMSSTAT_DATA[4] = cellNumber[3]; // disconnected cell cell number
  BMSSTAT_DATA[5] = cellNumber[4]; // disconnected temp sensor cell number

  HAL_CAN_AddTxMessage(&hcan, &TxHeader, BMSSTAT_DATA, &TxMailbox);
}

void BMSVINF_message(BMSconfigStructTypedef cfg, uint16_t cellVoltage[12][12]) {

  uint16_t minV;
  uint8_t minCell;
  uint16_t maxV;
  uint8_t maxCell;
  uint16_t averageV;
  uint32_t sum = 0;

  minV = cellVoltage[0][0];
  minCell = 1;
  maxV = cellVoltage[0][0];
  maxCell = 1;

  for (int i = 0; i < cfg.numOfICs; i++) {
		for (int j = 0; j < cfg.numOfCellsPerIC; j++) {

			if (cellVoltage[i][j] < minV) {
        minV = cellVoltage[i][j];
        minCell = i * cfg.numOfCellsPerIC + j + 1;
      }

      if (cellVoltage[i][j] > maxV) {
        maxV = cellVoltage[i][j];
        maxCell = i * cfg.numOfCellsPerIC + j + 1;
      }

      sum = sum + cellVoltage[i][j];

		}
  }

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

void BMSTINF_message(BMSconfigStructTypedef cfg, uint16_t cellTemp[12][4]) {

  uint16_t minT;
  uint8_t minCell;
  uint16_t maxT;
  uint8_t maxCell;
  uint16_t averageT;
  uint32_t sum = 0;

  minT = cellTemp[0][0];
  minCell = 1;
  maxT = cellTemp[0][0];
  maxCell = 1;

  for (int i = 0; i < cfg.numOfICs; i++) {
		for (int j = 0; j < cfg.numOfTempPerIC; j++) {

			if (cellTemp[i][j] < minT) {
        minT = cellTemp[i][j];
        minCell = i * cfg.numOfTempPerIC + j*2 + 1;
      }

      if (cellTemp[i][j] > maxT) {
        maxT = cellTemp[i][j];
        maxCell = i * cfg.numOfTempPerIC + j*2 + 1;
      }

      sum = sum + cellTemp[i][j];

		}
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
