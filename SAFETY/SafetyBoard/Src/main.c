/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



#define APPS_STDID        0x300
#define BTSF_STDID        0x301
#define FAULTS            0x0D0
// FAULT DEFS
  #define FAULT_ACTIVE    0xFF
  #define FAULT_INACTIVE  0x00

//FAULT DEGS
#define CAR_STATE         0x0D1
// CAR STATES
  #define LV_ON           0x01
  #define PRECHARGED      0x02
  #define ENABLE_FLIPPED  0x04
  #define RTDS_SOUND      0x08
  #define PWR_AVAILABLE   0x10 
  #define SOFT_FAULT      0x20
// CAR STATES
// 1 volt is 1241.21
//throttle A .66-3.2 scaled to .396-1.866
//throttle B .33-2.7 scaled to .198-1.62
// That means 5% from 0 is .4705 volts
const uint16_t ThrottleA_5 = 584;
// That means 25% from 0 is .7635 volts
const uint16_t ThrottleA_25 = 947;
const uint16_t brakeThreshold = 2000; //80; >> 3.5 volts on 5 v scale, 2.1 on 3.3v scale
const uint16_t RTD_Threshold = 2500; // NO idea for this

// APPS is .35 volts, so .21 volts scaled
//This values is 0-100 based
const int APPS_difference = 600; //~0.5 voltz

//These are based off above values measured
const int throttle_A_min = 491;
const int throttle_A_max = 2316;
const int throttle_B_min = 246;
const int throttle_B_max = 2010;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

uint16_t brakePos;
uint16_t steeringPos;
uint16_t brakePressure_1;
uint16_t brakePressure_2;
uint16_t throttle_A;
uint16_t throttle_B;


//need to figure out:
uint16_t max_throttle = 10;


//STATE FLAGS
char Prev_State = 0x00;     //boolean
int hardFaultFlag = 0;  //boolean
char BTSF_ACTIVE = 0x00;

// CAR STAT STATES FOR CAN
CAN_TxHeaderTypeDef TxCar_state;
uint8_t TxCar_state_data[8] = {0x00,0x45,0x45,0x86,0x56,0x97,0x67,0x98};
uint32_t TxCar_stateMailbox;

//CAN FAULT VARIABLES
CAN_TxHeaderTypeDef TxFaults;
uint8_t TxFault_data[8] = {0x55, 0x55, 0x55, 0x55,0x55, 0x55, 0x55, 0x55};
/*Set these values: 0xFF = Fault present
                    0x00 = No fault*/
uint8_t bms;        //TxData[0]
uint8_t imd;        //TxData[1]
uint8_t bspd;       //TxData[2]
uint8_t apps;       //TxData[3]
uint32_t TxFaultsMailbox;

//For Timers
extern uint32_t millisTimer;
extern uint32_t RTDS_Timer;
extern uint32_t CAN_Timer;

ADC_ChannelConfTypeDef sConfig = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

char checkBTSF();
char checkAPPS();
char APPS_Diff();
void sendFaultMsg();
void sendCar_state();
void readFaults();
void car_state_machine(char STATE);
uint16_t updateADC(int channel); 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//int refTime = millis();
//int APPSFlag = 0; //boolean

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  //HAL_Delay(200);


  //For Timers
  millisTimer = 100; //100 millis
  RTDS_Timer = 2500; //3 seconds
  CAN_Timer = 1000; //timer to send message every second

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // Update SystemCoreClock value
  SystemCoreClockUpdate();
  // Configure the SysTick timer to overflow every 1 ms
  SysTick_Config(SystemCoreClock / 1000);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

   while (1)
   {
     //READ FOR PRECHARGE
   if(HAL_GPIO_ReadPin(GPIOB, PRECHARGE_COMPLETE_Pin) == GPIO_PIN_RESET){
     car_state_machine(PRECHARGED);
     //READ FOR ENABLE
    if(HAL_GPIO_ReadPin(GPIOB,ENABLE_IN_Pin) == GPIO_PIN_RESET){
       car_state_machine(ENABLE_FLIPPED);
       //ADC for Brake pressure
       brakePressure_2 = updateADC(BRAKE_PRESSURE_2_ADC_CHANNEL);
       //SEE IF BRAKE IS PRESSED 
       if(brakePressure_2 >= RTD_Threshold){
       //set 3 second timer
        if(TxCar_state_data[0] == ENABLE_FLIPPED) {
         RTDS_Timer = 2500; //change to 3000 for 3 seconds
         car_state_machine(RTDS_SOUND);
          
         HAL_GPIO_WritePin(GPIOB, RTD_EN_Pin | RTDS_EN_Pin | BTSF_EN_Pin | APPS_EN_Pin, GPIO_PIN_SET);
         } 
       //RTD Sound Enable Play sound for 2.5 seconds 
       }
      
       // IF OUR TIMER IS OVER FINALLY //we also stop timer if we started it from RTDS state or Soft fault state
         if((RTDS_Timer == 0) && (TxCar_state_data[0] & 0x38)){
           HAL_GPIO_WritePin(GPIOB, RTDS_EN_Pin, GPIO_PIN_RESET);
           //SeT pwr
           car_state_machine(PWR_AVAILABLE);
            }// rtds buzzer stop
     }//END OF RTD SEQUENCE*/
     else if(HAL_GPIO_ReadPin(GPIOB,ENABLE_IN_Pin) == GPIO_PIN_SET)
     { //IF WE HAVE DE ENABLED LA COCHE
       TxCar_state_data[0] = PRECHARGED;
       HAL_GPIO_WritePin(GPIOB, RTD_EN_Pin,GPIO_PIN_RESET);
       HAL_Delay(500);//THIS IS PUT HERE TO AVOID BOUNCING WHEN ITS REMOVED
     }
   } // end of if precharge complete statement
      if(RTDS_Timer == 0)
       HAL_GPIO_WritePin(GPIOB, RTDS_EN_Pin, GPIO_PIN_RESET);
  
     // SEQUENCE FOR CHECKING SOFT FAULTS
     if(TxCar_state_data[0] >= RTDS_SOUND){
     if (checkBTSF()){
       HAL_GPIO_WritePin(GPIOB,BTSF_EN_Pin | APPS_EN_Pin,GPIO_PIN_RESET);
       TxCar_state_data[0] = SOFT_FAULT;
     }
     else if(TxCar_state_data[0] == SOFT_FAULT) {
         TxCar_state_data[0] = PWR_AVAILABLE;
         HAL_GPIO_WritePin(GPIOB, BTSF_EN_Pin|APPS_EN_Pin ,GPIO_PIN_SET);
     }
   }

   readFaults(); 
   if (CAN_Timer == 0){ //sending routine message every 1 second
     sendFaultMsg();
     sendCar_state();
     CAN_Timer = 1000;
     //HAL_GPIO_WritePin(BRAKE_LIGHT_EN_GPIO_Port, BRAKE_LIGHT_EN_Pin, GPIO_PIN_SET);

   }

   //Brake Light
   brakePressure_2 = updateADC(BRAKE_PRESSURE_2_ADC_CHANNEL);
   if(brakePressure_2 > brakeThreshold){
     HAL_GPIO_WritePin(GPIOB, BRAKE_LIGHT_EN_Pin, GPIO_PIN_SET);
   }
   else
     HAL_GPIO_WritePin(GPIOB, BRAKE_LIGHT_EN_Pin, GPIO_PIN_RESET);


     /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   }
  /* USER CODE END 3 */

  //TEST WHILE
 /* while(1){

    HAL_GPIO_WritePin(BRAKE_LIGHT_EN_GPIO_Port, BRAKE_LIGHT_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(500);
  }
  */

  
}

/********************************************************************************/
//Checks for APPS errors, sends fault if there is one 
//Returns one if difference in throttleA/B is greater than 10% for 100 ms
//Returns zero if else
/********************************************************************************/
char checkAPPS(){

  throttle_A = updateADC(THROTTLE_A_ADC_CHANNEL); 
  throttle_B = updateADC(THROTTLE_B_ADC_CHANNEL); 

/*
  if(throttle_A > throttle_B){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(100);
  }
  else
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  return 1;*/
  
  //Throttles Agree
  if(APPS_Diff() == 0){
   millisTimer = 100;
   //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  }
  
  //APPS_EN Fault
  if(millisTimer == 0){ //this occurs when APPS_diff is true for > 100 ms
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    return 1; //will set driving = 0;
  }
  return 0; //APPS is good

}

/********************************************************************************/
//Checks for BTSF errors, If the brake and throttle are pressed at the same time, above a certain threshold
//Returns one if fault was sensed and sent out
//zero if nothing is detected
/********************************************************************************/
char checkBTSF(){
  brakePressure_2 = updateADC(BRAKE_PRESSURE_2_ADC_CHANNEL);
  throttle_A = updateADC(THROTTLE_A_ADC_CHANNEL); 
  
  if(BTSF_ACTIVE)
  {
    if(throttle_A <= ThrottleA_5)
    {
      BTSF_ACTIVE = 0x00;
      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      return 0;
    }
    return 1;
  }

  if((brakePressure_2 > brakeThreshold) && (throttle_A > ThrottleA_25)){
       //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
       BTSF_ACTIVE = 0xFF;
       return 1;
  }
  
  
  return 0;
}

/********************************************************************************/
// This function updates the ADC values for all positions/pressures
//
//
/********************************************************************************/
uint16_t updateADC(int channel){  
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  if (channel == BRAKE_POS_ADC_CHANNEL) //Brake position 
    sConfig.Channel = ADC_CHANNEL_0;
  if (channel == STEERING_POS_ADC_CHANNEL) //steering position 
    sConfig.Channel = ADC_CHANNEL_1;
  if (channel == BRAKE_PRESSURE_1_ADC_CHANNEL) //brake pressure 1
    sConfig.Channel = ADC_CHANNEL_3;
  if (channel == BRAKE_PRESSURE_2_ADC_CHANNEL) //brake pressure 2
    sConfig.Channel = ADC_CHANNEL_2;
  if (channel == THROTTLE_A_ADC_CHANNEL) //throttle A
    sConfig.Channel = ADC_CHANNEL_8;
  if (channel == THROTTLE_B_ADC_CHANNEL) //throttle B 
    sConfig.Channel = ADC_CHANNEL_9;

  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  return HAL_ADC_GetValue(&hadc1);
  //HAL_ADC_Stop(&hadc1);
  //////////////////////////////////////////////////////////////
  /*
  HAL_ADC_Start(&hadc1);
  
  HAL_ADC_PollForConversion(&hadc1, 1000);
  brakePos = HAL_ADC_GetValue(&hadc1);  //brakePos
  HAL_ADC_PollForConversion(&hadc1, 1000);
  steeringPos = HAL_ADC_GetValue(&hadc1);  //steeringPos
  HAL_ADC_PollForConversion(&hadc1, 1000);
  brakePressure_1 = HAL_ADC_GetValue(&hadc1);  //brakePressure_1
  HAL_ADC_PollForConversion(&hadc1, 1000);
  brakePressure_2 = HAL_ADC_GetValue(&hadc1);  //brakePressure_2
  HAL_ADC_PollForConversion(&hadc1, 1000);
  throttle_A = HAL_ADC_GetValue(&hadc1);  //throttle_A
  HAL_ADC_PollForConversion(&hadc1, 1000);
  throttle_B = HAL_ADC_GetValue(&hadc1);  //throttle_B
  
  HAL_ADC_Stop(&hadc1);
  */
}

/********************************************************************************/
//Determines if the throttle percent difference is above 10%
//Returns: 1 if difference > 10%
//       0 if everything is good 
//
//Throttle_A will get values 491 to 2316 (throttle_A_min to max)
//Throttle_B will get values 246 to 2010 (throttle_B_min to max)
/********************************************************************************/
char APPS_Diff(){

//0.66 3.11
//0.32 2.77
 // int t_A = (int)throttle_A;
  //int t_B = (int)throttle_B;

  if(abs(throttle_A - throttle_B) > APPS_difference){ //600 ~ 0.5v
    return 1;
  }
  else return 0;

  /*

  t_A = (t_A - throttle_A_min)/(throttle_A_max - throttle_A_min); //Normalize both values to be 0-100
  t_B = (t_B - throttle_B_min)/(throttle_B_max - throttle_B_min);

  if(abs(t_A-t_B) > APPS_difference)
    return 1;
  else
    return 0;

    */


}
void sendFaultMsg(){
  TxFault_data[0] = bms;  //Set all the data (faults) to their current values
  TxFault_data[1] = imd;
  TxFault_data[2] = bspd;
  TxFault_data[3] = apps;
  HAL_CAN_AddTxMessage(&hcan, &TxFaults, TxFault_data, &TxFaultsMailbox);
}

void sendCar_state(){
  HAL_CAN_AddTxMessage(&hcan, &TxCar_state,TxCar_state_data, &TxCar_stateMailbox);
}

void car_state_machine(char STATE)
{
  if(STATE > TxCar_state_data[0])
  {
    TxCar_state_data[0] = STATE;
  }
}

void readFaults(){
  if (HAL_GPIO_ReadPin(GPIOD, FAULT_BSPD_STATUS_Pin) == GPIO_PIN_SET)
    bspd = FAULT_ACTIVE;
  else bspd = FAULT_INACTIVE;
  
  if (HAL_GPIO_ReadPin(GPIOC, FAULT_IMD_STATUS_Pin) == GPIO_PIN_SET)
    imd = FAULT_ACTIVE;
  else imd = FAULT_INACTIVE;
  
  if (HAL_GPIO_ReadPin(GPIOC, FAULT_BMS_STATUS_Pin) == GPIO_PIN_SET)
    bms = FAULT_ACTIVE;
  else bms = FAULT_INACTIVE;

  if(bms || imd){
    TxCar_state_data[0] = LV_ON;
    HAL_GPIO_WritePin(GPIOB, RTD_EN_Pin | RTDS_EN_Pin, GPIO_PIN_RESET);
  }

}

/* USER CODE END 0 */



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

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = DISABLE; //enable 
  hadc1.Init.ContinuousConvMode = DISABLE; //enable 
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  //Brake position
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  //Steering Position 
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
    Error_Handler();
  }
  
  //Brake Pressure 1
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
    Error_Handler();
  }
  
  //Brake Pressure 2
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
    Error_Handler();
  }
  
  //ADC for throttle_A
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
    Error_Handler();
  }
  
  //ADC for throttle_B
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
    Error_Handler();
  }
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


  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE; //enable this if no other nodes on bus
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  
    /* USER CODE BEGIN CAN_Init 1 */
 // HAL_CAN_Start(&hcan);
  TxFaults.StdId = FAULTS;         // CAN standard ID
  TxFaults.ExtId = 0x01;          // CAN extended ID
  TxFaults.RTR = CAN_RTR_DATA;      // CAN frame type
  TxFaults.IDE = CAN_ID_STD;        // CAN ID type
  TxFaults.DLC = 8;             // CAN frame length in bytes
  TxFaults.TransmitGlobalTime = DISABLE;  // CAN timestamp in TxData[6] and TxData[7]

  TxCar_state.StdId = CAR_STATE;         // CAN standard ID
  TxCar_state.ExtId = 0x01;          // CAN extended ID
  TxCar_state.RTR = CAN_RTR_DATA;      // CAN frame type
  TxCar_state.IDE = CAN_ID_STD;        // CAN ID type
  TxCar_state.DLC = 8;             // CAN frame length in bytes
  TxCar_state.TransmitGlobalTime = DISABLE;

  /* USER CODE END CAN_Init 1 */

    HAL_CAN_Start(&hcan);
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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
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
  HAL_GPIO_WritePin(GPIOB, BRAKE_LIGHT_EN_Pin|RTDS_EN_Pin|APPS_EN_Pin|RTD_EN_Pin 
                          |BTSF_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : FAULT_IMD_STATUS_Pin FAULT_BMS_STATUS_Pin */
  GPIO_InitStruct.Pin = FAULT_IMD_STATUS_Pin|FAULT_BMS_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : FAULT_BSPD_STATUS_Pin */
  GPIO_InitStruct.Pin = FAULT_BSPD_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAULT_BSPD_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HV_CONNECTED_Pin ENABLE_IN_Pin PRECHARGE_COMPLETE_Pin */
  GPIO_InitStruct.Pin = HV_CONNECTED_Pin|ENABLE_IN_Pin|PRECHARGE_COMPLETE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BRAKE_LIGHT_EN_Pin RTDS_EN_Pin APPS_EN_Pin RTD_EN_Pin 
                           BTSF_EN_Pin */
  GPIO_InitStruct.Pin = BRAKE_LIGHT_EN_Pin|RTDS_EN_Pin|APPS_EN_Pin|RTD_EN_Pin 
                          |BTSF_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

}

/* USER CODE BEGIN 4 */


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
