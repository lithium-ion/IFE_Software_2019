/*
*	Dec 8, 2018
*	IFE 2018-19
	Matt Vlasaty

	This program receives CAN messages and processes them through an interrupt.
	The files were generated with STM32CubeMX using version 1.6.1 of the HAL library for the STM32F1xx series.
	Change the location of gcc-arm-none-eabi in the makefile to build.

*/
	

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_CAN_Init();

	/* Initialize the Rx interrupt */
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(500);
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

	CAN_FilterConfTypeDef	sFilterConfig;
	static CanTxMsgTypeDef TxMessage;
	static CanRxMsgTypeDef RxMessage;

	hcan.pTxMsg = &TxMessage;
	hcan.pRxMsg = &RxMessage;

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_3TQ;
  hcan.Init.BS2 = CAN_BS2_4TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = ENABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;

	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

}

/*--- Interrupt callback function ---*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle)
{
	//unsigned char data[8];
	//data[0] = hcan.pRxMsg->Data[0];
	//HAL_UART_Transmit(&huart1, data, sizeof(data), 1000);

  // /*--- LED debugging ---*/
	// if (hcan.pRxMsg->StdId == 0xA0)
	// {
	// 	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	// }
  // else if (hcan.pRxMsg->StdId == 0xA5){
  //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // }
  // else if (hcan.pRxMsg->StdId == 0xA6){
  //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // }
  // else if (hcan.pRxMsg->StdId == 0xA7){
  //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // }
  // else if (hcan.pRxMsg->StdId == 0xA8){
  //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // }
  // else if (hcan.pRxMsg->StdId == 0xA9){
  //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // }
  // else if (hcan.pRxMsg->StdId == 0xAB){
  //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // }
  // else if (hcan.pRxMsg->StdId == 0xAC){
  //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // }
  // else if (hcan.pRxMsg->StdId == 0xAD){
  //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  // }

  if (hcan.pRxMsg->StdId == 0xA0)
	{
    unsigned char a_temp = (hcan.pRxMsg->Data[0] + hcan.pRxMsg->Data[1]*256) / 10;
    unsigned char b_temp = (hcan.pRxMsg->Data[2] + hcan.pRxMsg->Data[3]*256) / 10;
    unsigned char c_temp = (hcan.pRxMsg->Data[4] + hcan.pRxMsg->Data[5]*256) / 10;
    unsigned char g_temp = (hcan.pRxMsg->Data[6] + hcan.pRxMsg->Data[7]*256) / 10;
	}
  else if (hcan.pRxMsg->StdId == 0xA1){
    unsigned char cb_temp = (hcan.pRxMsg->Data[0] + hcan.pRxMsg->Data[1]*256) / 10;
    unsigned char one_temp = (hcan.pRxMsg->Data[2] + hcan.pRxMsg->Data[3]*256) / 10;
    unsigned char two_temp = (hcan.pRxMsg->Data[4] + hcan.pRxMsg->Data[5]*256) / 10;
    unsigned char three_temp = (hcan.pRxMsg->Data[6] + hcan.pRxMsg->Data[7]*256) / 10;    
  }
  else if (hcan.pRxMsg->StdId == 0xA2){
    unsigned char four_temp = (hcan.pRxMsg->Data[0] + hcan.pRxMsg->Data[1]*256) / 10;
    unsigned char five_temp = (hcan.pRxMsg->Data[2] + hcan.pRxMsg->Data[3]*256) / 10;
    unsigned char motor_temp = (hcan.pRxMsg->Data[4] + hcan.pRxMsg->Data[5]*256) / 10;
    unsigned char torque_shudder = (hcan.pRxMsg->Data[6] + hcan.pRxMsg->Data[7]*256) / 10;    
  }
  else if (hcan.pRxMsg->StdId == 0xA4){
    unsigned char d1 = hcan.pRxMsg->Data[0];
    unsigned char d2 = hcan.pRxMsg->Data[1];
    unsigned char d3 = hcan.pRxMsg->Data[2];
    unsigned char d4 = hcan.pRxMsg->Data[3];
    unsigned char d5 = hcan.pRxMsg->Data[4];
    unsigned char d6 = hcan.pRxMsg->Data[5];
    unsigned char d7 = hcan.pRxMsg->Data[6];
    unsigned char d8 = hcan.pRxMsg->Data[7];
  }
  else if (hcan.pRxMsg->StdId == 0xA5){
    unsigned char motor_angle = (hcan.pRxMsg->Data[0] + hcan.pRxMsg->Data[1]*256) / 10;
    unsigned char motor_speed = (hcan.pRxMsg->Data[2] + hcan.pRxMsg->Data[3]*256);
    unsigned char motor_freq = (hcan.pRxMsg->Data[4] + hcan.pRxMsg->Data[5]*256) / 10;
    unsigned char delta_filter = (hcan.pRxMsg->Data[6] + hcan.pRxMsg->Data[7]*256) / 10;     
  }
  else if (hcan.pRxMsg->StdId == 0xA6){ 
    unsigned char a_curr = (hcan.pRxMsg->Data[0] + hcan.pRxMsg->Data[1]*256) / 10;
    unsigned char b_curr = (hcan.pRxMsg->Data[2] + hcan.pRxMsg->Data[3]*256) / 10;
    unsigned char c_curr = (hcan.pRxMsg->Data[4] + hcan.pRxMsg->Data[5]*256) / 10;
    unsigned char dc_curr = (hcan.pRxMsg->Data[6] + hcan.pRxMsg->Data[7]*256) / 10;     
  }
  else if (hcan.pRxMsg->StdId == 0xA7){
    unsigned char dc_volt = (hcan.pRxMsg->Data[0] + hcan.pRxMsg->Data[1]*256) / 10;
    unsigned char out_volt = (hcan.pRxMsg->Data[2] + hcan.pRxMsg->Data[3]*256) / 10;
    unsigned char vab_volt = (hcan.pRxMsg->Data[4] + hcan.pRxMsg->Data[5]*256) / 10;
    unsigned char vbc_volt = (hcan.pRxMsg->Data[6] + hcan.pRxMsg->Data[7]*256) / 10;     
  }
  else if (hcan.pRxMsg->StdId == 0xA8){
    unsigned char flux_cmd = (hcan.pRxMsg->Data[0] + hcan.pRxMsg->Data[1]*256) / 1000;
    unsigned char flux_feed = (hcan.pRxMsg->Data[2] + hcan.pRxMsg->Data[3]*256) / 1000;
    unsigned char id_feed = (hcan.pRxMsg->Data[4] + hcan.pRxMsg->Data[5]*256) / 10;
    unsigned char iq_feed = (hcan.pRxMsg->Data[6] + hcan.pRxMsg->Data[7]*256) / 10;  
  }
  else if (hcan.pRxMsg->StdId == 0xA9){
    unsigned char v_1 = (hcan.pRxMsg->Data[0] + hcan.pRxMsg->Data[1]*256) / 100;
    unsigned char v_2 = (hcan.pRxMsg->Data[2] + hcan.pRxMsg->Data[3]*256) / 100;
    unsigned char v_5 = (hcan.pRxMsg->Data[4] + hcan.pRxMsg->Data[5]*256) / 100;
    unsigned char v_12 = (hcan.pRxMsg->Data[6] + hcan.pRxMsg->Data[7]*256) / 100;  
  }
  else if (hcan.pRxMsg->StdId == 0xAB){
    unsigned char posy_lo = (hcan.pRxMsg->Data[0] + hcan.pRxMsg->Data[1]*256) / 10000;
    unsigned char post_hi = (hcan.pRxMsg->Data[2] + hcan.pRxMsg->Data[3]*256) / 10000;
    unsigned char run_lo = (hcan.pRxMsg->Data[4] + hcan.pRxMsg->Data[5]*256) / 10000;
    unsigned char run_hi = (hcan.pRxMsg->Data[6] + hcan.pRxMsg->Data[7]*256) / 10000;  
  }
  else if (hcan.pRxMsg->StdId == 0xAC){
    unsigned char cmd_torque = (hcan.pRxMsg->Data[0] + hcan.pRxMsg->Data[1]*256) / 10;
    unsigned char torque_feed = (hcan.pRxMsg->Data[2] + hcan.pRxMsg->Data[3]*256) / 10000;
    unsigned char power_time = (hcan.pRxMsg->Data[4] + hcan.pRxMsg->Data[5]*256 + hcan.pRxMsg->Data[6]*256*256 + hcan.pRxMsg->Data[7]*256*256*256) / 10;
  }  
  else if (hcan.pRxMsg->StdId == 0xAD){
    unsigned char mod_ind = (hcan.pRxMsg->Data[0] + hcan.pRxMsg->Data[1] * 256) / 10;
    unsigned char flux_weak = (hcan.pRxMsg->Data[2] + hcan.pRxMsg->Data[3] * 256) / 10;
    unsigned char id_cmd = (hcan.pRxMsg->Data[4] + hcan.pRxMsg->Data[5]*256) / 10;
    unsigned char iq_cmd = (hcan.pRxMsg->Data[6] + hcan.pRxMsg->Data[7]*256) / 10;
  }

	/* Re-enable Rx interrupt */
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
	
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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
