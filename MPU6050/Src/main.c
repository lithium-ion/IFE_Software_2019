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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "sd_hal_mpu6050.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SD_MPU6050 mpu1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
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
  SD_MPU6050_Result result ;
  uint8_t mpu_ok[3] = {"OK\n"};
  uint8_t mpu_not[4] = {"NOK\n"};
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
    HAL_Delay(100);
    if(result == SD_MPU6050_Result_Ok)
    {
//      HAL_UART_Transmit(&huart1,mpu_ok,(uint16_t)15, 1000);
    }
    else
    {
//      HAL_UART_Transmit(&huart1,mpu_not,(uint16_t)17, 1000);
    }
    SD_MPU6050_ReadTemperature(&hi2c1,&mpu1);
    float tempf = mpu1.Temperature;
    SD_MPU6050_ReadGyroscope(&hi2c1,&mpu1);
    int16_t g_x = mpu1.Gyroscope_X;
    int16_t g_y = mpu1.Gyroscope_Y;
    int16_t g_z = mpu1.Gyroscope_Z;

    SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
    int16_t a_x = mpu1.Accelerometer_X;
    int16_t a_y = mpu1.Accelerometer_Y;
    int16_t a_z = mpu1.Accelerometer_Z;
//    unsigned char data[16];
//    data[0] = (temp>>8) & 0x0FF;
//    data[1] = temp & 0x0FF;
//
//    data[2] = (g_x>>8) & 0x0FF;
//    data[3] = g_x & 0x0FF;
//    data[4] = (g_y>>8) & 0x0FF;
//    data[5] = g_y & 0x0FF;
//    data[6] = (g_z>>8) & 0x0FF;
//    data[7] = g_z & 0x0FF;
//
//    data[8] = (a_x>>8) & 0x0FF;
//    data[9] = a_x & 0x0FF;
//    data[10] = (a_y>>8) & 0x0FF;
//    data[11] = a_y & 0x0FF;
//    data[12] = (a_z>>8) & 0x0FF;
//    data[13] = a_z & 0x0FF;
//
//    data[14] = 255;
//    data[15] = 255;

//    HAL_UART_Transmit(&huart1, data, sizeof(data), 1000);

    float fg_x = ((float)g_x)/16384.0;
    float fg_y = ((float)g_y)/16384.0;
    float fg_z = ((float)g_z)/16384.0;
    float fa_x = ((float)a_x)/16384.0;
    float fa_y = ((float)a_y)/16384.0;
    float fa_z = ((float)a_z)/16384.0;

    char *stemp = (tempf < 0) ? "-" : "+";
    char *sg_x = (fg_x < 0) ? "-" : "+";
    char *sg_y = (fg_y < 0) ? "-" : "+";
    char *sg_z = (fg_z < 0) ? "-" : "+";
    char *sa_x = (fa_x < 0) ? "-" : "+";
    char *sa_y = (fa_y < 0) ? "-" : "+";
    char *sa_z = (fa_z < 0) ? "-" : "+";

    tempf = (tempf < 0) ? -tempf : tempf;
    fg_x = (fg_x < 0) ? -fg_x : fg_x;
    fg_y = (fg_y < 0) ? -fg_y : fg_y;
    fg_z = (fg_z < 0) ? -fg_z : fg_z;
    fa_x = (fa_x < 0) ? -fa_x : fa_x;
    fa_y = (fa_y < 0) ? -fa_y : fa_y;
    fa_z = (fa_z < 0) ? -fa_z : fa_z;

    int itemp = tempf;
    int ig_x = fg_x;
    int ig_y = fg_y;
    int ig_z = fg_z;
    int ia_x = fa_x;
    int ia_y = fa_y;
    int ia_z = fa_z;

    int dtemp = tempf*1000;
    int dg_x = fg_x*1000;
    int dg_y = fg_y*1000;
    int dg_z = fg_z*1000;
    int da_x = fa_x*1000;
    int da_y = fa_y*1000;
    int da_z = fa_z*1000;

    char str[1000];
    sprintf(str, "Temp: %s%d.%03d ||| Gyro: %s%d.%03d, %s%d.%03d, %s%d.%03d ||| Accel: %s%d.%03d, %s%d.%03d, %s%d.%03d \n", stemp, itemp, dtemp, sg_x, ig_x, dg_x, sg_y, ig_y, dg_y, sg_z, ig_z, dg_z, sa_x, ia_x, da_x, sa_y, ia_y, da_y, sa_z, ia_z, da_z);
    int i = 0;
    for(; i < sizeof(str); i++){
        if(str[i] == '\n') break;
    }
    HAL_UART_Transmit (&huart1, (uint8_t *)str, i+2, 1000);


//  float fg_x = ((float)g_x)/16384.0;
//  float fg_y = ((float)g_y)/16384.0;
//  float fg_z = ((float)g_z)/16384.0;
//
//  float fa_x = ((float)a_x)/16384.0;
//  float fa_y = ((float)a_y)/16384.0;
//  float fa_z = ((float)a_z)/16384.0;
//
//  char str[200];
//  sprintf(str, "Temp: %2.2f ||| Gyro: %2.2f, %2.2f, %2.2f ||| Accel: %2.2f, %2.2f, %2.2f \n", tempf, fg_x, fg_y, fg_z, fa_x, fa_y, fa_z);
//  HAL_UART_Transmit (&huart1, (uint8_t *)str, sizeof(str), 1000);
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.Mode = UART_MODE_TX_RX;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
