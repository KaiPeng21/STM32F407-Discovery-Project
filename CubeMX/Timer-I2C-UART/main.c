/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// User Defined Function Declarations
void Bluetooth_Send_Accelerometer(signed short int x, signed short int y, signed short int z);
void Bluetooth_Send(char * message, uint16_t length);

// Electrical Component Constants
uint16_t ADXL_DEV_ADDR = 0x53 << 1; // STM requires device address to be shifted by 1 bit
uint8_t ADXL_POWCTL = 0x2D;
uint8_t ADXL_DATAPTR = 0x32;

// Buffer Data Structure
uint8_t adxl_buffer[6];
signed short int x;
signed short int y;
signed short int z;

// bluetooth receive buffer
uint8_t buffrec[5];

// timer flags
uint8_t onems_ct;
uint8_t tenms_ct;
uint8_t hunms_ct;
uint8_t tenms;
uint8_t hunms;
uint8_t onesec;

// i2c transmit/ receive flag
uint8_t is_i2c_transmitted;

int i = 0;

// Timer Callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance == TIM6) {
		onems_ct++;
	  if (onems_ct >= 10){
		  onems_ct = 0;
		  tenms = 1;
		  tenms_ct++;
			if (tenms_ct >= 10){
				tenms_ct = 0;
				hunms = 1;
				hunms_ct++;
				if (hunms_ct >= 10){
					hunms_ct = 0;
					onesec = 1;
				}
			}
	  }
	}
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	onems_ct = 0;
	tenms_ct = 0;
  hunms_ct = 0;
  tenms = 0;
  hunms = 0;
  onesec = 0;
	
	is_i2c_transmitted = 0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM6_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
	
	
	// USART and Bluetooth Interrupt Initializations
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	
	// I2C and ADXL345 Accelerometer initializations 
	uint8_t i2c_data[2];
	i2c_data[0] = ADXL_POWCTL;
	i2c_data[1] = 0;
	HAL_I2C_Master_Transmit(&hi2c1, ADXL_DEV_ADDR, i2c_data, 2, 100);
	i2c_data[0] = ADXL_POWCTL;
	i2c_data[1] = 16;
	HAL_I2C_Master_Transmit(&hi2c1, ADXL_DEV_ADDR, i2c_data, 2, 100);
	i2c_data[0] = ADXL_POWCTL;
	i2c_data[1] = 8;
	HAL_I2C_Master_Transmit(&hi2c1, ADXL_DEV_ADDR, i2c_data, 2, 100);
	
	
	// Start Timer
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim6);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if (tenms){
			tenms = 0;
			
			if (!is_i2c_transmitted){
				HAL_UART_Receive(&huart1, buffrec, 5, 1);
				i2c_data[0] = ADXL_DATAPTR;
		    HAL_I2C_Master_Transmit(&hi2c1, ADXL_DEV_ADDR, i2c_data, 1, 100);
				is_i2c_transmitted = 1;
			}else{
				HAL_I2C_Master_Receive(&hi2c1, ADXL_DEV_ADDR, adxl_buffer, 6, 100);
				is_i2c_transmitted = 0;
				x = (((signed short int) adxl_buffer[1]) << 8) | adxl_buffer[0];
				y = (((signed short int) adxl_buffer[3]) << 8) | adxl_buffer[2];
				z = (((signed short int) adxl_buffer[5]) << 8) | adxl_buffer[4];
				Bluetooth_Send_Accelerometer(x, y, z);
			}
			
		}
		
		if (hunms){
		  hunms = 0;	
			
		}
		
		if (onesec){
			onesec = 0;
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		}
		
		/*
		i2c_data[0] = ADXL_DATAPTR;
		HAL_I2C_Master_Transmit(&hi2c1, ADXL_DEV_ADDR, i2c_data, 1, 100);
		HAL_Delay(20);
		HAL_I2C_Master_Receive(&hi2c1, ADXL_DEV_ADDR, adxl_buffer, 6, 100);
		
		x = (((signed short int) adxl_buffer[1]) << 8) | adxl_buffer[0];
		y = (((signed short int) adxl_buffer[3]) << 8) | adxl_buffer[2];
		z = (((signed short int) adxl_buffer[5]) << 8) | adxl_buffer[4];
		
		
		Bluetooth_Send_Accelerometer(x, y, z);
		HAL_Delay(20);
		HAL_UART_Receive(&huart1, buffrec, 5, 1);
		HAL_Delay(50);
		*/
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void Bluetooth_Send_Accelerometer(signed short int x, signed short int y, signed short int z)
{
	char * bluetooth_buff;
	uint8_t tmp;
	
	if (x >= 100){
		
		tmp = (x / 100) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (x / 10) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (x % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
	
	} else if (x >= 10){
		
		tmp = (x / 10) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (x % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	} else if (x >= 0){
		
		tmp = (x % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	} else if (x >= -9){
		
		bluetooth_buff = "-";
		HAL_UART_Transmit(&huart1, (uint8_t *)bluetooth_buff, 1, 20);
		tmp = (-x % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	} else if (x >= -99){
		
		bluetooth_buff = "-";
		HAL_UART_Transmit(&huart1, (uint8_t *)bluetooth_buff, 1, 20);
		tmp = (-x / 10) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (-x % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	} else{
		
		bluetooth_buff = "-";
		HAL_UART_Transmit(&huart1, (uint8_t *)bluetooth_buff, 1, 20);
		tmp = (-x / 100) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (-x / 10) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (-x % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	}
	
	bluetooth_buff = ",";
	HAL_UART_Transmit(&huart1, (uint8_t *)bluetooth_buff, 1, 20);
	
	if (y >= 100){
		
		tmp = (y / 100) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (y / 10) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (y % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
	
	} else if (y >= 10){
		
		tmp = (y / 10) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (y % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	} else if (y >= 0){
		
		tmp = (y % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	} else if (y >= -9){
		
		bluetooth_buff = "-";
		HAL_UART_Transmit(&huart1, (uint8_t *)bluetooth_buff, 1, 20);
		tmp = (-y % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	} else if (y >= -99){
		
		bluetooth_buff = "-";
		HAL_UART_Transmit(&huart1, (uint8_t *)bluetooth_buff, 1, 20);
		tmp = (-y / 10) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (-y % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	} else{
		
		bluetooth_buff = "-";
		HAL_UART_Transmit(&huart1, (uint8_t *)bluetooth_buff, 1, 20);
		tmp = (-y / 100) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (-y / 10) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (-y % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	}
	
	bluetooth_buff = ",";
	HAL_UART_Transmit(&huart1, (uint8_t *)bluetooth_buff, 1, 20);
	
	if (z >= 100){
		
		tmp = (z / 100) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (z / 10) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (z % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
	
	} else if (z >= 10){
		
		tmp = (z / 10) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (z % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	} else if (z >= 0){
		
		tmp = (z % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	} else if (z >= -9){
		
		bluetooth_buff = "-";
		HAL_UART_Transmit(&huart1, (uint8_t *)bluetooth_buff, 1, 20);
		tmp = (-z % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	} else if (z >= -99){
		
		bluetooth_buff = "-";
		HAL_UART_Transmit(&huart1, (uint8_t *)bluetooth_buff, 1, 20);
		tmp = (-z / 10) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (-z % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	} else{
		
		bluetooth_buff = "-";
		HAL_UART_Transmit(&huart1, (uint8_t *)bluetooth_buff, 1, 20);
		tmp = (-z / 100) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (-z / 10) % 10 + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		tmp = (-z % 10) + 48;
		HAL_UART_Transmit(&huart1, &tmp, 1, 20);
		
	}
	
	bluetooth_buff = "\n\r";
	HAL_UART_Transmit(&huart1, (uint8_t *)bluetooth_buff, 2, 20);
}

void Bluetooth_Send(char * message, uint16_t length)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)message, length, 20);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
