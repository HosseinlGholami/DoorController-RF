
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
void send_command(int CMD)
	// RS, HD7, HD6 , HD5, HD4, LD7, LD6 , LD5, LD4
{
		//RS
		if((CMD & 0x100) == 0x100)
			{
				HAL_GPIO_WritePin(GPIOE, RS_Pin, GPIO_PIN_SET);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOE, RS_Pin, GPIO_PIN_RESET);
			}
			
		//High order D7
		if((CMD & 0x80) == 0x80)
			{
				HAL_GPIO_WritePin(GPIOD, D7_Pin, GPIO_PIN_SET);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOD, D7_Pin, GPIO_PIN_RESET);
			}
			
		//High order D6
		if((CMD & 0x40) == 0x40)
			{
				HAL_GPIO_WritePin(GPIOD, D6_Pin, GPIO_PIN_SET);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOD, D6_Pin, GPIO_PIN_RESET);
			}
			
		//High order D5
		if((CMD & 0x20) == 0x20)
			{
				HAL_GPIO_WritePin(GPIOE, D5_Pin, GPIO_PIN_SET);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOE, D5_Pin, GPIO_PIN_RESET);
			}
			
		//High order D4
		if((CMD & 0x10) == 0x10)
			{
				HAL_GPIO_WritePin(GPIOE, D4_Pin, GPIO_PIN_SET);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOE, D4_Pin, GPIO_PIN_RESET);
			}
		//Send Command by toggle EN pin	
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOE, E_Pin, GPIO_PIN_SET); //EN High
		HAL_Delay(2);
		HAL_GPIO_WritePin(GPIOE, E_Pin, GPIO_PIN_RESET); //EN Low
		HAL_Delay(10);
		
		//Turn off All GPIO Pins
		HAL_GPIO_WritePin(GPIOE, D5_Pin|D4_Pin|E_Pin|RS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, D7_Pin|D6_Pin, GPIO_PIN_RESET);
			
		

		//RS
		if((CMD & 0x100) == 0x100)
			{
				HAL_GPIO_WritePin(GPIOE, RS_Pin, GPIO_PIN_SET);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOE, RS_Pin, GPIO_PIN_RESET);
			}
			
		//Low order D7
		if((CMD & 0x08) == 0x08)
			{
				HAL_GPIO_WritePin(GPIOD, D7_Pin, GPIO_PIN_SET);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOD, D7_Pin, GPIO_PIN_RESET);
			}
			
		//High order D6
		if((CMD & 0x04) == 0x04)
			{
				HAL_GPIO_WritePin(GPIOD, D6_Pin, GPIO_PIN_SET);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOD, D6_Pin, GPIO_PIN_RESET);
			}
			
		//High order D5
		if((CMD & 0x02) == 0x02)
			{
				HAL_GPIO_WritePin(GPIOE, D5_Pin, GPIO_PIN_SET);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOE, D5_Pin, GPIO_PIN_RESET);
			}
			
		//High order D4
		if((CMD & 0x01) == 0x01)
			{
				HAL_GPIO_WritePin(GPIOE, D4_Pin, GPIO_PIN_SET);
			}
		else 
			{
				HAL_GPIO_WritePin(GPIOE, D4_Pin, GPIO_PIN_RESET);
			}
		//Send Command by toggle EN pin	
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOE, E_Pin, GPIO_PIN_SET); //EN High
		HAL_Delay(2);
		HAL_GPIO_WritePin(GPIOE, E_Pin, GPIO_PIN_RESET); //EN Low
		HAL_Delay(10);
		
		//Turn off All GPIO Pins
		HAL_GPIO_WritePin(GPIOE, D5_Pin|D4_Pin|E_Pin|RS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, D7_Pin|D6_Pin, GPIO_PIN_RESET);	
		
}


int main(void)
{
  /* USER CODE BEGIN 1 */

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
  /* USER CODE BEGIN 2 */
	
	
		HAL_GPIO_WritePin(GPIOE, D5_Pin|D4_Pin|E_Pin|RS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, D7_Pin|D6_Pin, GPIO_PIN_RESET);
		
		
		HAL_Delay(500);
		send_command(0x33);
		send_command(0x32);
		//Function set (set font and lines)
		send_command(0x28);
		
		//On off control
		send_command(0x0E);
		
		//Entry mode set
		send_command(0x06);
		
		//Clear LCD
		send_command(0x01);
		
		//Return Home
		send_command(0x02);
		
		
		//Write text to lcd
		send_command(0x148); //H
		send_command(0x145); //E
		send_command(0x14C); //L
		send_command(0x14C); //L
		send_command(0x14F); //O
		send_command(0x1FE); //Space
		send_command(0x157); //W
		send_command(0x14F); //O
		send_command(0x152); //R
		send_command(0x14C); //L
		send_command(0x144); //D
		
		
		

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
		
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D5_Pin|D4_Pin|E_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, D7_Pin|D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D5_Pin D4_Pin E_Pin RS_Pin */
  GPIO_InitStruct.Pin = D5_Pin|D4_Pin|E_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D7_Pin D6_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
