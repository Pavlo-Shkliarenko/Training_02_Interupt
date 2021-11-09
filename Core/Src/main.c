/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DELAY_VALUE 25
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern int8_t mode_blink,
              delay_step;

extern _Bool blink_on;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void firstBlinkMode(void);
void secondBlinkMode(void);
void thirdBlinkMode(void);
void fourthBlinkMode(void);
void fifthBlinkMode(void);
void pinSet(void);
void pinReset(void);

uint16_t pins[4] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(blink_on)
	  {
		  switch(mode_blink)
		  {
			  case 1:
			  {
				  firstBlinkMode();

				  break;
			  }

			  case 2:
			  {

				  secondBlinkMode();

				  break;
			  }

			  case 3:
			  {

				  thirdBlinkMode();

				  break;
			  }

			  case 4:
			  {

				  fourthBlinkMode();

				  break;
			  }

			  case 5:
			  {

				  fifthBlinkMode();

				  break;
			  }

			  default:
			  {
				  pinSet();

				  break;
			  }
		  }
	  }
	  else
	  {
		  pinReset();
	  }
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 PC9 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void firstBlinkMode(void)
{
	uint8_t second_led = 3,
		    first_led = 0;

    for(first_led = 0; first_led < 4; first_led++)
    {

	    HAL_GPIO_WritePin(GPIOD, pins[first_led] | pins[second_led], GPIO_PIN_SET);
	    HAL_Delay(delay_step * DELAY_VALUE);

	    HAL_GPIO_WritePin(GPIOD, pins[first_led] | pins[second_led], GPIO_PIN_RESET);

	    second_led = first_led;
    }
}

void secondBlinkMode(void)
{
	HAL_GPIO_WritePin(GPIOD, pins[0] | pins[1] | pins[2] | pins[3], GPIO_PIN_SET);

	for(uint8_t first_led = 0; first_led < 4; first_led++)
	{

		HAL_GPIO_WritePin(GPIOD, pins[first_led], GPIO_PIN_RESET);
		HAL_Delay(delay_step * DELAY_VALUE);

		HAL_GPIO_WritePin(GPIOD, pins[first_led], GPIO_PIN_SET);
	}
}

void thirdBlinkMode(void)
{
	  uint8_t  first_led = 0;

      for(first_led = 0; first_led < 4; first_led++)
      {

		  HAL_GPIO_WritePin(GPIOD, pins[first_led], GPIO_PIN_SET);
		  HAL_Delay(delay_step * DELAY_VALUE);

		  HAL_GPIO_WritePin(GPIOD, pins[first_led], GPIO_PIN_RESET);
	  }
}

void fourthBlinkMode(void)
{
	for(uint8_t first_led = 0, second_led = 2; first_led < 2; first_led++, second_led++)
	{
		HAL_GPIO_WritePin(GPIOD, pins[first_led] | pins[second_led], GPIO_PIN_SET);
    	HAL_Delay(delay_step * DELAY_VALUE);

	    HAL_GPIO_WritePin(GPIOD, pins[first_led] | pins[second_led], GPIO_PIN_RESET);
	}
}

void fifthBlinkMode(void)
{
	for(uint8_t first_led = 0, second_led = 1; first_led < 4; first_led += 2, second_led += 2)
	{

	    HAL_GPIO_WritePin(GPIOD, pins[first_led] | pins[second_led], GPIO_PIN_SET);
		HAL_Delay(delay_step * DELAY_VALUE);

		HAL_GPIO_WritePin(GPIOD, pins[first_led] | pins[second_led], GPIO_PIN_RESET);
	}
}

void pinSet(void)
{
	HAL_GPIO_WritePin(GPIOD, pins[0] | pins[1] | pins[2] | pins[3], GPIO_PIN_SET);
}

void pinReset(void)
{
	HAL_GPIO_WritePin(GPIOD, pins[0] | pins[1] | pins[2] | pins[3], GPIO_PIN_RESET);
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
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
