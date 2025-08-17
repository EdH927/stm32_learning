/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32c07blink.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

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
	// Enable PERIPH clks
	RCC_IOPENR |= GPIO_AEN;
	// Enable TIM2
	RCC_APBENR1 |= RCC_TIM2_EN;

	// Set TIM2 Params
	TIM2_PSC = 47999U;
	TIM2_ARR = TIM2_CNTRST;
	TIM2_CNT = 0U;
	TIM2_EGR |= TIM2_EGR_UG;

	// Enable the timer interrupt
	TIM2_DIER |= TIM2_DIER_EN;
	NVIC_EnableIRQ(TIM2_IRQn);

	// Start the counter
	TIM2_CR1 |= TIM_CR1_CEN;

	// Set PA0 as output
    GPIOA_MODE &=  ~(1U << 3);
    GPIOA_MODE |=   (1U << 2);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void TIM2_IRQHandler(void) {
	  if (TIM2_SR & TIM2_SR_UIF) {
		  GPIOA_ODR ^= LED_PIN;
		  TIM2_SR = ~TIM2_SR_UIF;
	  }
}
