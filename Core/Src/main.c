/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "abuzz.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "transmit.h"
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

enum traffic_light_state currentState;

enum traffic_light_state {
	s_CarsRed_PedestrianGreen = 0b10001,
	s_CarsRedYellow_PedestrianRed = 0b11010,
	s_CarsYellow_PedestrianRed = 0b01010,
	s_CarsGreen_PedestrianRed = 0b00110,
	s_CarsRed_PedestrianRed = 0b10010,
	s_init = 0b11111
};

enum event {
	ev_none,
	ev_button_push,
	ev_state_timeout,
	ev_return_state,
	ev_error = -99
};

#define EVQ_SIZE 10
enum event evq[ EVQ_SIZE ];
int evq_count = 0;
int evq_front_ix = 0;
int evq_rear_ix = 0;

void evq_init(){
	for(int i = 0; i < EVQ_SIZE; i++)
		evq[i] = ev_error;
}

bool evq_is_full(){
	return evq_count == EVQ_SIZE;
}

bool evq_is_empty(){
	return evq_count == 0;
}

void evq_push_back(enum event e){
	if (!evq_is_full()) {
		evq[evq_rear_ix] = e;
		evq_rear_ix = (evq_rear_ix + 1) % EVQ_SIZE;
		evq_count++;
	} else {
		printf("Queue full");
	}
}

enum event evq_pop_front(){
	if (!evq_is_empty()) {
		enum event e = evq[evq_front_ix];
		evq[evq_front_ix] = ev_error;
		evq_front_ix = (evq_front_ix + 1) % EVQ_SIZE;
		evq_count--;
		return e;
	} else {
		return ev_none;
	}
}

void reset_traffic_lights(){
	HAL_GPIO_WritePin(CARS_RED_GPIO_Port, CARS_RED_Pin, RESET);
	HAL_GPIO_WritePin(CARS_YELLOW_GPIO_Port, CARS_YELLOW_Pin, RESET);
	HAL_GPIO_WritePin(CARS_GREEN_GPIO_Port, CARS_GREEN_Pin, RESET);
	HAL_GPIO_WritePin(PED_RED_GPIO_Port, PED_RED_Pin, RESET);
	HAL_GPIO_WritePin(PED_GREEN_GPIO_Port, PED_GREEN_Pin, RESET);
}

void set_traffic_lights(enum traffic_light_state state) {
	switch (state) {
		case s_CarsRed_PedestrianRed:
			reset_traffic_lights();
			currentState = s_CarsRed_PedestrianRed;
			HAL_GPIO_WritePin(CARS_RED_GPIO_Port, CARS_RED_Pin, SET);
			HAL_GPIO_WritePin(PED_RED_GPIO_Port, PED_RED_Pin, SET);
			break;
		case s_CarsRedYellow_PedestrianRed:
			reset_traffic_lights();
			currentState = s_CarsRedYellow_PedestrianRed;
			HAL_GPIO_WritePin(CARS_RED_GPIO_Port, CARS_RED_Pin, SET);
			HAL_GPIO_WritePin(CARS_YELLOW_GPIO_Port, CARS_YELLOW_Pin, SET);
			HAL_GPIO_WritePin(PED_RED_GPIO_Port, PED_RED_Pin, SET);
			break;
		case s_CarsYellow_PedestrianRed:
			reset_traffic_lights();
			currentState = s_CarsYellow_PedestrianRed;
			HAL_GPIO_WritePin(CARS_YELLOW_GPIO_Port, CARS_YELLOW_Pin, SET);
			HAL_GPIO_WritePin(PED_RED_GPIO_Port, PED_RED_Pin, SET);
			break;
		case s_CarsRed_PedestrianGreen:
			reset_traffic_lights();
			currentState = s_CarsRed_PedestrianGreen;
			HAL_GPIO_WritePin(CARS_RED_GPIO_Port, CARS_RED_Pin, SET);
			HAL_GPIO_WritePin(PED_GREEN_GPIO_Port, PED_GREEN_Pin, SET);
			break;
		case s_CarsGreen_PedestrianRed:
			reset_traffic_lights();
			currentState = s_CarsGreen_PedestrianRed;
			HAL_GPIO_WritePin(CARS_GREEN_GPIO_Port, CARS_GREEN_Pin, SET);
			HAL_GPIO_WritePin(PED_RED_GPIO_Port, PED_RED_Pin, SET);
			break;
		case s_init:
			reset_traffic_lights();
			currentState = s_init;
			HAL_GPIO_WritePin(CARS_RED_GPIO_Port, CARS_RED_Pin, SET);
			HAL_GPIO_WritePin(CARS_YELLOW_GPIO_Port, CARS_YELLOW_Pin, SET);
			HAL_GPIO_WritePin(CARS_GREEN_GPIO_Port, CARS_GREEN_Pin, SET);
			HAL_GPIO_WritePin(PED_RED_GPIO_Port, PED_RED_Pin, SET);
			HAL_GPIO_WritePin(PED_GREEN_GPIO_Port, PED_GREEN_Pin, SET);
			break;
	}
}

void buzz(){
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, SET);
	abuzz_start();
	HAL_Delay(500);
	abuzz_p_long();
	HAL_Delay(500);
	abuzz_stop();
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, RESET);
}

void toggleIndicator(){
	HAL_GPIO_TogglePin(INDIC_GPIO_Port, INDIC_Pin);
	buzz();
}

void test_circular_queue() {
    Transmit(&huart1, "Initialized queue:\n");
    evq_init();
    Transmit(&huart1, "<print_evq>");

    Transmit(&huart1, "Pushing events 1 to 5:\n");
    for (int i = 1; i <= 5; i++) {
        evq_push_back(i);
    }
    Transmit(&huart1, "<print_evq>");

    Transmit(&huart1, "Popping 3 events:\n");
    char buffer[32];
    for (int i = 0; i < 3; i++) {
        sprintf(buffer, "Popped: %d\n", evq_pop_front());
        Transmit(&huart1, buffer);
    }
    Transmit(&huart1, "<print_evq>");

    Transmit(&huart1, "Pushing events 6 to 10:\n");
    for (int i = 6; i <= 10; i++) {
        evq_push_back(i);
    }
    Transmit(&huart1, "<print_evq>");

    Transmit(&huart1, "Popping 5 events:\n");
    for (int i = 0; i < 5; i++) {
        sprintf(buffer, "Popped: %d\n", evq_pop_front());
        Transmit(&huart1, buffer);
    }
    Transmit(&huart1, "<print_evq>");

    Transmit(&huart1, "Pushing events 11 to 12:\n");
    for (int i = 11; i <= 12; i++) {
        evq_push_back(i);
    }
    Transmit(&huart1, "<print_evq>");
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  evq_init();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    //test_circular_queue();
	set_traffic_lights(s_init);
	enum event currentEvent;
	int ticks_left_in_state = 0;

	while (1)
	{
	    uint32_t buttonPressed = GPIOB->IDR & BTN_Pin;

	    if (buttonPressed && (currentState == s_init || currentState == s_CarsGreen_PedestrianRed))
	        currentEvent = ev_button_push;

			if (ticks_left_in_state == 0){
				switch (currentState){
					case s_CarsGreen_PedestrianRed:
						if (currentEvent == ev_button_push){
							toggleIndicator();
							HAL_Delay(2000);
							set_traffic_lights(s_CarsYellow_PedestrianRed);
							ticks_left_in_state = 25;
							currentEvent = ev_none;
						}
						break;
					case s_CarsYellow_PedestrianRed:
						set_traffic_lights(s_CarsRed_PedestrianRed);
						ticks_left_in_state = 15;
						break;
					case s_CarsRed_PedestrianRed:
						if(currentEvent == ev_none) {
							set_traffic_lights(s_CarsRed_PedestrianGreen);
							ticks_left_in_state = 50;
						} else if (currentEvent == ev_return_state){
							set_traffic_lights(s_CarsRedYellow_PedestrianRed);
							toggleIndicator();
							ticks_left_in_state = 20;
						}
						break;
					case s_CarsRedYellow_PedestrianRed:
						set_traffic_lights(s_CarsGreen_PedestrianRed);
						ticks_left_in_state = 15;
						break;
					case s_CarsRed_PedestrianGreen:
						set_traffic_lights(s_CarsRed_PedestrianRed);
						ticks_left_in_state = 10;
						currentEvent = ev_return_state;
						break;
					case s_init:
						if (currentEvent == ev_button_push){
							currentEvent = ev_none;
							set_traffic_lights(s_CarsGreen_PedestrianRed);
							ticks_left_in_state = 20;
						}
						break;
				}
			} else {
				ticks_left_in_state--;
			}
			HAL_Delay(100);

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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CARS_RED_Pin|CARS_YELLOW_Pin|CARS_GREEN_Pin|PED_RED_Pin
                          |PED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, INDIC_Pin|LD2_Pin|LD3_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CARS_RED_Pin CARS_YELLOW_Pin CARS_GREEN_Pin PED_RED_Pin
                           PED_GREEN_Pin */
  GPIO_InitStruct.Pin = CARS_RED_Pin|CARS_YELLOW_Pin|CARS_GREEN_Pin|PED_RED_Pin
                          |PED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZ_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INDIC_Pin LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = INDIC_Pin|LD2_Pin|LD3_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
