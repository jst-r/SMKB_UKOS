/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "math.h"
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint16_t status = 1;
volatile uint16_t resetLength = 0;
volatile uint16_t setLength = 0;
char buffer[30];
volatile uint16_t element_num = 0;
volatile uint16_t data = 0;
volatile uint16_t counter = 0;
static uint16_t depth = 80;
static uint16_t sample_rate = 10000;
volatile uint32_t raw_data[80] = {0}; //Increase array size if neccessary
static uint16_t time_reject = 600;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);

/* USER CODE BEGIN PFP */

void CheckData (void){
	uint32_t  time_sufficient = 0;
	/*calculate sum non-zero value elements*/
	for(int k =0; k< (depth-1);k++){
		if (raw_data[k] !=0  ){
			if(raw_data[k] > 0x20000){
				time_sufficient = time_sufficient + (raw_data[k] - 0x20000); 
				//HAL_UART_Transmit(&huart3,(uint8_t*)buffer, sprintf(buffer, "setLength = %d\n", (raw_data[k] - 0x20000)), 200);
			}else{
				time_sufficient = time_sufficient + raw_data[k];
				//HAL_UART_Transmit(&huart3,(uint8_t*)buffer, sprintf(buffer, "resetLength = %d\n", raw_data[k]), 200);
			}
		}
	}
	/* ***** */
	
	int time_sec = time_sufficient / sample_rate; 
	if (time_sec > 2){																//Is record time enough for reconstruction
		__NOP();																				//Insert your filtration here
	}
}


void ClearBuffer (void){
	element_num = 0;
	CheckData();
	for (int i = 0; i < (depth - 1); i++){
		raw_data[i] = 0;
	}
	setLength = 0;
	HAL_UART_Transmit_IT(&huart3,(uint8_t*)buffer, sprintf(buffer, "IDLE\n"));
}


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
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);
	HAL_TIM_Base_Start_IT(&htim15);
	HAL_TIM_Base_Start_IT(&htim16);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_TIM15
                              |RCC_PERIPHCLK_TIM16;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 71;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 99;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7199;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_3_3V_EN_GPIO_Port, GPIO_3_3V_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_14V_EN_GPIO_Port, GPIO_14V_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI1_END_STOP_Pin EXTI2_END_STOP_Pin */
  GPIO_InitStruct.Pin = EXTI1_END_STOP_Pin|EXTI2_END_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_3_3V_EN_Pin GPIO_14V_EN_Pin */
  GPIO_InitStruct.Pin = GPIO_3_3V_EN_Pin|GPIO_14V_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	
	if (htim->Instance == TIM16){
		
		if ((counter <= 3) & (counter >= 1)){
			HAL_UART_Transmit(&huart3, "Spinning motor forward\n", 23, 200);
//			HAL_UART_Transmit(&huart3, "\n", 1, 200);
		}	else if ((counter >= 4) & (counter != 0)){
			HAL_UART_Transmit(&huart3, "Spinning motor backward\n", 24, 200);
			}
		counter = 0;
		}
	
	
	
	//GPIO - SET - > hydrophone -> 0 -> osc -> 1;
	if (htim->Instance == TIM15) {
		/*
		if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2) == SET & status == 0)||(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == RESET & status == 1)){		
		//HAL_UART_Transmit(&huart3,(uint8_t*)buffer, sprintf(buffer, "resetLength = %d\n", resetLength), 1000);
		//	HAL_UART_Transmit(&huart3,(uint8_t*)buffer, sprintf(buffer, "resetLength = %d\n", setLength), 1000);
			//	if (setLength / resetLength > 10) {
		//			HAL_UART_Transmit(&huart3, "Motor control Start\n", 20 ,200);
		//		} else if (setLength / resetLength <10) {
		//			HAL_UART_Transmit(&huart3, "Motor control Stop\n", 20, 200);}
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == SET) {
					if (resetLength < 1000) {
						//HAL_UART_Transmit(&huart3, "Less than a 1000\n", 20, 200);
					if (counter == 0){
				//	TIM16->CCR1=0;
				//	HAL_UART_Transmit(&huart3, "Motor is ready\n", 15, 200);
					}
					counter ++;
						
					}	else {
						//HAL_UART_Transmit(&huart3, "More than a 1000\n",20, 200);
					//counter ++;
					}
					//HAL_UART_Transmit_IT(&huart3,(uint8_t*)buffer, sprintf(buffer, "resetLength = %d\n", resetLength)); //define the variable
					resetLength = 0;
					status = 1;
				} else {
					if (setLength < 500){
						resetLength = setLength + resetLength;
					} 
					//HAL_UART_Transmit_IT(&huart3,(uint8_t*)buffer, sprintf(buffer, "setLength = %d\n", setLength));
					setLength = 0;	
					status = 0;
					
				}
		} else {
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == SET ) {
				setLength ++;
			//	HAL_UART_Transmit(&huart3,(uint8_t*)buffer, sprintf(buffer, "setLength = %d\n", resetLength), 1000);
				if (setLength >= 10000) {
					setLength = 9999;
				}
				status = 1;
			} else {
				resetLength ++;
		//		HAL_UART_Transmit(&huart3,(uint8_t*)buffer, sprintf(buffer, "resetLength = %d\n", resetLength), 1000);
				status = 0;}
		}
		*/
		
		
		/*
		if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2) == SET & status == 0)||(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == RESET & status == 1)){ //If state has changed - do smth
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == SET){
				raw_data[element_num] = raw_data[element_num] + resetLength;
				//HAL_UART_Transmit(&huart3,(uint8_t*)buffer, sprintf(buffer, "resetLength = %d\n", resetLength), 200);
				//HAL_UART_Transmit(&huart3,(uint8_t*)buffer, sprintf(buffer, "resetLength = %d\n", raw_data[element_num]), 200);
				
				//Filter function call
				
				resetLength = 0;
				element_num ++;
			} else { 
				if (element_num > 0){
					if (setLength < time_reject){
						element_num --;
						raw_data[element_num] = raw_data[element_num] + setLength;
						resetLength = resetLength + setLength;
					} else {
					raw_data[element_num] = setLength | 0x20000;
					element_num ++;
					//HAL_UART_Transmit(&huart3,(uint8_t*)buffer, sprintf(buffer, "setLength = %d\n", setLength), 200);
					}
					setLength = 0;
				} 
			}
		} 
		*/
		
		if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2) == SET & status == 0)||(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == RESET & status == 1)){ //If state has changed - do smth
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == SET){
				HAL_UART_Transmit(&huart3,(uint8_t*)buffer, sprintf(buffer, "resetLength = %d\n", resetLength), 200);
				/*Filter function call*/	
				resetLength = 0;
			} else { 
				HAL_UART_Transmit(&huart3,(uint8_t*)buffer, sprintf(buffer, "setLength = %d\n", setLength), 200);
				/*Filter function call*/
				setLength = 0;
				} 
			}
		
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == SET){
				setLength ++;
				if (setLength > (sample_rate * 4)){
					setLength = sample_rate * 4;
					ClearBuffer();
				}
				status = 1;	
			} else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == RESET){
				resetLength ++;
				status = 0;
			}
		
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_1) {
		HAL_UART_Transmit(&huart3, "Stop Motor pos 1\n", 17, 200 );
  } 
	else if (GPIO_Pin == GPIO_PIN_2) {
		HAL_UART_Transmit(&huart3, "Stop Motor pos 2\n", 17, 200 );
	} 
	else {
	  __NOP();
	}
}

/*int WordDivider(X){
	uint16_t  first = X / 1000;
	return first;
}

	int	WordDivider2(X){
	uint16_t second = X % 1000;
	return second;
}
*/
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
