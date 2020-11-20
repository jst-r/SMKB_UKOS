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
#include "string.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define IDLE   0
#define DONE   1
#define F_CLK  72000000UL
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Common variables init
int T_Pulse = 0;
int State_Number=0;
int State_Sequence[20] = {0};
uint8_t DecimalArr[35] = {'\0'};
uint8_t FreqArr[10] = {'\0'};
volatile char decimal = 0;
volatile uint8_t gu8_State = IDLE;
volatile char NESS_FQ = 0;
volatile char NESS_DR = 0;
volatile char NESS_SQ = 1;
volatile char NESS_OFF = 0;
volatile uint8_t gu8_MSG[35] = {'\0'};
volatile uint32_t gu32_T1 = 0;
volatile uint32_t gu32_T2 = 0;
volatile uint32_t gu32_Ticks = 0;
volatile uint16_t gu16_TIM2_OVC = 0;
volatile uint32_t gu32_Freq = 0;
volatile uint16_t ClosedTime;
volatile uint16_t ChangeState;
volatile uint16_t a = 0;
volatile uint16_t b = 0;

//ADC variables init
//0. ADC_State status flag
char ADC_State = 0;
//1. ADC value
volatile uint16_t ADCVal;
//2.Voltage 
volatile double Voltage;
//3. VDDA
volatile double VDDA = 3.316; // Set the VDDA value here.
//4. Temperature
volatile uint16_t TemperatureEdge;
//5.Divider 
double base = 2;
double power = 12;
double T_div = 0.005;
double div;
volatile uint16_t Temperature;   // Measured Temperature Value

//RTC variables init
//0. RTC status flag
char RTC_State = 0;
//1.RTC global counter
volatile uint16_t counter;
volatile uint16_t der_count;
volatile uint16_t off_count;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
//Following: LED interface; indicate current algorithm's code Number. later algorithm's code Number = int decimal
void LED_I_O (int exam){
		int word[6];
		for(int num = 0; num < 6; num++){
			word[num] = (exam >> num) & 00000001;
		}
		if (word[0] == 0){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		}
		if (word[0] == 1) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		if (word[1] == 0) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		}
		if (word[1] == 1) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		if (word[2] == 0) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		}
		if (word[2] == 1) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		}
		if (word[3] == 0) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		}
		if (word[3] == 1) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		}
		if (word[4] == 0) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		}
		if (word[4] == 1) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		}
		if (word[5] == 0) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		}
		if (word[5] == 1) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		}
	}



//Following: decoder of "raw input data" from Data Recieve 
void SequenceAnalyze(int State_Sequence[]){
		decimal = 0;
		int bin[20] = {0};
		int bin_number = 0;
		int count_num = 0;
		for(int num = 0; num < 20; num++){
			if (State_Sequence[num]!= 0){
			count_num++;
			}
		}
		for (int i = 0; i < count_num; i++){
			if ((i % 2) == 0){
				int n0 = (State_Sequence[i] + (State_Sequence[0] + 1) / 2)/State_Sequence[0];
				for (int k = 0; k < n0; k++){
					bin[bin_number] = 0;				
					bin_number++;
				}
			} else {
			
			if ((i % 2) == 1){
				int n1 = (State_Sequence[i] + (State_Sequence[0] + 1) / 2)/State_Sequence[0];
				for (int k = 0; k < n1; k++){
					bin[bin_number] = 1;
					bin_number++;	
					}
				}
			}	
		}

		for(int z = 2; z <= 7; z++){
			decimal=decimal + bin[z] * pow(2,(z - 2));
		}
		LED_I_O(decimal);
		for(int f = 0; f < 20; f++){
			bin[f] = 0;
			State_Sequence[f] = 0;
			}
		State_Number = 0;
}

//Following: function of reading information, transmitted via state sequence of ON and OFF 
void Data_Recieve (){
	if (NESS_DR == 1){
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == SET){};
		TIM4->CNT= 0;
			loop1:
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == RESET){}
		T_Pulse=TIM4->CNT;
		State_Sequence[State_Number]=T_Pulse;
		State_Number++;
		TIM4->CNT= 0;
			loop2:
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == SET){
			
			if((TIM4->CNT)>(State_Sequence[0]*7)){
				T_Pulse=State_Sequence[0]*7;
				State_Sequence[State_Number]=T_Pulse;	
				SequenceAnalyze(State_Sequence);	
				NESS_DR=0;
			} else { goto loop2;}
			
		} else {
			T_Pulse=TIM4->CNT;
			State_Sequence[State_Number]=T_Pulse;
			State_Number++;
			TIM4->CNT= 0;
			goto loop1;
		}	
	}
}

/*Following: RTC Action function declaration
	RTC_Action (ClosedTime, ChangeState )
	ClosedTime (0-65355) time of valve closed state   (seconds)
	ChangeState (0-65355) - time of valve state changing (seconds)
*/
void RTC_Action(ClosedTime, ChangeState){     
if (RTC_State == 1){
	 if (counter > ChangeState){
		 counter = 0;
	 RTC_State = 0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);	 //open valve, visit black mesa
	}
}

else{
	if (counter > ChangeState + ClosedTime){
		counter = 0;
		RTC_State = 1;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // reset valve
		}
	}
}
/*Following: ADC Action function declaration
	ADC_Action (TemperatureEdge, ChangeState)
	TemperatureEdge (0-65355) - valve triggering temperature value (Celsius)
	ChangeState (0-65355) - time of valve state changing (seconds)

*/
void ADC_Action(TemperatureEdge, ChangeState){ 
	HAL_ADC_Stop_DMA(&hadc1);
	div = (pow(base,power)); 
	Voltage = ((ADCVal / div) * VDDA);
	Temperature = Voltage / T_div;	
 if (Temperature > TemperatureEdge){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Open valve, release black mesa;
	ADC_State = 1;
	 if (counter == ChangeState){  //
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);	 // Close valve
		ADC_State = 0;
		counter = 0;
	 }
 }
 else if (Temperature < TemperatureEdge){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);	 // Close valve
	ADC_State = 0;
 }
}

	

void Derivative(void){
if (gu32_Freq < 390){
	int a = gu32_Freq;
	 int i = 0;
	 for (der_count; der_count < 10;){
		 int b = a;
		 int a = gu32_Freq;
		 if (b == a) {
				for (der_count; der_count < 10;){
					FreqArr[i] = gu32_Freq;
					i++;}
				i = 0;
				der_count = 0;
				}
			}
		}
	}

int Abs(X,Y){
 if ((X - Y) < 0) {
		return (X - Y) * (-1);}
 else{
	 return (X - Y);
	}
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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim4); //"timer for time_of_state identification via Data Recieve"
	HAL_RTCEx_SetSecond_IT(&hrtc);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		NESS_FQ = 2;
		HAL_Delay(400); //NB somehow improve this part
		if (gu32_Freq > 390){
			NESS_SQ = 1;
		der_count = 0;}
		if (gu32_Freq <= 390 && gu32_Freq > 200){
			NESS_FQ = 0;
			NESS_DR = 1;
			ADC_State = 1;
			RTC_State = 1;
			//Derivative();
			//Data_Recieve();
			HAL_Delay(400);
		} else {
		LED_I_O(0);
		HAL_Delay(400);
		if (decimal > 0){
			if (decimal < 11) {
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCVal,1);
		} 
		else {
				HAL_RTCEx_SetSecond_IT(&hrtc);
				} 
			}	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_3;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, Valv3_Pin|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : OFF_PIN_Pin */
  GPIO_InitStruct.Pin = OFF_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(OFF_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Valv3_Pin PB10 PB12 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = Valv3_Pin|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
//Following: UART transmit data function callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	if(NESS_FQ > 0){
		TIM2->CCR1=0;
		if(gu8_State == IDLE)
		{
			gu32_T1 = TIM2->CCR1;
			gu16_TIM2_OVC = 0;
			gu8_State = DONE;
		}
		else if(gu8_State == DONE)
		{
			gu32_T2 = TIM2->CCR1;
			gu32_Ticks = gu32_T2 - gu32_T1;
			gu32_Freq = (uint32_t)(F_CLK/(gu32_Ticks * 720));
			gu8_State = IDLE;
		}
	NESS_FQ--;
} else {
}	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim1)
{
	gu16_TIM2_OVC++;
}

void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc){
 // RTC interrupt handler           (
	counter++;
	der_count++;
	if (gu32_Freq < 390 && NESS_SQ == 1 && gu32_Freq >= 259){
		a = gu32_Freq;
		if (Abs(a,b) <= 1) {
			if (der_count == 8) {
				decimal = ceil((a - 260) / 5) ;
				der_count = 0;
				NESS_SQ = 0;}
			else{ 
				b = a;}}
		else{
		b = a;
		der_count = 0;
		}
	}	
	
	if (NESS_OFF == 1){
		off_count++;
		if (off_count >= 120){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); 
			if (off_count >= 150){
				//NB, modify by addin gpio B10 (power supply control)
			}
			NESS_OFF = 0;
			off_count = 0;
	}
}
	 //Sorting by split
	if (decimal <= 12){
		if (decimal <= 6){
			if (decimal == 1){        //1T
			ADC_Action(80, 30); // ADC_Action(TemperatureEdge, ChangeState)
			}
			else if (decimal == 2){  //2T
			ADC_Action(90, 30);
			}
			else if (decimal == 3) {   //3T
			ADC_Action(100, 30);
			}
			else if (decimal == 4) {   //4T
			ADC_Action(110, 30);   
			}
			else if (decimal == 5) {  //5T
			ADC_Action(120, 30);   
			}
		else if (decimal == 6) {   //6T
		ADC_Action(130, 30);   
		}
	}
		else if (decimal == 7) {  //7T
		ADC_Action(140, 30);   
		}
		else if (decimal == 8) {   //8T
		ADC_Action(150, 30);   
		}
		else if (decimal == 9) {   //9T
		ADC_Action(160, 30);   
		}
		else if (decimal == 10) {  //10T
		ADC_Action(170, 30);   }
		
		if (decimal == 11) {   //1B
	RTC_Action(50, 30); // RTC_Action(ClosedTime, ChangeState)
	}
	else if (decimal == 12) {   //2B
	RTC_Action(60, 30);   
	}
}
	else if (decimal <= 18)
	{
		if (decimal == 13) {   //3B
	RTC_Action(70, 30);   
	}
	else if (decimal == 14) {   //4B
	RTC_Action(80, 30);  
	}
	else if (decimal == 15) {   //5B
	RTC_Action(90, 30);   
	}

	else if (decimal == 16) {   //6B
	RTC_Action(100, 30);  
	}
	else if (decimal == 17) {   //7B
	RTC_Action(110, 30);   
	}
	else if (decimal == 18) {   //8B
	RTC_Action(120, 30);   
	}
}
	else if (decimal == 19) {   //9B
	RTC_Action(130, 30);   
	}
	else if (decimal == 20) {   //10B
	RTC_Action(140, 30);   
	}
	else if (decimal == 21) {   //11B
	RTC_Action(150, 30);   
	}
	else if (decimal == 22) {   //12B
	RTC_Action(160, 30);  
	}
	else if (decimal == 23) {   //13B
	RTC_Action(170, 30);   
	}
	else if (decimal == 24) {   //14B
	RTC_Action(180, 30);
		
	}
}

 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
	 UNUSED(GPIO_Pin);
	 if (GPIO_Pin == OFF_PIN_Pin)
	 {
	if (NESS_OFF == 1){	 
	 NESS_OFF = 0;
		off_count = 0;}
	else {
	NESS_OFF = 1;}
	 }
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
