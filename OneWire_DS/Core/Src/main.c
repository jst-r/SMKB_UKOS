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
#define DS28E18_PORT GPIOC
#define DS28E18_PIN GPIO_PIN_1
#define PullUp_Pin GPIO_PIN_2
	
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, DS_1, DS_2, DS_3, DS_4, ST_1_0, ST_1_1;
uint8_t DS_5,DS_6,DS_7,DS_8,DS_9,DS_10,DS_11,DS_12,DS_13,DS_14,DS_15,DS_16, DS_17, DS_18;
uint16_t SUM, RH, TEMP, step_3_0, step_3_1;	
uint8_t Presence = 0;
uint16_t Response = 0;

float temperature = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/*  delay in microseconds */
void delay(uint16_t time) 
	{
	__HAL_TIM_SET_COUNTER(&htim6,0);
	while((__HAL_TIM_GET_COUNTER(&htim6))<time);
	}
//--------------------------------------------

	
	
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
	{
		 GPIO_InitTypeDef GPIO_InitStruct = {0};
		 GPIO_InitStruct.Pin = GPIO_Pin;
		 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		 HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
	}
	
	
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
	{
		 GPIO_InitTypeDef GPIO_InitStruct = {0};
		 GPIO_InitStruct.Pin = GPIO_Pin;
		 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		 GPIO_InitStruct.Pull = GPIO_NOPULL;
		 HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
	}
	
uint8_t DS28E18_Start(void)
	{
		 
		 Set_Pin_Output(DS28E18_PORT,DS28E18_PIN);//set the pin as output
		 HAL_GPIO_WritePin(DS28E18_PORT, DS28E18_PIN, GPIO_PIN_RESET); // pull the pin low
		 delay(500);
		 Set_Pin_Input(DS28E18_PORT, DS28E18_PIN);
		 delay(65);
		 if((HAL_GPIO_ReadPin(DS28E18_PORT, DS28E18_PIN)== GPIO_PIN_RESET)) 
			 {
			 Response = 1;
			 }
		 else Response = -1;
		 delay(400);
		 return Response;
	}

void Write(uint8_t data)
	{
		Set_Pin_Output(DS28E18_PORT, DS28E18_PIN);
		
		for(int i=0; i<8; i++)
		{
			if((data &(1<<i))!=0) //if the bit is high
			{
				//write 1
				Set_Pin_Output(DS28E18_PORT, DS28E18_PIN); //set pn as output
				HAL_GPIO_WritePin(DS28E18_PORT, DS28E18_PIN,GPIO_PIN_RESET);
				delay(1);
				Set_Pin_Input(DS28E18_PORT, DS28E18_PIN);
				delay(60); 				
			}	
			else //if the bus bit is low
			{
				// write 0
				Set_Pin_Output(DS28E18_PORT, DS28E18_PIN);
				HAL_GPIO_WritePin(DS28E18_PORT, DS28E18_PIN, GPIO_PIN_RESET);
				delay(60);
				
				Set_Pin_Input(DS28E18_PORT, DS28E18_PIN);
				//delay(100);
			}
		}
	}
	
uint8_t Read(void)
{
	uint8_t value = 0;
	
	Set_Pin_Input(DS28E18_PORT, DS28E18_PIN); //set as input
	
	for(int i=0;i<8;i++)
	{
		Set_Pin_Output(DS28E18_PORT, DS28E18_PIN);// set as output
		HAL_GPIO_WritePin(DS28E18_PORT, DS28E18_PIN, GPIO_PIN_RESET);
		delay(2);
		Set_Pin_Input(DS28E18_PORT, DS28E18_PIN); // set as input
		//delay(5);
		if(HAL_GPIO_ReadPin(DS28E18_PORT, DS28E18_PIN)) // if GPIO Pin is HIGH
		{
			value |= 1<<i; // read = 1
		}
		 delay(60); //wait for 60 seconds
	}
	return value;
}
			
uint16_t Step_1(void)
{ 	
	Presence = DS28E18_Start();
	//HAL_Delay(1);
	Write(0xCC); //skip ROM
	Write(0x66);
	Write(0x05);
	Write(0x83);
	Write(0x0B);
	Write(0x03);
	Write(0xA5);
	Write(0x0F);
	ST_1_0 = Read();
	ST_1_1 = Read();
	Write(0xAA);
	//delay(1);
	return Presence;
}
//void Step_2(void)
	
void Step_3(void)
{
	Presence = DS28E18_Start();
	delay(1);
	Write(0xCC); //Match ROM
	/*Write(0x56); // 0 byte
	Write(0x70); // 1 byte
	Write(0x8E); // 2 byte
	Write(0x00); // 3 byte
	Write(0x00); // 4 byte
	Write(0x00); // 5 byte
	Write(0x43); // 6 byte*/
	Write(0x66); // 7 byte
	Write(0x05); // num of bytes 
	Write(0x83); // Write GPIO Config
	Write(0x0B);
	Write(0x03);
	Write(0xA5);
	Write(0x0F);
	step_3_0 = Read();
	step_3_1 = Read();
	Write(0xAA); // Release byte
	delay(1);
}	

void set2SPI (void)
{
	DS28E18_Start();
	Write(0xCC);
	Write(0x66);
	Write(0x02);
	Write(0x55);
	Write(0x38);
	DS_11 = Read();
	DS_12 = Read();
	if(DS_11 == 0x7E && DS_12 == 0x35)
	{
		Write(0xAA);
	}
}
	
void Check(void)
{
	DS_1 = Read();
	DS_2 = Read();
	DS_3 = Read();
	DS_4 = Read();
	DS_5 = Read();
}

void Check1(void)
{
	DS_6 = Read();
	DS_7 = Read();
	DS_8 = Read();
	DS_9 = Read();
	DS_10 = Read();
}

void Check2(void)
{
	DS_13 = Read();
	DS_14 = Read();
	DS_15 = Read();
	DS_16 = Read();
	DS_17 = Read();
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* Test at DS18B20 completed
	Presence = DS28E18_Start();
	HAL_Delay(1);
	Write(0xCC); //skip ROM
	Write(0x44);
	HAL_Delay(800);
			
	Presence = DS28E18_Start();
	HAL_Delay(1);
	Write(0xCC);
	Write(0xBE);
	
	Temp_byte1 = Read();
	Temp_byte2 = Read();
	TEMP = (Temp_byte2<<8)|Temp_byte1;
	temperature = (float)TEMP/16;
	
		
	HAL_Delay(3000);             */
  
	

	
	
	Step_1();
	delay(1000);
	Check();
	
	Step_3();
	delay(1000);
	Check1();
	
	set2SPI();
	delay(1000);
	Check2();
	
	HAL_Delay(3000);
	

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 60-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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
