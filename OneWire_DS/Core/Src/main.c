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
uint8_t SR_0, SR_1, SR_2, SR_1, SR_3, SR_4, SR_5, SR_6, SR_7, SR_8, SR_9, SR_10, SR_11, SR_12, SR_13;
uint16_t SUM, RH, TEMP, step_3_0, step_3_1, i, j, Run_0, Run_1;
/*Read sequencer response handler*/
uint16_t byte_0, byte_1, byte_2, byte_3, byte_4, byte_5, byte_6, byte_7, byte_7, byte_8, byte_9, byte_10;
uint16_t byte_11, byte_12, byte_13, byte_14, byte_15, byte_16, byte_17,byte_18, byte_19, byte_20, byte_21;
/*END*/
/*Run Sequencer response handler*/
uint16_t rs_0, rs_1, rs_2, rs_3, rs_4, rs_5, rs_6, rs_7, rs_8, rs_9, rs_10, rs_11, rs_12; 
/*Result handler*/
uint16_t res_0, res_1, res_2, res_3, res_4, res_5, res_6, res_7, res_8, res_9, res_10, res_11, res_12, res_13;
uint16_t res_14, res_15, res_16, res_17;
/*POR*/
uint16_t por_0, por_1, por_2, por_3, por_4, por_5, por_6, por_7, por_8, por_9, por_10; 
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
	
uint8_t Start(void)
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
		 delay(60); //wait for 60 microseconds
	}
	return value;
}
			
uint16_t Step_1(void)
{ 	
	Presence = Start();
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
	Presence = Start();
	delay(1);
	Write(0xCC); //Match ROM
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
	delay(1000);
}	

 void Step_4(void){
	Start();
	Write(0x55); //0 bit
	Write(0x56); // 1 bit
	Write(0x70); // 2 bit
	Write(0x8E);
	Write(0x00);
	Write(0x00);
	Write(0x00);
	Write(0x00);
	Write(0x43);
	Write(0x66); 	//
	Write(0x01);
	Write(0x7A);
	Read();
	Read();
	Write(0xAA);
 }

void set2SPI (void)
{
	Start();
	Write(0xCC);
	Write(0x66);
	Write(0x02);
	Write(0x55);
	Write(0x38);// least HEX char: 0 - 100kHz Speed, B - 2.3MHz speed
	DS_11 = Read();
	DS_12 = Read();
	//if(DS_11 == 0xBE && DS_12 == 0x36)
//	{
	Write(0xAA);
//	}
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

void Write_Sequencer(void)
{
	Start();
	Write(0xCC);	//SKIP ROM
	Write(0x66);  //Command Start
	Write(0x0C-2);	//Len
	Write(0x11);  //Write Sequencer
	Write(0x00);  //ADDR_LO
	Write(0x00);  //ADDR_Hi
	Write(0xCC);  // SENS_VDD_ON
//Write(0x01);  //~CS HIGH
//	Write(0xDD);  //Delay
//	Write(0x01);	//2^x ms delay
	Write(0x80);  //~CS LOW
	Write(0xC0);  //SPI Write/Read byte
	Write(0x00);  //Lenght of Write
	Write(0x02);	//Len of Read (bytes)
 //Write(0xDD);  //Delay
//	Write(0x01);	//2ms
	Write(0xFF);  //Buffer, ADDR = 0x0A
	Write(0xFF);  //Buffer  ADDR = 0x0B
//	Write(0x01);	//~CS HIGH
//	Write(0xBB);  //SENS_VDD_OFF
	SR_11 = Read();
	SR_12 = Read();
	Write(0xAA);
	delay(1000);
	SR_3 = Read();  //must be 0xFF
	SR_4 = Read();  // 0x01
	SR_5 = Read();  // 0xAA
	SR_6 = Read();  // 0x7E
	SR_7 = Read();  // 0x10
	
}

void Read_Sequencer(void)
{
	Start();
	Write(0xCC);  // Skip ROM
	Write(0x66);  // Start Command
	Write(0x03);  // Command Len
	Write(0x22);  // Read Sequencer Command
	Write(0x00);  // Start ADDR
	Write(0x34);  // Finish ADDR
	i = Read();
	j = Read();
	Write(0xAA);
	delay(1000);
	byte_0 = Read();
	byte_1 = Read();
	byte_2 = Read();
	byte_3 = Read();
  byte_4 = Read();
	byte_5 = Read();
	byte_6 = Read();
	byte_7 = Read();
	byte_8 = Read();
	byte_9 = Read();
	byte_10 = Read();
  byte_11 = Read();
  byte_12 = Read();
	byte_13 = Read();  // buffer 
	byte_14 = Read();  // buffer
	byte_15 = Read();
	byte_16 = Read();
}
	
void Run_Sequencer(void)
{
	Start();
	Write(0xCC);
	Write(0x66);
	Write(0x04);
	Write(0x33);
	Write(0x00);
	Write(0x34);
	Write(0x00);
	Run_0 = Read();
	Run_1 = Read();
	Write(0xAA);
	delay(1000);
}
void Check_Run(void)
{
	Start();
	rs_0 = Read();
	rs_1 = Read(); 
	rs_2 = Read(); 
	rs_3 = Read(); 
	rs_4 = Read();
}

uint16_t Read_Pull(void) // pulls the register from sequencer memory
{
	Start();
	Write(0xCC);  // Skip ROM
	Write(0x66);  // Start Command
	Write(0x03);  // Command Len
	Write(0x22);  // Read Sequencer Command
	Write(0x00);  // Start ADDR
	Write(0x34);  // Finish ADDR
	i = Read();
	j = Read();
	Write(0xAA);
	delay(1000);
	res_0 = Read();
	res_1 = Read();
	res_2 = Read();
	res_3 = Read();
  res_4 = Read();
	res_5 = Read();
	res_6 = Read();
	res_7 = Read();
	res_8 = Read();
	res_9 = Read();
	res_10 = Read();
  res_11 = Read(); //buffer
  res_12 = Read(); //buffer
	res_13 = Read();
	res_14 = Read();
	res_15 = Read();
	res_16 = Read();
}

void Clear_POR(void)
{
	Start();
	Write(0xCC); // Skip ROM
	Write(0x66);
	Write(0x01);
	Write(0x7A);
	por_0 = Read();
	por_1 = Read();
	Write(0xAA);
	delay(1000);
	por_2 = Read(); 
	por_3 = Read();
	por_4 = Read();
	por_5 = Read();
	por_6 = Read();
	por_7 = Read();
	por_8 = Read();
	por_9 = Read();
	por_10 = Read();
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
	Presence = Start();
	HAL_Delay(1);
	Write(0xCC); //skip ROM
	Write(0x44);
	HAL_Delay(800);
			
	Presence = Start();
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
	Check1();
	
	Clear_POR();
	
//	Step_4();
//	delay(1000);
//	Check2();
	
	set2SPI();
	delay(1000);
	Check2();
	
	Write_Sequencer();
	
	Read_Sequencer();
	
	Run_Sequencer();
	delay(10000);
//	Check_Run();
	
	Read_Pull(); 
	
	HAL_Delay(100);
	

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  htim6.Init.Prescaler = 32-1;
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
