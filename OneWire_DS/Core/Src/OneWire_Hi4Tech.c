#include "OneWire_Hi4Tech.h"
#include "main.h"

TIM_HandleTypeDef htim6;
#define DS28E18_PORT GPIOC
#define DS28E18_PIN GPIO_PIN_1
#define PullUp_Pin GPIO_PIN_2

//#include "stm32f3xx_hal_msp.h"

// variables - useful and not

uint8_t Response = 0, Presence = 0;
uint32_t buffer[20] = {0};
uint16_t b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b9, b10, b11, b12, b13, b14,
b15, b16, b17, b18, b19, b20, b21, b22, b23, b24, b25, b26, b27, b28, b29, b30;




/*
//microcontroller setup//
TIMx Prescaler - (Freq(Mhz) / n(Mhz))-1
Counter period - 0xfffff-1
*/

/*Functions Definitions*/

//Set GPIOx Pin as Output
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
	{
		 GPIO_InitTypeDef GPIO_InitStruct = {0};
		 GPIO_InitStruct.Pin = GPIO_Pin;
		 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		 HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
	}
	
//Set GPIOx Pin as Input
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
	{
		 GPIO_InitTypeDef GPIO_InitStruct = {0};
		 GPIO_InitStruct.Pin = GPIO_Pin;
		 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		 GPIO_InitStruct.Pull = GPIO_NOPULL;
		 HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
	}
			 
//Set delay in microseconds
	
void delay(uint16_t time) 
	{
	__HAL_TIM_SET_COUNTER(&htim6,0);
	while((__HAL_TIM_GET_COUNTER(&htim6))<time);
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
	Write(0xCC); //skip ROM
	Write(0x66); //Start Command
	Write(0x05); //Command len
	Write(0x83); //Write GPIO Configuration Command
	Write(0x0B); //Access to the GPIO control register
	Write(0x03); //Only value allowed
	Write(0xA5); //GPIO_CTRL_HI_Value
	Write(0x0F); //GPIO_CTRL_LO_Value
	buffer[0] = Read();
	buffer[1] = Read();
	Write(0xAA);
	return Presence;
}
	
void Step_3(void) //Step 2 is skipped
{
	Presence = Start();
	Write(0xCC); //Skip ROM
	Write(0x66); //Start Command
	Write(0x05); //Command Len
	Write(0x83); //Write GPIO Configuration Command
	Write(0x0B); //Access to the GPIO control register
	Write(0x03); //Only value allowed
	Write(0xA5); //GPIO_CTRL_HI_Value
	Write(0x0F); //GPIO_CTRL_LO_Value
	buffer[2] = Read();
	buffer[3] = Read();
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
	buffer[4] = Read();
	buffer[5] = Read();
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
	buffer[6] = Read();
	buffer[7] = Read();
	Write(0xAA);
}
	
void Check(char seq)
{
	if(seq == 1) //Step 1 check
		{
			for(int i = 8; i<=12; i++)
			{
			 buffer[i] = Read();
			}
		}
	else if(seq == 2)		// Step 3 check
	{
		for(int i = 13; i<=17;i++)
		{
			buffer[i] = Read();
		}
	}
	else if(seq == 3)
		
	}
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
//Write(0xDD);  //Delay
//Write(0x01);	//2^x ms delay
	Write(0x80);  //~CS LOW
	Write(0xC0);  //SPI Write/Read byte
	Write(0x00);  //Lenght of Write
	Write(0x02);	//Len of Read (bytes)
//Write(0xDD);  //Delay
//Write(0x01);	//2ms
	Write(0xFF);  //Buffer, ADDR = 0x0A
	Write(0xFF);  //Buffer  ADDR = 0x0B
//Write(0x01);	//~CS HIGH
//Write(0xBB);  //SENS_VDD_OFF
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