// DS28E18 library by Hi4Tech
// Authors: butaforsky, marcus_fur, jstre
#include "main.h"

extern TIM_HandleTypeDef htim16;		//DEFINE_TIM for us delay
#define DS28E18_PORT GPIOC					//DEFINE_GPIO ports here
#define DS28E18_PIN GPIO_PIN_2
#define PullUp_Pin GPIO_PIN_1

//#include "stm32f3xx_hal_msp.h"

// variables - useful and not
uint8_t Response = 0, Presence = 0;
uint32_t buffer[100] = {0};
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
	__HAL_TIM_SET_COUNTER(&htim16,0);
	while((__HAL_TIM_GET_COUNTER(&htim16))<time);
	}
	
uint8_t Start(void)
	{
		 Set_Pin_Output(DS28E18_PORT,DS28E18_PIN);//set the pin as output
		 HAL_GPIO_WritePin(DS28E18_PORT, DS28E18_PIN, GPIO_PIN_RESET); // pull the pin low
		 delay(480);
		 Set_Pin_Input(DS28E18_PORT, DS28E18_PIN);
		 delay(65);
		 if((HAL_GPIO_ReadPin(DS28E18_PORT, DS28E18_PIN)== GPIO_PIN_RESET)) 
			 {
			 Response = 1;
			 }
		 else Response = 2;
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
		 delay(80); //wait for 60 microseconds
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
	
void Step_3(void) //Step 2 is skipped, used for multidevice systems
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
	Write(0x8E); // 3 bit
	Write(0x00); // 4 bit
	Write(0x00); // 5 bit
	Write(0x00); // 6 bit
	Write(0x00); // 7 bit
	Write(0x43); // 8 bit
	Write(0x66); 	// Start command
	Write(0x01);
	Write(0x7A);
	//buffer[4] = Read();
	//buffer[5] = Read();
	Write(0xAA);
 }

void set2SPI(void) //Check seq = 0
{
	Start();
	Write(0xCC);
	Write(0x66);
	Write(0x02);
	Write(0x55);
	Write(0x38);// least HEX char: 0 - 100kHz Speed, B - 2.3MHz speed 8 - 400kHz Speed
	buffer[6] = Read();
	buffer[7] = Read();
	Write(0xAA);
	delay(1000);
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
	else if(seq == 3) //Step 4 check
	{
		for(int i = 18; i<=22;i++)
		{
			buffer[i] = Read();
		}
	}
	else if(seq == 4) // Run Check
		{
		for(int i = 23; i<=27; i++)
				{
					buffer[i] = Read();
				}
		}
	else if(seq == 5) // Read Check
		{
		for(int i = 28; i<=28+15; i++)
				{
					buffer[i] = Read();
				}
		}
		else if(seq == 6) // Write Check
		{
		for(int i = 46; i<=50; i++)
				{
					buffer[i] = Read();
				}
		}
		else if(seq == 7) // Read Pull
		{
		for(int i = 51; i<=68; i++)
				{
					buffer[i] = Read();
				}
		}
		else if(seq == 8) // Clear POR
		{
			for(int  i = 73; i<=82; i++)
				{
					buffer[i] = Read();
				}
		}
		else  if(seq == 0) // set2SPI
		{
			for(int i = 83; i<=87; i++)
			{
				buffer[i] = Read();
			}
		}
	}
void Write_Sequencer(void) //check seq = 6
{
	Start();
	Write(0xCC);	//SKIP ROM
	Write(0x66);  //Command Start
	Write(0x0C);	//Len
	Write(0x11);  //Write Sequencer
	Write(0x00);  //ADDR_LO
	Write(0x00);  //ADDR_Hi
	Write(0xCC);  // SENS_VDD_ON
	Write(0xBB);
	Write(0xCC);
//Write(0x01);  //~CS HIGH
//	Write(0xDD);  //Delay
//	Write(0x00);	//2^x ms delay
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
	buffer[44] = Read();
	buffer[45] = Read();
	Write(0xAA);
	delay(1000);	
}

void Read_Sequencer(void) //check seq = 5
{
	Start();
	Write(0xCC);  // Skip ROM
	Write(0x66);  // Start Command
	Write(0x03);  // Command Len
	Write(0x22);  // Read Sequencer Command
	Write(0x00);  // Start ADDR
	Write(0x34);  // Finish ADDR
	buffer[83] = Read();
	buffer[84] = Read();
	Write(0xAA);
	delay(1000);
}
	
void Run_Sequencer(void) //  check seq = 4
{
	Start();
	Write(0xCC);  // SKIP ROM
	Write(0x66);  //Start Commmand
	Write(0x04);  //Command Len
	Write(0x33);
	Write(0x00);
	Write(0x34);
	Write(0x00);
	delay(100);
  buffer[69] = Read();
  buffer[70] = Read();
	Write(0xAA);
	delay(100);
	
	
}

void Read_Pull(void) // pulls the register from sequencer memory
	// check seq = 7
{
	Start();
	Write(0xCC);  // Skip ROM
	Write(0x66);  // Start Command
	Write(0x03);  // Command Len
	Write(0x22);  // Read Sequencer Command
	Write(0x00);  // Start ADDR
	Write(0x34);  // Finish ADDR
	buffer[71] = Read();
	buffer[72] = Read();
	Write(0xAA);
	delay(1000);
}

void Clear_POR(void) // Check seq = 8
{
	Start();
	Write(0xCC); // Skip ROM
	Write(0x66); // Start Command
	Write(0x01); // Access
	Write(0x7A); // Byte  select
	buffer[85] = Read();
	buffer[86] = Read();
	Write(0xAA); // Success byte
	delay(1000);
}

void init_OW(void)
{
	Step_1();
	Check(1);
//	delay(5000);
	Step_3();
	Check(2);
//	delay(10000);
//	Step_4();
//	Check(3);
//	delay(7000);
	Clear_POR();
	Check(8);
//	delay(3000);
	set2SPI();
	Check(0);
	Write_Sequencer();
	Check(6);
}	
	
void run_OW(void)
{
		
		Read_Sequencer();
		Check(5);
		Run_Sequencer();
		delay(1000);	
	//	Read_Pull();
	//	Check(7);
}
