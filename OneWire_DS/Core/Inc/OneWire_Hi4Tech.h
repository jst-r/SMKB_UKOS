// DS28E18 library by Hi4Tech
// Authors: butaforsky, marcus_fur, jstre
#ifndef ONE_WIRE_H
#define ONE_WIRE_H


#include "main.h"


//Functions Declarations
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void us_delay(uint16_t);
void DWT_Init(void);
uint8_t Start(void);
void Write(uint8_t);
uint8_t Read(void);
uint16_t Step_1(void);
void Step_3(void);
void Step_4(void);
void set2SPI(void);
void Check(char);
void Write_Sequencer(void);
void Read_Sequencer(void);
void Run_Sequencer(void);
uint16_t Read_Pull(void);
void Clear_POR(void);
uint16_t run_OW(void);
void init_OW(void);

#endif