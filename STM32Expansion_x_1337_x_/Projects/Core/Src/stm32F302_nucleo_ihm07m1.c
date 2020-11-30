/**
 ******************************************************************************
 * @file    stm32F302_nucleo_ihm07m1.c
 * @author  System lab - Automation and Motion control team
 * @version V1.0.0
 * @date    06-July-2015
 * @brief   This file provides the interface between the MC-lib and STM Nucleo
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


  #include "stm32F302_nucleo_ihm07m1.h"
  #include "6Step_Lib.h"

  extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters; /*!< Main SixStep structure*/ 
  extern SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters; /*!< SixStep PI regulator structure*/ 
  extern void MC_ADCx_SixStep_Bemf(void);
  extern void MC_TIMx_SixStep_timebase(void);
  extern void MC_SysTick_SixStep_MediumFrequencyTask(void);
  
  
/** @addtogroup MIDDLEWARES     MIDDLEWARES 
  * @brief  Middlewares Layer
  * @{ 
  */


/** @addtogroup MC_6-STEP_LIB    MC_6-STEP_LIB
  * @brief  Motor Control driver
  * @{ 
  */

/** @addtogroup stm32F302_nucleo_ihm07m1    stm32F302_nucleo_ihm07m1
  * @brief  Interface file for STM32F302 and Motor Control Library configuration
  * @{ 
  */

/** @defgroup MC_SixStep_ADC_Channel    MC_SixStep_ADC_Channel
  *  @{
    * @brief Select the new ADC Channel 
*/
  void MC_SixStep_ADC_Channel(uint32_t adc_ch)
  {
     ADCx.Instance->CR |= ADC_CR_ADSTP;  
     while(ADCx.Instance->CR & ADC_CR_ADSTP);   
     /* Clear the old SQx bits for the selected rank */
     ADCx.Instance->SQR1 &= ~__HAL_ADC_SQR1_RK(ADC_SQR2_SQ5, 1);
     /* Set the SQx bits for the selected rank */
     ADCx.Instance->SQR1 |= __HAL_ADC_SQR1_RK(adc_ch, 1);
     ADCx.Instance->CR |= ADC_CR_ADSTART;  
  }

/**
  * @} 
  */

/** @defgroup MC_SixStep_Nucleo_Init    MC_SixStep_Nucleo_Init
  *  @{
    * @brief Init the STM32 register
*/

  void MC_SixStep_Nucleo_Init()
  {
    TIM_ClearInputConfigTypeDef sClearInputConfig;
    ADC_ChannelConfTypeDef sConfig;
    
    /******************** ETR CONFIGURATION ************************************/
    sClearInputConfig.ClearInputState = 1;  
    sClearInputConfig.ClearInputSource = TIM_CLEARINPUTSOURCE_ETR;
    sClearInputConfig.ClearInputPolarity = TIM_CLEARINPUTPOLARITY_NONINVERTED;
    sClearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
    sClearInputConfig.ClearInputFilter = 0;
    HAL_TIM_ConfigOCrefClear(&HF_TIMx, &sClearInputConfig, HF_TIMx_CH1);
    HAL_TIM_ConfigOCrefClear(&HF_TIMx, &sClearInputConfig, HF_TIMx_CH2);
    HAL_TIM_ConfigOCrefClear(&HF_TIMx, &sClearInputConfig, HF_TIMx_CH3);
    /***************************************************************************/
    
    __HAL_FREEZE_TIM1_DBGMCU();  /* Stop TIM during Breakpoint*/
    
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK); /* Enable the TIM Break interrupt */
    
    /******************** REGULAR CHANNELS CONFIGURATION *************************/
 
    sConfig.Rank = 1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_Bemf_CH1; /* BEMF feedback phase A */
    sConfig.SamplingTime = ADC_Bemf_CH1_ST;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_Bemf_CH2; /* BEMF feedback phase B */
    sConfig.SamplingTime = ADC_Bemf_CH2_ST;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_Bemf_CH3; /* BEMF feedback phase C */
    sConfig.SamplingTime = ADC_Bemf_CH3_ST;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    /***************************************************************************/
  } 

/** @defgroup HAL_ADC_ConvCpltCallback    HAL_ADC_ConvCpltCallback
  *  @{
    * @brief ADC callback 
*/
  void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
  {
     MC_ADCx_SixStep_Bemf();
  }
/**
  * @} 
  */

/** @defgroup HAL_TIM_PeriodElapsedCallback    HAL_TIM_PeriodElapsedCallback
  *  @{
    * @brief htim callback 
*/
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
    MC_TIMx_SixStep_timebase();
  }
/**
  * @} 
  */

/** @defgroup HAL_SYSTICK_Callback    HAL_SYSTICK_Callback
  *  @{
    * @brief Systick callback 
*/
  void HAL_SYSTICK_Callback()
  {
    MC_SysTick_SixStep_MediumFrequencyTask();
  }
/**
  * @} 
  */

  

/** @defgroup DisableInput_CH1_D_CH2_D_CH3_D    DisableInput_CH1_D_CH2_D_CH3_D
  *  @{
    * @brief Disable All Input channels for L6230
  */

void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D()
{  
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_3);
}

/**
  * @} 
  */

/** @defgroup Start_PWM_driving    Start_PWM_driving
  *  @{
    * @brief Enable the PWM generation on Input channels
  */

void MC_SixStep_Start_PWM_driving()
{
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&HF_TIMx, HF_TIMx_CH1);           //TIM1_CH1 ENABLE   
  HAL_TIM_PWM_Start(&HF_TIMx, HF_TIMx_CH2);           //TIM1_CH2 ENABLE   
  HAL_TIM_PWM_Start(&HF_TIMx, HF_TIMx_CH3);           //TIM1_CH3 ENABLE  
} 

/**
  * @} 
  */

/** @defgroup Stop_PWM_driving    Stop_PWM_driving
  *  @{
    * @brief Disable the PWM generation on Input channels
  */

void MC_SixStep_Stop_PWM_driving()
{
 	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&HF_TIMx, HF_TIMx_CH1);           //TIM1_CH1 DISABLE   
  HAL_TIM_PWM_Stop(&HF_TIMx, HF_TIMx_CH2);           //TIM1_CH2 DISABLE   
  HAL_TIM_PWM_Stop(&HF_TIMx, HF_TIMx_CH3);           //TIM1_CH3 DISABLE  
}  

/**
  * @}
  */

/** @defgroup HF_TIMx_SetDutyCycle_CH1    HF_TIMx_SetDutyCycle_CH1
  *  @{
    * @brief Set the Duty Cycle value for CH1 
*/

void MC_SixStep_HF_TIMx_SetDutyCycle_CH1(uint16_t CCR_value)
{ 
	HF_TIMx.Instance->HF_TIMx_CCR1 = CCR_value; 
}
/**
  * @} 
  */

/** @defgroup HF_TIMx_SetDutyCycle_CH2    HF_TIMx_SetDutyCycle_CH2
  *  @{
    * @brief Set the Duty Cycle value for CH2 
*/

void MC_SixStep_HF_TIMx_SetDutyCycle_CH2(uint16_t CCR_value)
{ 
	HF_TIMx.Instance->HF_TIMx_CCR2 = CCR_value;
}
/**
  * @} 
  */


/** @defgroup HF_TIMx_SetDutyCycle_CH3    HF_TIMx_SetDutyCycle_CH3
  *  @{
    * @brief Set the Duty Cycle value for CH3 
*/

void MC_SixStep_HF_TIMx_SetDutyCycle_CH3(uint16_t CCR_value)
{ 
	HF_TIMx.Instance->HF_TIMx_CCR3 = CCR_value;
}

/** @defgroup Bemf_delay_calc    Bemf_delay_calc
  *  @{
    * @brief Bemf delay calculation
*/

void Bemf_delay_calc()
{ 
 if(PI_parameters.Reference>=0)
 {  
 if(SIXSTEP_parameters.speed_fdbk_filtered<=12000 && SIXSTEP_parameters.speed_fdbk_filtered>10000)
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_1;    
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=10000 && SIXSTEP_parameters.speed_fdbk_filtered>7800)
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2; 
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=7800 && SIXSTEP_parameters.speed_fdbk_filtered>6400) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;    
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=6400 && SIXSTEP_parameters.speed_fdbk_filtered>5400)  
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;      
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=5400 && SIXSTEP_parameters.speed_fdbk_filtered>4650) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;      
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=4650 && SIXSTEP_parameters.speed_fdbk_filtered>4100) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;      
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=4100 && SIXSTEP_parameters.speed_fdbk_filtered>3650) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;      
  }     
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=3650 && SIXSTEP_parameters.speed_fdbk_filtered>3300) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;      
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=3300 && SIXSTEP_parameters.speed_fdbk_filtered>2600) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;    
  } 
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=2600 && SIXSTEP_parameters.speed_fdbk_filtered>1800) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
  } 
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=1800 && SIXSTEP_parameters.speed_fdbk_filtered>1500) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
  } 
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=1500 && SIXSTEP_parameters.speed_fdbk_filtered>1300) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
  }    
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=1300 && SIXSTEP_parameters.speed_fdbk_filtered>1000) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=1000 && SIXSTEP_parameters.speed_fdbk_filtered>500) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
  }  
 }
 else 
 {
  if(SIXSTEP_parameters.speed_fdbk_filtered>=-12000 && SIXSTEP_parameters.speed_fdbk_filtered<-10000)
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_1;    
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-10000 && SIXSTEP_parameters.speed_fdbk_filtered<-7800)
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2; 
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-7800 && SIXSTEP_parameters.speed_fdbk_filtered<-6400) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;    
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-6400 && SIXSTEP_parameters.speed_fdbk_filtered<-5400)  
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;      
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-5400 && SIXSTEP_parameters.speed_fdbk_filtered<-4650) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;      
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-4650 && SIXSTEP_parameters.speed_fdbk_filtered<-4100) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;      
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-4100 && SIXSTEP_parameters.speed_fdbk_filtered<-3650) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;      
  }     
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-3650 && SIXSTEP_parameters.speed_fdbk_filtered<-3300) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;      
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-3300 && SIXSTEP_parameters.speed_fdbk_filtered<-2650) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;     
  } 
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-2600 && SIXSTEP_parameters.speed_fdbk_filtered<-1800) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
  } 
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1800 && SIXSTEP_parameters.speed_fdbk_filtered<-1500) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
  }   
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1500 && SIXSTEP_parameters.speed_fdbk_filtered<-1300) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
  }   
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1300 && SIXSTEP_parameters.speed_fdbk_filtered<-1000) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1000 && SIXSTEP_parameters.speed_fdbk_filtered<-500) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
  } 
  
 }
}

/**
  * @}  
  */

/** @defgroup Get_UART_data   Get_UART_data
  *  @{
    * @brief Get the UART value from DR register
*/
/*uint32_t Get_UART_Data()
{
  return (UART.Instance->RDR);
}       */
/**
 * @}  
 */ 

/**
  * @} // end STM32F302_Interface
  */

/**
  * @}  end MC_6-STEP_LIB 
  */
    
/**
  * @} // end MIDDLEWARES
  */
