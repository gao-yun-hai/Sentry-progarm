/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "arm_math.h"
#include "user_lib.h"

#include "gimbal_task.h"
/* USER CODE BEGIN 0 */
//#define GIMBAL_CONTROL_TIME 1
/* USER CODE END 0 */

TIM_HandleTypeDef htim5;

/* TIM5 init function */
void MX_TIM5_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 839;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim5);

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();
  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspPostInit 0 */

  /* USER CODE END TIM5_MspPostInit 0 */
  
    /**TIM5 GPIO Configuration    
    PH11     ------> TIM5_CH2
    PH10     ------> TIM5_CH1 
    */
    GPIO_InitStruct.Pin = TIM5_CH2_Pin|TIM5_CH1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM5_MspPostInit 1 */

  /* USER CODE END TIM5_MspPostInit 1 */
  }

}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();
  }
  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */
void TIM5_PWM_Init(uint32_t speed1,uint32_t speed2)
{
	  TIM5->CCR1=speed1;
	  TIM5->CCR2=speed2;
}

/**
	**************************************************************
	** Descriptions: 摩擦轮电机初始化函数
	** Input:  NULL	
	** Output: NULL
	**************************************************************
**/
//#define SHOOT_CONTROL_TIME  GIMBAL_CONTROL_TIME

//#define Fric_UP 120
//#define Fric_DOWN 100
//#define Fric_OFF 100

void GUN_Init(void)
{
  /*摩擦轮*/
		__HAL_TIM_ENABLE(&htim5);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	TIM5_PWM_Init(100,100);
 HAL_Delay(2000);
	
  ramp_init(&fric1_ramp,  0.01f, 110,100);
	ramp_init(&fric2_ramp, 0.01f, 110, 100);
	
//	__HAL_TIM_ENABLE(&htim5);
//		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
//		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
//		TIM5_PWM_Init(200,200);
//		HAL_Delay(3000);
//		TIM5_PWM_Init(100,100);
//		HAL_Delay(2000);
//	ramp_init(&fric1_ramp, SHOOT_CONTROL_TIME * 0.1f, Fric_UP, Fric_OFF);
//	ramp_init(&fric2_ramp, SHOOT_CONTROL_TIME * 0.1f, Fric_UP, Fric_OFF);
	
//	TIM5->CCR1 = 200;
//	HAL_Delay(3000);
//	TIM5->CCR1 = 100;
//	HAL_Delay(2000);
//	TIM5->CCR1= 101;
//	HAL_Delay(1000);
//	TIM5->CCR2 = 200;
//	HAL_Delay(3000);
//	TIM5->CCR2 = 100;
//	HAL_Delay(2000);
//	TIM5->CCR2= 101;
//	HAL_Delay(1000);
//	TIM5_PWM_Init(110,110);
}
//void GUN_Init(void)
//{
////	
////		__HAL_TIM_ENABLE(&htim5);
////	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
////	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
////	HAL_Delay(2000);
////	TIM5_PWM_Init(200,200);
////	HAL_Delay(3000);
////	TIM5_PWM_Init(100,100);
////	HAL_Delay(2000);
////	TIM5_PWM_Init(101,0);
////	HAL_Delay(1000);
////	TIM5_PWM_Init(110,0);
////	HAL_Delay(3000);
////	TIM5_PWM_Init(110,101);
////	HAL_Delay(2000);
////	TIM5_PWM_Init(110,110);
////	HAL_Delay(1000);
////		TIM5_PWM_Init(105,105);
////	HAL_Delay(2000);
////	ramp_init(&fric_ramp,0.008,125,102);
////	HAL_Delay(1000);
////	ramp_init(&fric_ramp,0.008,125,102);
////	HAL_Delay(1000);
////	ramp_init(&fric1_ramp, SHOOT_CONTROL_TIME * 0.1f, Fric_UP, Fric_OFF);
////	ramp_init(&fric2_ramp, SHOOT_CONTROL_TIME * 0.1f, Fric_UP, Fric_OFF);
//	        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
////        ramp_calc(&fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

////        if(fric1_ramp.out == fric1_ramp.max_value)
////        {
////            ramp_calc(&fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
////        }

//}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
