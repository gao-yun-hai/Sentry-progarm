/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define UART8_TX_Pin GPIO_PIN_1
#define UART8_TX_GPIO_Port GPIOE
#define UART8_RX_Pin GPIO_PIN_0
#define UART8_RX_GPIO_Port GPIOE
#define CAN1_RX_Pin GPIO_PIN_0
#define CAN1_RX_GPIO_Port GPIOD
#define CAN2_RX_Pin GPIO_PIN_12
#define CAN2_RX_GPIO_Port GPIOB
#define USART6_TX_Pin GPIO_PIN_14
#define USART6_TX_GPIO_Port GPIOG
#define CAN1_TX_Pin GPIO_PIN_1
#define CAN1_TX_GPIO_Port GPIOD
#define USART1_RX_Pin GPIO_PIN_7
#define USART1_RX_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_13
#define CAN2_TX_GPIO_Port GPIOB
#define USART3_RX_Pin GPIO_PIN_11
#define USART3_RX_GPIO_Port GPIOC
#define USART3_TX_Pin GPIO_PIN_10
#define USART3_TX_GPIO_Port GPIOC
#define USART6_RX_Pin GPIO_PIN_9
#define USART6_RX_GPIO_Port GPIOG
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define TIM5_CH2_Pin GPIO_PIN_11
#define TIM5_CH2_GPIO_Port GPIOH
#define TIM5_CH1_Pin GPIO_PIN_10
#define TIM5_CH1_GPIO_Port GPIOH
/* USER CODE BEGIN Private defines */
#define Mocha_Bopan           1   //0为关闭   1为开启
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
