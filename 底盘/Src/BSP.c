#include "BSP.h"

volatile unsigned long long FreeRTOSRunTimeTicks;

xQueueHandle UART1_RX_QueHandle;//����1���ն���
xQueueHandle UART2_RX_QueHandle;//����2���ն���
xQueueHandle UART6_RX_QueHandle;//����6���ն���
xQueueHandle UART8_RX_QueHandle;//����8���ն���

void Power_Init(void)
{
#if BoardNew

HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);   //power1
HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);   //power2
HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);   //power3
HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //power4

#endif
	HAL_Delay(50);
}


void ConfigureTimerForRunTimeStats(void)  //ʱ��ͳ��
{
	FreeRTOSRunTimeTicks = 0;
	MX_TIM3_Init(); //����50us��Ƶ��20K
}
void BSP_Init(void)
{
	
	/*���ź�����ʱ��*/
  MX_GPIO_Init();
	HAL_Delay(1000);
	Power_Init();
	/*dma*/
  MX_DMA_Init();
	/*can*/
	MX_CAN1_Init();
	MX_CAN2_Init();	
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	/*��ʱ��*/
  MX_TIM5_Init();
 MX_TIM12_Init();//����ģ�鶨ʱ��
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM6_Init();

	SystemState_Inite();
  /*ADC*/
	MX_ADC1_Init();
	/*����*/
//  MX_UART8_Init();
//  MX_USART1_UART_Init();
	MX_USART2_UART_Init();//��bug
//  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
	/*SPI*/
	MX_SPI5_Init();

//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

//	__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	
	/*ʹ�ܴ��ڽ���*/
	HAL_UART_Receive_DMA(&huart6,USART6_RX_DATA,SizeofReferee);
	
	/*ʹ��DMA�ж�*/
//	HAL_UART_Receive_DMA(&huart1,USART1_RX_DATA,SizeofRemote); //��һ����Ŀ���Ǵ���һ�ν����ڴ棬��CAN��һ��
//	HAL_UART_Receive_DMA(&huart8,UART8_RX_DATA,SizeofJY901);
//  HAL_UART_Receive_DMA(&huart8,HOST_Buffer.buffer,sizeof(HOST_Buffer.buffer));//Sabar
//	HAL_UART_Receive_DMA(&huart2,USART2_RX_DATA,SizeofMinipc);
HAL_UART_Receive_DMA(&huart6,USART6_RX_DATA,SizeofReferee);  	

/*�ر�dma�ж�*/
   __HAL_DMA_DISABLE_IT(&hdma_usart6_rx,DMA_IT_TC|DMA_IT_HT|DMA_IT_TE|DMA_IT_FE|DMA_IT_DME);
   /* ���г�ʼ��  */
//  UART1_RX_QueHandle=xQueueCreate(SizeofRemote,30);
//	if(NULL==UART1_RX_QueHandle) while(1); 
//  UART2_RX_QueHandle=xQueueCreate(5,30);
//  UART6_RX_QueHandle=xQueueCreate(5,30);
//  UART8_RX_QueHandle=xQueueCreate(SizeofJY901,30);
//	if(NULL==UART8_RX_QueHandle) while(1);   

/*����ADC��DMA���գ�ע�⻺�治��С��2����������Ϊ_IO�ͼ��ױ���*/
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)uhADCxConvertedValue, 10); 
	/*������*/
	// MPU6500_Init();
	/*ʹ��can�ж�*/
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0); 
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
	
	HAL_Delay(1000);

}
