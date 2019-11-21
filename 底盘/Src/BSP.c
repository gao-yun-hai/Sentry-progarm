#include "BSP.h"

volatile unsigned long long FreeRTOSRunTimeTicks;

xQueueHandle UART1_RX_QueHandle;//串口1接收队列
xQueueHandle UART2_RX_QueHandle;//串口2接收队列
xQueueHandle UART6_RX_QueHandle;//串口6接收队列
xQueueHandle UART8_RX_QueHandle;//串口8接收队列

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


void ConfigureTimerForRunTimeStats(void)  //时间统计
{
	FreeRTOSRunTimeTicks = 0;
	MX_TIM3_Init(); //周期50us，频率20K
}
void BSP_Init(void)
{
	
	/*引脚和引脚时钟*/
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
	/*定时器*/
  MX_TIM5_Init();
 MX_TIM12_Init();//测速模块定时器
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM6_Init();

	SystemState_Inite();
  /*ADC*/
	MX_ADC1_Init();
	/*串口*/
//  MX_UART8_Init();
//  MX_USART1_UART_Init();
	MX_USART2_UART_Init();//有bug
//  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
	/*SPI*/
	MX_SPI5_Init();

//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

//	__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	
	/*使能串口接收*/
	HAL_UART_Receive_DMA(&huart6,USART6_RX_DATA,SizeofReferee);
	
	/*使能DMA中断*/
//	HAL_UART_Receive_DMA(&huart1,USART1_RX_DATA,SizeofRemote); //这一步的目的是创建一段接受内存，和CAN的一样
//	HAL_UART_Receive_DMA(&huart8,UART8_RX_DATA,SizeofJY901);
//  HAL_UART_Receive_DMA(&huart8,HOST_Buffer.buffer,sizeof(HOST_Buffer.buffer));//Sabar
//	HAL_UART_Receive_DMA(&huart2,USART2_RX_DATA,SizeofMinipc);
HAL_UART_Receive_DMA(&huart6,USART6_RX_DATA,SizeofReferee);  	

/*关闭dma中断*/
   __HAL_DMA_DISABLE_IT(&hdma_usart6_rx,DMA_IT_TC|DMA_IT_HT|DMA_IT_TE|DMA_IT_FE|DMA_IT_DME);
   /* 队列初始化  */
//  UART1_RX_QueHandle=xQueueCreate(SizeofRemote,30);
//	if(NULL==UART1_RX_QueHandle) while(1); 
//  UART2_RX_QueHandle=xQueueCreate(5,30);
//  UART6_RX_QueHandle=xQueueCreate(5,30);
//  UART8_RX_QueHandle=xQueueCreate(SizeofJY901,30);
//	if(NULL==UART8_RX_QueHandle) while(1);   

/*开启ADC的DMA接收，注意缓存不能小于2，不能设置为_IO型即易变量*/
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)uhADCxConvertedValue, 10); 
	/*陀螺仪*/
	// MPU6500_Init();
	/*使能can中断*/
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0); 
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
	
	HAL_Delay(1000);

}
