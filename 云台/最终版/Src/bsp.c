#include "bsp.h" 
#include "communication.h"
#include "minipc.h"
#include "tim.h"


void BSP_Init(void)
{
	
	/*引脚和引脚时钟*/
  MX_GPIO_Init();
	HAL_Delay(2000);
//	Power_Init();
	/*dma*/
  MX_DMA_Init();
	/*can*/
	MX_CAN1_Init();
	MX_CAN2_Init();	
	CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);
	/*定时器*/
  MX_TIM5_Init();
  //MX_TIM12_Init();//测速模块定时器
	//MX_TIM6_Init();
//  SystemState_Inite();
  /*ADC*/
	//MX_ADC1_Init();
	/*串口*/
  MX_UART8_Init();
  MX_USART1_UART_Init();
	MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
	/*SPI*/
	//MX_SPI5_Init();

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
//	__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	
	/*使能DMA中断*/
	HAL_UART_Receive_DMA(&huart1,USART1_RX_DATA,SizeofRemote); //这一步的目的是创建一段接受内存，和CAN的一样
//	HAL_UART_Receive_DMA(&huart8,UART8_RX_DATA,SizeofJY901);
//  HAL_UART_Receive_DMA(&huart8,HOST_Buffer.buffer,sizeof(HOST_Buffer.buffer));//Sabar
  HAL_UART_Receive_DMA(&huart6,USART6_RX_DATA,SizeofMinipc);

   /* 队列初始化  */
//  UART1_RX_QueHandle=xQueueCreate(SizeofRemote,30);
//	if(NULL==UART1_RX_QueHandle) while(1); 
//  UART2_RX_QueHandle=xQueueCreate(5,30);
//  UART6_RX_QueHandle=xQueueCreate(5,30);
//  UART8_RX_QueHandle=xQueueCreate(SizeofJY901,30);
//	if(NULL==UART8_RX_QueHandle) while(1);   

/*开启ADC的DMA接收，注意缓存不能小于2，不能设置为_IO型即易变量*/
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)uhADCxConvertedValue, 10); 
	/*陀螺仪*/
	// MPU6500_Init();
	/*摩擦轮*/
	GUN_Init();
	/*使能can中断*/
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0); 
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
	
//	JY61_Frame();
	HAL_Delay(1000);

}
