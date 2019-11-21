#include "led_task.h"
#include "SystemState.h"
#include "gpio.h"
#include "Motor_USE_CAN.h"
#include "communication.h "
#include "usart.h"

#define Check_PERIOD  16
#define BLINK_PERIOD  200
#define OFF_PERIOD    300

#define BLINK_GREEN();  {GREEN_LED(1);osDelayUntil(&xLastWakeTime,BLINK_PERIOD);GREEN_LED(0);osDelayUntil(&xLastWakeTime,OFF_PERIOD);}
#define BLINK_RED();    {RED_LED(1);osDelayUntil(&xLastWakeTime,BLINK_PERIOD);RED_LED(0);osDelayUntil(&xLastWakeTime,OFF_PERIOD);}
void Led_Task(void const * argument)
{
	
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		 while(0)
    {
      OFF:GREEN_LED(0);
      osDelayUntil(&xLastWakeTime,2000);
    }
//		if((SystemState.task_OutLine_Flag&0x01))//云台任务
//		{
//			 BLINK_GREEN();
//			 goto OFF;
//		}
//	  else if((SystemState.task_OutLine_Flag&0x02))//检测掉线任务
//    {
//      BLINK_GREEN();
//      BLINK_GREEN();
//      goto OFF;
//    }
//    else if((SystemState.task_OutLine_Flag&0x04))//Minipc任务
//    {
//      BLINK_GREEN();
//      BLINK_GREEN();
//      BLINK_GREEN();
//      goto OFF;
//    }
//    else if((SystemState.task_OutLine_Flag&0x08))//Minipc任务
//    {
//      BLINK_GREEN();
//      BLINK_GREEN();
//      BLINK_GREEN();
//      BLINK_GREEN();
//      goto OFF;
//    }
//    else if((SystemState.task_OutLine_Flag&0x10))//RemoteDataTask_ON,
//    {
//      BLINK_GREEN();
//      BLINK_GREEN();
//      BLINK_GREEN();
//      BLINK_GREEN();
//      BLINK_GREEN();
//      goto OFF;
//    }
//    else 
//    {
//      GREEN_LED(1);
      GREEN_Blink();
      osDelayUntil(&xLastWakeTime,Check_PERIOD);
//    }
	  	osDelay(50);
		
	 }
 
}


void Check_Task(void const * argument)
{
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
  
    while(0)
    {
      OFF:RED_LED(0);
      osDelayUntil(&xLastWakeTime,2000);
    }
    if((SystemState.OutLine_Flag&0x01))//MOTOR_YAW
    {
      BLINK_RED();
      goto OFF;
    }
    
    if((SystemState.OutLine_Flag&0x02))//MOTOR_PIT
    {
      BLINK_RED();
      BLINK_RED();
      goto OFF;
    }
    
//    if((SystemState.OutLine_Flag&0x04))//minipc
//    {
//      BLINK_RED();
//      BLINK_RED();
//      BLINK_RED();
//      goto OFF;
//    }
    
//    if((SystemState.OutLine_Flag&0x08))//remote
//    {
//      BLINK_RED();
//      BLINK_RED();
//      BLINK_RED();
//      BLINK_RED();
//      goto OFF;
//    }
    
    else 
    {
      RED_LED(1);
      osDelayUntil(&xLastWakeTime,Check_PERIOD);
    }
	}
}
