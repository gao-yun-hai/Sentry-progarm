/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "led_task.h"
#include "SystemState.h"
#include "gpio.h"
#include "Motor_USE_CAN.h"
#include "communication.h "
#include "usart.h"
/* �ڲ��궨��----------------------------------------------------------------*/

/* �ڲ��Զ�����������--------------------------------------------------------*/

/* ���������Ϣ����----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* �ڲ���������--------------------------------------------------------------*/
#define LED_PERIOD  600
#define Check_PERIOD  100
/* �ⲿ��������--------------------------------------------------------------*/

SystemStateDef Remote;
SystemStateDef Chassis_motor;
SystemStateDef Gimbal_Motor;
SystemStateDef JY61;
/* �ⲿ����ԭ������-----------------------------------------------------------

-----------------------------------------------------------------------------
-*/
/* �ڲ�����------------------------------------------------------------------*/
/* �ڲ�����ԭ������----------------------------------------------------------*/

/* �������岿�� -------------------------------------------------------------*/



void Led_Task(void const * argument)
{
	
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		
		RefreshTaskOutLineTime(LedTask_ON);
		

				
				if((SystemState.OutLine_Flag&0x01))
				{
					  Chassis_motor.Mode=1;
					  printf("motor1 over");
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
				    Chassis_motor.Mode=0;
				}
				
				if((SystemState.OutLine_Flag&0x02))
				{
					  Chassis_motor.Mode=2;
					  printf("motor2 over");
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
					Chassis_motor.Mode=0;
				}
				
				if((SystemState.OutLine_Flag&0x04))
				{
					  Chassis_motor.Mode=3;
					  printf("motor3 over");
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
					Chassis_motor.Mode=0;
				}
				
				if((SystemState.OutLine_Flag&0x08))
				{
					  Chassis_motor.Mode=4;
					  printf("motor4 over");
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
						Chassis_motor.Mode=0;
				}
				
				if((SystemState.OutLine_Flag&0x10))
				{
					  printf("BANGA over");
						Chassis_Motor_Disable(&hcan1);
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
						Chassis_motor.Mode=0;
				}
				
				if((SystemState.OutLine_Flag&0x20))//����ϵͳͨ��ʧ��
				{
					  printf("Referee over");
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
						Chassis_motor.Mode=0;
				}

				if((SystemState_can.OutLine_Flag&0x01))//ң��������
				{
					  printf("remote over");
						osDelayUntil(&xLastWakeTime,200);				  
				}
				if((SystemState_can.OutLine_Flag&0x02))
				{
					  printf("motor_Y over");
						osDelayUntil(&xLastWakeTime,200);
				}
				if((SystemState_can.OutLine_Flag&0x04))
				{
					  printf("motor_P over");
						osDelayUntil(&xLastWakeTime,200);
				} 
				if((SystemState_can.OutLine_Flag&0x08))
				{
					  printf("motor_B over");
						osDelayUntil(&xLastWakeTime,200);
				} 
				
				if((SystemState_can.OutLine_Flag&0x10))
				{
					  printf("JY61 over");
					  
				} 
			
			osDelayUntil(&xLastWakeTime,LED_PERIOD);
 }
}



void Check_Task(void const * argument)
{
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
	if((SystemState.task_OutLine_Flag&0x01))
				{
					printf("testTask GG \n\t");
					osDelayUntil(&xLastWakeTime,100);
				}
				
				
				if((SystemState.task_OutLine_Flag&0x02))
				{
					printf("ChassisContrlTask GG \n\t");
					Chassis_Motor_Disable(&hcan1);
					osDelayUntil(&xLastWakeTime,100);
				} 
				
				
				if((SystemState.task_OutLine_Flag&0x04))
				{
						printf("LedTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 


				if((SystemState.task_OutLine_Flag&0x08))
				{
						printf("vOutLineCheckTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 

				
	    	if((SystemState_can.task_OutLine_Flag&0x01))
				{
					printf("testTask GG \n\t");
					osDelayUntil(&xLastWakeTime,100);
				}
	  
				if((SystemState_can.task_OutLine_Flag&0x02))
				{
						printf("RemoteDataTask GG \n\t");
			      Remote_Disable();
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState_can.task_OutLine_Flag&0x04))
				{
						printf("GimbalContrlTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState_can.task_OutLine_Flag&0x08))
				{
						printf("GunTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState_can.task_OutLine_Flag&0x10))
				{
						printf("LedTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 


				if((SystemState_can.task_OutLine_Flag&0x20))
				{
						printf("vOutLineCheckTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 
			
				osDelayUntil(&xLastWakeTime,Check_PERIOD);
	
	}
}


