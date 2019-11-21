#ifndef  SystemState_H
#define  SystemState_H


#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#define OutLine_Time 50 //���߼��ʱ��
#define Motor_Stall_Time 200
#define Motor_Stall_Spd 5

#define MyFlagSet(x,y) x=x|(0x00000001<<y) //���ñ�־λ  y�ڼ�λ
#define MyFlagClear(x,y) x=x&~(0x00000001<<y)
#define MyFlagGet(x,y) (x&(0x00000001<<y))



typedef struct{
	short Mode;//����ģʽ
	short Enable;//״̬
	short State;//״̬
	short Task;//����
//	BEEPMode Beep;//������
	int Time;//System run time mm
	TIM_HandleTypeDef *htim;//ʱ����������
	uint16_t OutLine_Flag;//���߱�־
	uint16_t task_OutLine_Flag;//���߱�־	
//	RobotDistDef RobotDist;//�����˲���
}SystemStateDef;

extern SystemStateDef SystemState;
//extern SystemStateDef Remote;
//extern SystemStateDef Gimbal_Motor;
//extern SystemStateDef JY61;

typedef enum
{

		Yaw_NO,
		Pitch_NO,
  	Minipc_NO,
  Remote_NO,
	
		DeviceTotal_No	
}DeviceX_NoDEF;

typedef enum
{

	GimbalContrlTask_ON,
  vOutLineCheckTask_ON,
	MiniPC_Data_task_ON,
	RemoteDataTask_ON,
	
	TASKTotal_No	
}TASK_NoDEF;


int SystemState_Inite(void);//SystemState��ʼ��
void RefreshSysTime(void);//ˢ��ϵͳʱ�䣨mm��
uint32_t GetSystemTimer();//��ȡϵͳ��ǰ׼ȷʱ��


void OutLine_Check(void);//���߼����
void TASK_Check(void);//������
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No);//ˢ������ͨ��ʱ��ʱ������
void RefreshTaskOutLineTime(TASK_NoDEF Task_No);

void vOutLineCheck_Task(void const *argument);


#endif
