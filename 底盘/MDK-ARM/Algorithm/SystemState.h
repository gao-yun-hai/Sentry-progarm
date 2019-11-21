#ifndef __SysState_H__
#define __SysState_H__

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"


#define OutLine_Time 100 //���߼��ʱ��
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

typedef enum
{
		Motor1_NO,
	  Motor2_NO,
		Motor3_NO,
		Motor4_NO,
	  BANGA_NO,
	  Referee_ON,
	
		DeviceTotal_No	
}DeviceX_NoDEF;

typedef enum
{
	testTask_ON,
	ChassisContrlTask_ON,
	RemoteDataTask_ON,
	LedTask_ON,
	GunTask_ON,
	vOutLineCheckTask_ON,
	
	TASKTotal_No	
}TASK_NoDEF;

extern SystemStateDef SystemState;
extern SystemStateDef SystemState_can;
extern SystemStateDef Remote;
extern SystemStateDef Chassis_motor;


int SystemState_Inite(void);//SystemState��ʼ��
void RefreshSysTime(void);//ˢ��ϵͳʱ�䣨mm��
float GetSystemTimer(void);//��ȡϵͳ��ǰ׼ȷʱ��


void OutLine_Check(void);//���߼����
void TASK_Check(void);//������
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No);//ˢ������ͨ��ʱ��ʱ������
void RefreshTaskOutLineTime(TASK_NoDEF Task_No);


void vOutLineCheck_Task(void const *argument);





#endif
