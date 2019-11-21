#include "date_pro_task.h"
#include "main.h"
#include "minipc.h"
#include "gimbal_task.h"
#include "SystemState.h"   

#define press_times  20

uint8_t press_counter;
int8_t chassis_gimble_Mode_flg;

SystemStateDef Remote;

#define REMOTE_PERIOD 1
pid_t pid_minipc_yaw={0};
pid_t pid_minipc_pit={0};

extern RC_Ctl_t RC_Ctl; //ң������
extern Heat_Gun_t  ptr_heat_gun_t;
uint8_t set_flag = 0;
extern float llastpit_angle;
uint8_t h=0;
/***************************************************************************************
**
	*	@brief	RemoteControlProcess()
	*	@param
	*	@supplement	��ң�������жԽӣ���ң���������ݽ��д���ʵ�ֶԵ��̡���̨����������Ŀ���
	*	@retval	
****************************************************************************************/

void RemoteControlProcess()  
{

					if(press_counter>=press_times)
					{
		        Chassis.state=1;
							press_counter=press_times+1;
						
									if(RC_Ctl.rc.s1==1)
									{
//										ptr_heat_gun_t.sht_flg=1;//���̵���ģʽ���ϣ�
										press_counter=0;
								minipc_rx.state_flag=3;//Զ���뵥��
										
									}
                  else if(RC_Ctl.rc.s1==2)
                  {
									minipc_rx.state_flag=2;
                   ptr_heat_gun_t.sht_flg=2;//���̶̹��Ƕ�ģʽ���£�
                  }
									else {
										ptr_heat_gun_t.sht_flg=0;
								minipc_rx.state_flag=0;//ֹͣ���У�//�����õı�־λ
									}
				
//							CAN_Send_YT(&hcan2, Chassis.mode,Chassis.flag,minipc_rx.state_flag,minipc_rx.bopan_flag,Chassis.state);
//					   CAN_Send_YT(&hcan2, Chassis.mode,Chassis.flag,minipc_rx.state_flag,minipc_rx.bopan_flag);
					}
}

/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	�Լ�������ݽ��д���
	*	@retval	
****************************************************************************************/
void MouseKeyControlProcess()
{
	static uint8_t shut_flag;

			//��꣨�ƶ��ٶ�*1000/50��
	pit_set.expect = pit_set.expect+RC_Ctl.mouse.y/2;	
	yaw_set.expect = yaw_set.expect+RC_Ctl.mouse.x/2;	
				
					
		if(RC_Ctl.mouse.press_l==1)        //����������
		{
			press_counter++;
			if(press_counter>=10)
				{
					press_counter=10+1;
					
					 switch(shut_flag)
					 {
						 case 1:
						 {
								ptr_heat_gun_t.sht_flg=1; 
						 }break;
						 case 2:
						 {
							 ptr_heat_gun_t.sht_flg=2; 
						 }
						 case 3:
						 {
							 ptr_heat_gun_t.sht_flg=2; 
						 }break;
						 
						 default : break;
				   }

					press_counter=0;
				}
		}
		else 	if(RC_Ctl.key.v & 0x100)     //r���л�����ģʽ
		{
			press_counter++;
					if(press_counter>=50)
					{
						
								press_counter=0;
								shut_flag++;									
								press_counter=0; 
						
					}
					if(shut_flag>3) shut_flag=1;
		}
	
}


/***************************************************************************************
**
	*	@brief	hard_brak()
	*	@param
	*	@supplement	����ֹͣ����
	*	@retval	
****************************************************************************************/
void hard_brak()
{

}


/* �������岿�� -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	ң�����ݽ��ռ���������
	*	@retval	
****************************************************************************************/
void Remote_Data_Task(void const * argument)
{
	
 Chassis.state=0;


	
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	
	
	for(;;)
	{
   
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14); //GRE_main
			
    	RefreshTaskOutLineTime(RemoteDataTask_ON);
			
//			Send_MiniPC_Data(ptr_jy901_t_angular_velocity.vz,ptr_jy901_t_angular_velocity.vy,0);
//			CAN_Send_YK(&hcan2,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
				switch(RC_Ctl.rc.s2)
				{
					case 1: RemoteControlProcess();break; 
					case 2: MouseKeyControlProcess();break; 
					case 3: Chassis.state=0;break; 
					default :break;

				}					
					CAN_Send_YT(&hcan2, Chassis.mode,Chassis.flag,minipc_rx.state_flag,minipc_rx.bopan_flag,Chassis.state);
            press_counter++;
		
		
		if(Remote.Mode)//ң��������
		{
		    Remote_Disable();
		}

			osDelay(5);
	}
	
}
/***************************************************************************************
**
	*	@brief	MiniPC_Data_task(void const * argument)
	*	@param
	*	@supplement	�Ӿ����ݴ�������
	*	@retval	
****************************************************************************************/
void Minipc_Pid_Init()
{
		PID_struct_init(&pid_minipc_yaw, POSITION_PID, 6000, 5000,
									3.08f,	0.000001f, 0.05f);  //	1.3f,	0.01f, 1.0f
		//pid_pos[i].deadband=500;
		PID_struct_init(&pid_minipc_pit, POSITION_PID, 6000, 5000,
									0.075f,	0.000002f, 0.0f	);  // 0.01f,	0.1f, 1.0f
		pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	
	//yaw_get=+_640 pit_get=+_360
//    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //��Դ���� _����
}
/***************************************************************************************
**
	*	@brief	MiniPC_Data_task(void const * argument)
	*	@param
	*	@supplement	�Ӿ����ݴ�������
	*	@retval	
****************************************************************************************/
extern uint8_t flag;
void MiniPC_Data_task(void const * argument)
{
	minipc_rx.state_flag = 0;  
	minipc_rx.angle_pit  = 0;
	minipc_rx.angle_yaw  = 0;  
  Chassis.state=0;	
//	Minipc_Pid_Init(); 

	for(;;)
	{
		 if(flag)
		 { 
			  flag=0;
			 portTickType xLastWakeTime;
		 xLastWakeTime = xTaskGetTickCount();
	

			RefreshTaskOutLineTime(MiniPC_Data_task_ON);
			
			
			if(minipc_rx.distance<200)
       Chassis.mode=1;
			else Chassis.mode=0;
   		CAN_Send_YT(&hcan2, Chassis.mode,Chassis.flag,minipc_rx.state_flag,minipc_rx.bopan_flag,Chassis.state);
			
//			if(minipc_rx.angle_yaw>640) minipc_rx.angle_yaw=640;
//			if(minipc_rx.angle_yaw<-640) minipc_rx.angle_yaw=-640;
//	    if(minipc_rx.angle_pit>360) minipc_rx.angle_pit=360;
//			if(minipc_rx.angle_pit<-360) minipc_rx.angle_pit=-360;
			
//			pid_calc(&pid_minipc_yaw, (int16_t)minipc_rx.angle_yaw, 0);
//			pid_calc(&pid_minipc_pit, (int16_t)minipc_rx.angle_pit, 0);
//			pid_minipc_yaw.pos_out=-(pid_minipc_yaw.pos_out);
//			pid_minipc_pit.pos_out=-(pid_minipc_pit.pos_out);
//		
//     minipc_rx.angle_pit=minipc_rx.angle_pit*0.095;	
//			minipc_rx.angle_yaw=minipc_rx.angle_yaw*3;
		  yaw_set.expect+= minipc_rx.angle_yaw;//+yaw_get.total_angle;
			if(set_flag == 0)
			{
				pit_set.expect=llastpit_angle;
				set_flag = 1;
			}
				
//			pit_set.expect+= minipc_rx.angle_pit;
//			pit_set.expect=pit_get.total_angle;
//			 pit_set.expect+=pid_minipc_pit.pos_out;
      pit_set.expect+=minipc_rx.angle_pit;
			
//			pit_set.expect= minipc_rx.angle_pit+pit_get.total_angle;
		
	
//			yaw_set.expect=pid_minipc_yaw.pos_out+yaw_get.total_angle;
//			pit_set.expect=pid_minipc_pit.pos_out+pit_get.total_angle;
//			yaw_set.mode = minipc_rx.state_flag;
//			osDelay(5);
   	}
		 
					osDelay(5);

	}
}


