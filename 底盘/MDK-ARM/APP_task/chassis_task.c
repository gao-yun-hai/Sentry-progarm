/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "chassis_task.h"
#include "SystemState.h"
#include "user_lib.h"
#include "stdlib.h"
#include "time.h"
/* �ڲ��궨��----------------------------------------------------------------*/
#define LEFT_ROUND 0
#define RIGHT_ROUND -130
#define Predict_MAXRemainPower 190
#define Predict_MINRemainPower 80
#define CHASSIS_PERIOD 5//5
#define FarDist 100  //Զ������λ��
#define NearMidDist 50  //�����в�����λ��
#define NearDist 10 //��������λ��
#define Max 4000
#define Min -4000
#define CHASSIS_CONTROL_TIME 0.005//0.002 
#define CHASSIS_ACCEL_X_NUM 0.1666666667f

/* �ڲ��Զ�����������--------------------------------------------------------*/

/* ���������Ϣ����----------------------------------------------------------*/
//extern osMessageQId Chassis_QueueHandle;

/* �ڲ���������--------------------------------------------------------------
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
float pid_calc(pid_t* pid, float get, float set);
void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,
													float dstVmmps_Y,float dstVmmps_W);
----------------------------------------------------------------------------
*/
/* �ⲿ��������--------------------------------------------------------------*/
moto3508_type  moto_3508_set = {.flag = 0}; 
extern float power;		//����  	_���Ա���
int8_t chassis_disable_flg;
/* ���õ��ⲿ����ԭ������----------------------------------------------------------*/

/* �ڲ�����------------------------------------------------------------------*/
//pid_t pid_3508_pos;     		 //���̵��λ�û�
//pid_t pid_3508_spd[4];			 //���̵���ٶȻ�
//pid_t pid_3508_current[4];	 //���̵����������	
//pid_t pid_chassis_follow = {0};//���̸���λ�û�
//pid_t pid_chassis_follow_spd = {0};//���̸����ٶȻ�

pid_t pid_3508_pos[2];     		 //���̵��λ�û�
pid_t pid_3508_spd[2];			 //���̵���ٶȻ�
pid_t pid_3508_current[2];	 //���̵����������	
float SpeedSet = 0;

Mode_Set Chassis;

static float Current_set[4] = {0};  //���ݸ��������ƵĻ���

//���Ա���
int16_t angle[2];
//int Run_Dir;//����ģʽ����λ
int Run_Dir1=1;//Ѳ�ߣ�����ģʽ����λ
int Run_Dir2=2;//����ģʽ����λ
int Run_Dir4=2;//ֱ�߹������ģʽ����
int Run_Dir5=1; //ֱ�߱���ģʽ����λ
fp32 round_set=0; //����Ȧ��ƽ��ֵ
int GO_Dir;
int gdflag1;
int gdflag2;
int active_flag;
int Speed;
int Crazy_n=0;
int Crazy_flag=0;
int attack_flag;

HP hp;
int round_now;
int round_last;
int round_left;
int round_right;

const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
first_order_filter_type_t chassis_cmd_slow_set_v;
ramp_function_source_t chassis_cmd_slow_set_v1;
/* �ڲ�����ԭ������----------------------------------------------------------*/
void hard_brak(void)
{
		chassis_cmd_slow_set_v.out=0;
}

void random_figure(void)
{
	static fp32 a=0,b=0,c=0;
	uint32_t d;
	d=rand()%2000;//[i8:598 1870 1855 847 1476][2: 1290 719 788 575 1061]
		if(d<1000)
	  { 
		 d=1000;
	  }
	  else if(d>1000)
	  {
		 d=2000;
	  }
	a=HAL_GetTick();

	if(a-b>d)
	{
		c=rand()%3;//[2:0 1 2 2 0]
		if((c==0)||(c==1))
		{
			Run_Dir5=c;
		}
		b=a;
	}
	
	
}
void auto_collimation(void)
{
	//gdflag1=HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_5);
  //gdflag2=HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_6);
	if(Chassis.mode==0)
	{
	  if(Chassis.flag==0)
     {
				Run_Dir4=2;//0���ң�1����2ֹͣ
     }
		
		else if((RIGHT_ROUND<round_set)&&(round_set<LEFT_ROUND)&&Chassis.flag==2)//RIGHT_ROUND<moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2<LEFT_ROUND&&Chassis.flag==2
    {			
			 Run_Dir4=0;//Run_Dir4=0;
    }
		else if((RIGHT_ROUND<round_set)&&(round_set<LEFT_ROUND)&&Chassis.flag==1)//RIGHT_ROUND<(moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2<LEFT_ROUND&&Chassis.flag==1
    {			
			 Run_Dir4=1;//Run_Dir4=1;
    }
		
		else if(round_set<=RIGHT_ROUND)//(moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2<RIGHT_ROUND
		{
			Run_Dir4=2;//Run_Dir4=2;
			if(Chassis.flag==2)
			{
				Run_Dir4=2;//Run_Dir4=2;
			}
			else if(Chassis.flag==1)
			{
			 Run_Dir4=1;//Run_Dir4=1;
			}
		}
		else if(round_set>=LEFT_ROUND)//(moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2>LEFT_ROUND
		{
			Run_Dir4=2;//Run_Dir4=2;
			if(Chassis.flag==1)
			{
				Run_Dir4=2;//Run_Dir4=2;
			}
			else if(Chassis.flag==2)
			{
			 Run_Dir4=0;//Run_Dir4=0;
			}
		}
//		else if((gdflag1==0&&Chassis.flag==2)||(gdflag2==0&&Chassis.flag==1))
//    {			
//			 Run_Dir2=2;
//    }
//	  else if(gdflag1==0&&Chassis.flag==1)
//		{
//			Run_Dir2=1;
//		}
//		else if(gdflag2==0&&Chassis.flag==2)
//		{
//			Run_Dir2=0;
//		}
//		else if(gdflag1==0&&gdflag2==0)
//		{
//			Run_Dir2=2;
//		}
		round_last=(moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2;
		active_flag=1;
		
		//Run_Dir2=2;
  }
else if(Chassis.mode==1)
	{
		Run_Dir4=2;//Tun_Dir4=2;
		round_last=(moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2;
		active_flag=1;
	}
}
void Crazy_flag_Decide(void)
{   
	static fp32 time_now=0,time_last=0,hp_last=0,hp_now=0;
	static  uint8_t  count_crazy = 0;
	time_now = HAL_GetTick();
	hp_now=Robot.remainHp;
	if(time_now-time_last>2000)
	{
			if((hp_now-hp_last < -80))
			{
				Crazy_flag=1;
			}
			else if (Crazy_flag) 
			{
				count_crazy++;
				if (count_crazy > 2)
				{
						Crazy_flag=0;
						count_crazy = 0;
				}
			}
			
			hp_last=hp_now;
			time_last=time_now;
	}

}



void attack_flag_Decide(void)
{
  static fp32 time1_now=0,time1_last=0,hp1_last=0,hp1_now=0;
	static uint8_t count1_crazy=0;
	time1_now = HAL_GetTick();
	hp1_now=Robot.remainHp;
	if(time1_now-time1_last>1000)
	{
			if((hp1_now-hp1_last < 0))
			{
				attack_flag=1;
			}
			else if(attack_flag)
			{
				count1_crazy++;
				if(count1_crazy>4)
				{
					attack_flag=0;
					count1_crazy=0;
				}
			}
			
			hp1_last=hp1_now;
			time1_last=time1_now;

	}


}
	
int Speed_Decide(void)
{

	if(Robot.Chassis_Power.Chassis_Power_buffer>=Predict_MAXRemainPower)
	{
		Speed=6000;//6000
	}
	else if(Robot.Chassis_Power.Chassis_Power_buffer<=Predict_MINRemainPower)
	{
		Speed=4000;//4000
	}
	
	return Speed;
}

void xun_luo(void)
{
//	if(gdflag1!=1)
//	{
//			Run_Dir1=1;
//		 if(gdflag2!=1)
//   	{
//				Run_Dir1=2;
//		}
//	}
//	if(gdflag2!=1)
//	{
//			Run_Dir1=0;
//		 if(gdflag1!=1)
//   	{
//				Run_Dir1=2;
//		}
//	}
	static char Run_Dir3=0;
  if(round_set>=LEFT_ROUND)
   Run_Dir3 = 0;
  else if(round_set<= RIGHT_ROUND)
   Run_Dir3 =1;
	
	if(Run_Dir3==0)//Run_Dir3==0
  {
		 
		 SpeedSet=-3500;//-3500
     first_order_filter_cali(&chassis_cmd_slow_set_v, SpeedSet);
		 //ramp_calc( &chassis_cmd_slow_set_v1, SpeedSet);
		 
  }
	else if (Run_Dir3==1)//Run_Dir3==1
	{
		SpeedSet=3500;//3500���������
		first_order_filter_cali(&chassis_cmd_slow_set_v, SpeedSet);
		 //ramp_calc( &chassis_cmd_slow_set_v1, SpeedSet);
	}
	else if(Run_Dir3==2)//Run_Dir3==2
	{
		chassis_cmd_slow_set_v.out=0;
	}
	round_last=(moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2;
	active_flag=1;
}

//void gen_sui(void)
//{ 
//	if(Chassis.mode==0)//Chassis.modeΪ0��Ŀ��û�н���ä����Χ��Ϊ1��Ŀ�����ä��
//	{
//	    gdflag1=HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_5);
//      gdflag2=HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_6);
//      if(Chassis.flag==0)//Chassis.flagΪ0ֹͣ��Ϊ1����Ϊ2����
//      {
//				Run_Dir=2;
//      }
//		
//		if(gdflag1==1&&gdflag2==1&&Chassis.flag==2)
//    {			
//			 Run_Dir=0;
//    }
//			else if(gdflag1==1&&gdflag2==1&&Chassis.flag==1)
//    {			
//			 Run_Dir=1;
//    }

//		if((gdflag1==0&&Chassis.flag==2)||(gdflag2==0&&Chassis.flag==1))//һ���⵽Ŀ�꣬���ڱ����ڸò���ĩ��ʱֹͣ
//    {			
//			 Run_Dir=2;
//    }
//	  if(gdflag1==0&&Chassis.flag==1)//����һ�����£�Ŀ��ת�Ƶ���һ�࣬�ڱ�����һ�����Ŀ��
//		{
//			Run_Dir=1;
//		}
//		if(gdflag2==0&&Chassis.flag==2)
//		{
//			Run_Dir=0;
//		}
//	}
//	else if(Chassis.mode==1)//������Ұä�����ڱ���̨�߸���Ŀ�꣬���̱߷����˶�
//	{
//		Run_Dir=2;
//	}
//    if(Run_Dir==0)
//    {
//		   SpeedSet=-2000; 
//       first_order_filter_cali(&chassis_cmd_slow_set_v, SpeedSet);			
//    }
//	  else if(Run_Dir==1)
//			
//	  {
//		    SpeedSet=2000;
//		   first_order_filter_cali(&chassis_cmd_slow_set_v, SpeedSet);
//	  }
//		else if(Run_Dir==2)
//		{
//			chassis_cmd_slow_set_v.out = 0;
//		}
//	round_last=(moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2;
//	//Run_Dir=2;
//}

void gong_ji(void)
{ 
//	gdflag1=HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_5);
//  gdflag2=HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_6);
//  attack_flag_Decide();
	if(attack_flag==0)
	{	
	 auto_collimation();
	}
	else if(attack_flag==1)
	{
	 round_now=(moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2;
	 round_left=round_last+20;
	 round_right=round_last-20;
	 if(round_set<=RIGHT_ROUND)//(moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2<RIGHT_ROUND
	 {
		round_last=round_now + 20;
	 }
	 if(round_set>=LEFT_ROUND)//(moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2>LEFT_ROUND
	 {
		round_last=round_now - 20;
	 }
	 
   if(active_flag)
	 {
		 Run_Dir4=1;//Run_Dir4=1;
		 active_flag=0;
	 }
	 if(round_now>=round_left)
	 {
	  Run_Dir4 = 0;//����Run_Dir4=0;
	 }
	 else if(round_now<= round_right)
	 {
		 Run_Dir4 =1;	  //����Run_Dir4=1;
	 }
	}
		 
		 if(Run_Dir4==0)//Run_Dir4=0
     {
		   SpeedSet=-3500; //-3500
       first_order_filter_cali(&chassis_cmd_slow_set_v, SpeedSet);			
     }
	   else if(Run_Dir4==1)//Run_Dir4=1
	   {
		   SpeedSet=3500;//3500
		   first_order_filter_cali(&chassis_cmd_slow_set_v, SpeedSet);
	   }
		 else if(Run_Dir4==2) //Run_Dir4=2
		 {	
			chassis_cmd_slow_set_v.out = 0;
		 }
	  
	
}

void Crazy_mode(void)
{
	static char Run_Dir=0;// 0:��Զ������ 1:���������
	static char Run_Loc=0;// 0Ϊ�Ҳൽ�е�  1Ϊ��ൽ�е�
	static char Run_Dir_pre=1;
	
//	if(Robot.postion.yaw>0)Run_Loc=0;
//	else(Robot.postion.yaw<0)Run_Loc=1;
	if(Run_Loc==0)//���Ҳൽ�е㣬���ݵ�ǰ��λ���ж��˶�����
	{
		if((moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2>NearMidDist-30)//���뻹Ҫ���㣿
			Run_Dir=1;
		if((moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2<NearDist+10)
		  Run_Dir=0;
	}
	else
	{
		if((moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2<FarDist)
			Run_Dir=1;

		if((moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2>NearMidDist)
			Run_Dir=0;
	}
	
//�޶��ٶȣ���ʱ�������
		if(Run_Dir==1)
	{
		SpeedSet=-5000;
		first_order_filter_cali(&chassis_cmd_slow_set_v, SpeedSet);
	}
	else if(Run_Dir==0)
	{
		SpeedSet=5000;
		first_order_filter_cali(&chassis_cmd_slow_set_v, SpeedSet);
	}
	 
//	if(SpeedSet>=2000)
//		SpeedSet = 2000;
//		if(SpeedSet<=-2000)
//		SpeedSet = -2000;
	
		
//	�򷴷�����
if(Run_Dir_pre!=Run_Dir)
{
	Crazy_n--;
}
	Run_Dir_pre=Run_Dir;           //Run_Dir_pre=1;Crazy_n=0;

	if(Crazy_n<=0)//ͨ��ѭ����������0ʱ ��λ����
	{
		Crazy_n=SystemState.Time%8+1;//��ȡ��ǰʱ���8����������ѭ���������趨
		if(Run_Loc==0)
			Run_Loc=1;
		else
			Run_Loc=0;
	}
}

void Crazy_mode2(void)
	{
	if(RIGHT_ROUND<round_set && round_set<LEFT_ROUND)//RIGHT_ROUND<moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2<LEFT_ROUND
	{
		random_figure();
	}	
	if(round_set<=RIGHT_ROUND)//moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2<RIGHT_ROUND
	{
			Run_Dir5=1;//Run_Dir5=1;
		 if(round_set>=LEFT_ROUND)//moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2>LEFT_ROUND
   	{
				Run_Dir5=2;//Run_Dir5=2;
		}
	}
	if(round_set>LEFT_ROUND)//moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2>LEFT_ROUND
	{
			Run_Dir5=0;//Run_Dir5=0;
		 if(round_set<RIGHT_ROUND)//moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2<RIGHT_ROUND
   	{
				Run_Dir5=2;//Run_Dir5=2;
		}
	}
	
	if(Run_Dir5==0)
  {
		 
		 SpeedSet=-Speed_Decide();
     first_order_filter_cali(&chassis_cmd_slow_set_v, SpeedSet);
		 //ramp_calc( &chassis_cmd_slow_set_v1, SpeedSet);
		 
  }
	else if (Run_Dir5==1)
	{
		SpeedSet=Speed_Decide();
		first_order_filter_cali(&chassis_cmd_slow_set_v, SpeedSet);
		 //ramp_calc( &chassis_cmd_slow_set_v1, SpeedSet);
	}
	else if(Run_Dir5==2)
	{
		chassis_cmd_slow_set_v.out=0;
	}
	round_last=(moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2;
	active_flag=1;
}
//{
//	//gdflag1=HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_5);
//  //gdflag2=HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_6);
//	if(gdflag1==1 && gdflag2==1)
//	{
//		random_figure();
//	}
//	if(Run_Dir1==0)
//  {
//		 
//		 SpeedSet=-Speed_Decide();
//     first_order_filter_cali(&chassis_cmd_slow_set_v, SpeedSet);
//		 
//  }
//	else if (Run_Dir1==1)
//	{
//		SpeedSet=Speed_Decide();
//		first_order_filter_cali(&chassis_cmd_slow_set_v, SpeedSet);
//	
//	}
//	else if(Run_Dir1==2)
//	{
//		chassis_cmd_slow_set_v.out=0;
//	}
//	
//}

void Chassis_pid_init(void)
{
	
	for(int i=0; i<2; i++)
		{ 
	    PID_struct_init(&pid_3508_pos[i], POSITION_PID, 10000, 1000,
									1.5f,	0.0f,	20.0f);  // motos angular rate closeloop.pid:1.5,0.0,20.0
		}
	
		for(int i=0; i<2; i++)
		{ 
			PID_struct_init(&pid_3508_spd[i], POSITION_PID, 10000, 2000,
										1.5f,	0.01f,	0.1f	);  //4 motos angular rate closeloop.
		}
	
}

/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Chassis_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	���̿�������
	*	@retval	
****************************************************************************************/
void Chassis_Contrl_Task(void const * argument)
{
	static float  wheel[4]={0,0,0,0};
//	static uint8_t niuyao_flag=0;
	osDelay(4000);//��ʱ200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  Chassis.mode=1;
  chassis_disable_flg=0;
	Chassis_pid_init();
	chassis_cmd_slow_set_v.out = 0;
	
	srand(100);//[i8:598 1870 1855 847 1476]

	//��һ���˲�����б����������
    first_order_filter_init(&chassis_cmd_slow_set_v, CHASSIS_CONTROL_TIME, chassis_x_order_filter );//һ���˲���ʼ������
    //ramp_init(&chassis_cmd_slow_set_v1, CHASSIS_CONTROL_TIME, Max, Min);

	for(;;)
	{
	//HAL_GPIO_TogglePin(GPIOG, LED1_Pin|LED2_Pin|LED3_Pin);
	//HAL_GPIO_WritePin(GPIOG, LED1_Pin|LED2_Pin|LED3_Pin, 1);
    RefreshTaskOutLineTime(ChassisContrlTask_ON);
		round_set=(moto_chassis_get[0].round_cnt+moto_chassis_get[1].round_cnt)/2;//����Ȧ��ƽ��ֵ
    Crazy_flag_Decide();//ͨ����Ѫ�����������˶�ģʽ
		attack_flag_Decide();//ͨ���Ƿ��Ѫ�������ù���ģʽ
//    gdflag1=HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_5);
//    gdflag2=HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_6);
		      if(Robot.remainHp<100||Crazy_flag==1)
					{
			      Crazy_mode2();
					}
					else
					{
						if(Chassis.minipc_rx_state_flag==0)//���Ѫ������1000 ��û�з��ֵо�������ģʽ�ͷŵ�Ѳ��ģʽ
						{
								xun_luo();
						}
						else if (Chassis.minipc_rx_state_flag!=0)
						{
								gong_ji();
						}
						
	//					else if (Chassis.minipc_rx_state_flag==1||Chassis.minipc_rx_state_flag==3) 
	//          {
	//					    gen_sui();
	//					}	
					}
					 if (Chassis.stop_flag==0) //��ʽ����ȥ���öδ���
	          {
						    hard_brak();
						}	
					
		
		
					
		for(int i=0; i<2; i++)
			{		
			//		pid_calc(&pid_3508_pos[i], moto_chassis_get[i].total_angle, RoundSet);
				  pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, chassis_cmd_slow_set_v.out);//chassis_cmd_slow_set_v.out����һ���˲����
				
			}
      Current_set[0] = pid_3508_spd[0].pos_out;
			Current_set[1] = pid_3508_spd[1].pos_out;
		  power_limit(Current_set);
			

			if(chassis_disable_flg==1)
			{
				  Chassis_Motor_Disable(&hcan1);
			}
			else
			{  
				if(current_get.Current_Offset_num > 200)
				{
					Chassis_Motor(&hcan1,
					//Current_set[0],Current_set[1]);
       0,
         0);					
	       }					
			}
				
		
			osDelayUntil(&xLastWakeTime, CHASSIS_PERIOD);
  }
}




///***************************************************************************************
//**
//	*	@brief	hard_brak()
//	*	@param
//	*	@supplement	����ֹͣ����
//	*	@retval	
//****************************************************************************************/
//void hard_brak()
//{
//		moto_3508_set.dstVmmps_X=0;
//		moto_3508_set.dstVmmps_Y=0;
//		moto_3508_set.dstVmmps_W=0;
//}