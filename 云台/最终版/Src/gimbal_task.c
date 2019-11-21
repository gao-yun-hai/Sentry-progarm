#include "gimbal_task.h"
#include "tim.h"
#include "SystemState.h"
#include "math.h"
ramp_function_source_t fric1_ramp;
ramp_function_source_t fric2_ramp;

#define SHOOT_FRIC_PWM_ADD_VALUE    5.0f

extern uint8_t h;
Pos_Set  yaw_set;
Pos_Set  pit_set;
Mode_Set Gimbal;
Mode_Set Chassis;
Mode_Set pit;
int8_t gimbal_disable_flg;
float SpeedSet=0;
static  int16_t Yaw_Current_Value = 0;
static  int16_t Pitch_Current_Value = 0;

float current_total_angle=0;
static float llast_angle=0;
float llastpit_angle=0;
int a;
#define GIMBAL_PERIOD 10

//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.01
#define CHASSIS_ACCEL_X_NUM 0.1666666667f

const static fp32 pitch_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
first_order_filter_type_t pitch_cmd_slow_set_v;

pid_t pid_yaw       = {0};  //yaw轴位置环
pid_t pid_yaw_jy901 = {0};  //外接陀螺仪 /*目前只用于位置环*/ 
pid_t pid_pit       = {0};	//pit轴位置环
pid_t pid_yaw_spd   = {0};	//yaw轴速度环
pid_t pid_pit_spd   = {0};	//pit轴速度环
pid_t pid_yaw_spd1   = {0};	//yaw轴速度环 巡航模式
pid_t pid_pit_spd1   = {0};	//pit轴速度环 巡航模式
pid_t pid_yaw_jy901_spd = {0};
pid_t pid_pit_jy901 = {0};
pid_t pid_pit_jy901_spd = {0};
/**                                                           //待续
	**************************************************************
	** Descriptions: 云台pid初始化
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/

void gimbal_pid_init(void)
{

	PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.5f); 
	PID_struct_init(&pid_pit_spd, POSITION_PID, 30000, 30000,
									33.0f, 0.00001f, 0.5f);//35.0f,	3.0f,	0.0f	);
//  PID_struct_init(&pid_pit_jy901_spd, POSITION_PID, 5000, 1000,
//                 3.0f, 0.0f, 0.0f );
//	
  PID_struct_init(&pid_yaw_spd, POSITION_PID, 10000, 1000,
										4.0f,	0.0f,	0.0f	);  //4 motos angular rate closeloop.
	
  /* yaw axis motor pid parameter */
	 PID_struct_init(&pid_yaw, POSITION_PID, 5000, 1000,
                  0.5f, 0.0f, 0.0f); 
//	 PID_struct_init(&pid_yaw_spd, POSITION_PID, 5000, 1000,
//                  2.0f, 0.0f, 0.0f );
//									
//	 PID_struct_init(&pid_yaw_jy901, POSITION_PID, 5000, 500,
//                 3.0f, 0.0f, 3.0f);//1.5f, 0.0f, 2.0f 
//	 PID_struct_init(&pid_yaw_jy901_spd, POSITION_PID, 5000, 500,
//                  3.0, 0.01f, 5.0f );	//3.0, 0.01f, 5.0f							
//	
	PID_struct_init(&pid_pit_spd1, POSITION_PID, 30000, 30000,
										22.0f, 0.01f, 0.5f);//10.0f,	3.0f,	0.0f	);
	PID_struct_init(&pid_yaw_spd1, POSITION_PID, 10000, 2000,
										4.0f,	0.0f,	0.0f	);  //4 motos angular rate closeloop.
}


extern uint8_t set_flag;
void xun_han(void)
{
		set_flag = 0;
	 yaw_set.expect=-500;

	   //判断反向
		if(pit_get.total_angle>=-30)
		pit.flag = 1;
	  else if(pit_get.total_angle<-400)
		pit.flag =0;
	
		//加减速过程
	  if(pit.flag==1)

		{
		 SpeedSet=-250; 
     first_order_filter_cali(&pitch_cmd_slow_set_v, SpeedSet);
		 
    }
	  else if(pit.flag==0)
//		SpeedSet+=5;
		{
		SpeedSet=150;
		first_order_filter_cali(&pitch_cmd_slow_set_v, SpeedSet);
	  }
		
		//速度环控制
	  pid_calc(&pid_yaw_spd1,yaw_get.speed_rpm,yaw_set.expect);
	  Yaw_Current_Value= (pid_yaw_spd1.pos_out);
		
		pid_calc(&pid_pit_spd1,pit_get.speed_rpm,pitch_cmd_slow_set_v.out);
		Pitch_Current_Value=(pid_pit_spd1.pos_out);

		//保留此时的角度值，便进入其它模式使用
		llast_angle=yaw_get.total_angle;
	  llastpit_angle=pit_get.total_angle;
}

void gong_ji(void)
{
	current_total_angle=yaw_get.total_angle-llast_angle;
	
		if(pit_set.expect>=20)
		pit_set.expect =20;
	  else if(pit_set.expect<-3500)
		pit_set.expect =-3500;
		
	 pid_calc(&pid_yaw, current_total_angle,yaw_set.expect);
	 pid_calc(&pid_yaw_spd,yaw_get.speed_rpm,pid_yaw.pos_out);
	 Yaw_Current_Value= (pid_yaw_spd.pos_out);
	
		pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
		pid_calc(&pid_pit_spd,pit_get.speed_rpm,(pid_pit.pos_out));
		Pitch_Current_Value=(pid_pit_spd.pos_out);
			//a=(yaw_get.total_angle/8191*360/(3591/187)/2)%360;
        a=fmod((yaw_get.total_angle*0.001144),360);
		//判断枪口的朝向，使底盘运动
		if((a<-10)&&(a>-170))//逆时针为负值。
			Chassis.flag=2;//往右运动
		else if((a<-190)&&(a>-350)) Chassis.flag=1;//往右运动
		else  Chassis.flag=0;//停止
}





/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	云台电机控制
	*	@retval	
****************************************************************************************/

void Gimbal_Contrl_Task(void const * argument)
{
	yaw_set.expect = 0; 
	pit_set.expect = 0;
	yaw_set.mode   = 0;

//	CAN2rxFLAG.Game_time=0;
	Gimbal.flag=0;
  Gimbal.mode=0;
  minipc_rx.state_flag=0;
	minipc_rx.bopan_flag=0;
	Pitch_Current_Value=0;
	Yaw_Current_Value=0;
	
	gimbal_pid_init();
	
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();	
	
	first_order_filter_init(&pitch_cmd_slow_set_v, CHASSIS_CONTROL_TIME, pitch_order_filter );
	for(;;)		
    {  
	   
		  	RefreshTaskOutLineTime(GimbalContrlTask_ON);
      ramp_calc(&fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

      if(fric1_ramp.out == fric1_ramp.max_value)
        {
            ramp_calc(&fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        }
     if(fric2_ramp.out== fric2_ramp.max_value)
        {
             minipc_rx.bopan_flag=1;
        }
      
        TIM5_PWM_Init(fric1_ramp.out,fric2_ramp.out);

			 if(minipc_rx.state_flag!=0)
			{	
				 gong_ji();
					if(minipc_rx.state_flag==3){ TIM5_PWM_Init(120,120);
					}			
				 else { TIM5_PWM_Init(115,115);
					}		
				 
			}
			else{
		
				 xun_han();//巡逻模式
			
			}	
//Cloud_Platform_Motor(&hcan1,0,0);
 Cloud_Platform_Motor(&hcan1,Yaw_Current_Value,Pitch_Current_Value);
	
//			
//		 else{
//			 Cloud_Platform_Motor(&hcan1,0,0);
//			 TIM5_PWM_Init(100,100);
//		 }
		osDelay(5);
	 }
 }
 
 


