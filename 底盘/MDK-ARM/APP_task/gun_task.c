#include "gun_task.h"
#include "SystemState.h"
#include "protocol.h"
#include "user_lib.h"
#include "Motor_USE_CAN.h"
Heat_Gun_t  ptr_heat_gun_t;

#define GUN_PERIOD  10
#define Mocha_PERIOD  1
#define BLOCK_TIME 2000
#define REVERSE_TIME 2000



pid_t pid_dial_pos  = {0};  //拨盘电机位置环
pid_t pid_dial_spd  = {0};	//拨盘电机速度环
int32_t time_three=0;
uint32_t cnt=0;

extern moto_measure_t   moto_chassis_get[];
extern moto_measure_t   moto_dial_get;   //					_注释
extern moto_measure_t   pit_get;
extern moto_measure_t   yaw_get;
extern Mode_Set Chassis;
extern CAN_HandleTypeDef hcan1;

#define my_abs(x) ((x)>0?(x):-(x)) //ABS宏定义，取绝对值，
//EXP1? EXE2: EXP3,在计算EXP1之后，如果数值为True，则计算EXP2，并将结果作为整个表达式的数值；如果E XP1的值为Flase，则计算EXP3

/* 内部函数原型声明----------------------------------------------------------*/
void Gun_Pid_Init()
{
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000,
									0.2f,0.0f,	2.0f);  
		//pid_pos[i].deadband=500;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 9000, 6000,
									2.5f,	0.1f,	0.1f	);  
		//pid_dial_spd.deadband=10;//2.5f,	0.03f,	1.0f	

//    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //电源引脚 _待续
} 

//void time_count(void)
//{
//	static fp32 time2_now=0;
//	static fp32 time2_last=0;
//	time2_now= HAL_GetTick();
//	if(time2_now-time2_last>180000)
//	{
//			time_three=1;
//	}

//}


	


/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	枪口热量限制任务
	*	@retval	
****************************************************************************************/
void Gun_Task(void const * argument)
{ 

	osDelay(100);
	portTickType xLastWakeTime;//??????
	xLastWakeTime = xTaskGetTickCount();//??????

	Gun_Pid_Init();
	
//	uint8_t motor_stop_flag=0;
	static int32_t set_angle = 0;
	int32_t set_speed = 0;
	static uint8_t set_cnt = 0;
  static uint8_t block_flag;
  
  block_flag=1;
  /*设定发弹*/
	for(;;)
 {
		
		 RefreshTaskOutLineTime(GunTask_ON);
	   
//		 if(Robot.game_progress==4)//5.23新添加
//		 {

//			 time_three=1;
//			 CAN_Send_YT(&hcan2,time_three);
//			 
//		 }
//		 if(Robot.game_progress!=4)
//		 {

//			 time_three=0;
//			 CAN_Send_YT(&hcan2,time_three);
//			 
//		 }
		 
//       if(Chassis.stop_flag=)
//			 {
//				 time_three=1;
//			 CAN_Send_YT(&hcan2,time_three);
//			 }
//			 else 
//			 {
//				 time_three=0;
//			 CAN_Send_YT(&hcan2,time_three);
//			 }
	   
//	   CAN_Send_YT(&hcan2,time_three);
	 
    //if(Chassis.minipc_rx_state_flag == 0||Chassis.stop_flag==0||Chassis.minipc_rx_bopan_flag == 0||Robot.heat.shoot_17_heat>480)//
		 if(Chassis.minipc_rx_state_flag == 0||Chassis.stop_flag==0||Chassis.minipc_rx_bopan_flag == 0||Robot.heat.shoot_17_heat>480)
		{
//			if(cnt==0&&Robot.game_progress==4)
//			{
//			 if(Robot.heat.shoot_17_speed==0)
//			 { 
//			  ptr_heat_gun_t.sht_flg = 2;
//			 }
//			 else 
//			 {
//				ptr_heat_gun_t.sht_flg = 0;
//				cnt++;
//			 }
//		  }
//			else
//			{
			ptr_heat_gun_t.sht_flg = 0;
			//}
	 }
		else
		{
			if(block_flag == 1 && Chassis.minipc_rx_state_flag == 3 && Chassis.minipc_rx_bopan_flag )//再加一个手动判断位
		 {
			ptr_heat_gun_t.sht_flg = 1;
		 }
		  if(block_flag == 1 && Chassis.minipc_rx_state_flag == 2 && Chassis.minipc_rx_bopan_flag)//再加一个手动判断位，扫瞄到敌车，在攻击范围内，连发模式
		 {
			ptr_heat_gun_t.sht_flg = 2;
		 }
		 if((block_flag == 1 )&& (Chassis.minipc_rx_state_flag == 2)&&(Robot.heat.shoot_17_heat>450) && Chassis.minipc_rx_bopan_flag)//再加一个手动判断位，扫瞄到敌车，在攻击范围内，连发模式
		 {
			ptr_heat_gun_t.sht_flg = 1;
		 }
	  }
 /*判断拨盘是否转到位置*/		
	 if(block_flag==1)
			{
				  moto_dial_get.REVE_time=GetSystemTimer();//不反转计时
		     /*判断拨盘是否转到位置*/			
					if(my_abs(moto_dial_get.round_cnt) >=5*set_cnt)
						{
							moto_dial_get.round_cnt=0;
							moto_dial_get.offset_angle=moto_dial_get.angle;
							moto_dial_get.total_angle=0;	
							moto_dial_get.run_time=GetSystemTimer();
						}
						else
						{
								if( my_abs(moto_dial_get.run_time-moto_dial_get.cmd_time)>BLOCK_TIME )//堵转判定
								{
									block_flag=0;	
								}
						}
					}
								
		if(block_flag==0)   ptr_heat_gun_t.sht_flg=10;//反转
			
		
			
 /*判断发射模式*/
	 if(Mocha_Bopan)
	 {
    switch(ptr_heat_gun_t.sht_flg)
    {
			case 0://停止
				{
					set_angle=0;
					set_speed=0;
					set_cnt=0;
				moto_dial_get.cmd_time=GetSystemTimer();
				}break;

//      case 1://按键单发模式
//				{
//					moto_dial_get.cmd_time=GetSystemTimer();
//					set_cnt=1;
//					set_speed=-5000;
////					set_angle=-42125*set_cnt;//36859.5=(45/360)*36*8191
////					
////					/*pid位置环*/
////					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
////					set_speed=pid_dial_pos.pos_out;

//				}break;
     case 3://固定角度模式
        {
					moto_dial_get.cmd_time=GetSystemTimer();
					set_cnt=1;
					set_angle=-45000*set_cnt;
				
					/*pid位置环*/
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
					set_speed=pid_dial_pos.pos_out;
        }break;
		case 1://单发模式
				{ 
					moto_dial_get.cmd_time=GetSystemTimer();
					set_speed=-1900;
					set_cnt=1;
					
				}break;
			case 2://连发模式
				{ 
					moto_dial_get.cmd_time=GetSystemTimer();
					set_speed=-4200;//15.5hz  4000/36(减速比）/60（分转化为秒）*8（每圈8发）=14.8Hz
					set_cnt=1;
					
				}break;
			case 10://反转
				{
						
					moto_dial_get.reverse_time=GetSystemTimer();		
						
					if( moto_dial_get.reverse_time-moto_dial_get.REVE_time > REVERSE_TIME )//反转设定
					{
						block_flag=1;
						moto_dial_get.round_cnt=0;
						moto_dial_get.offset_angle=moto_dial_get.angle;
						moto_dial_get.total_angle=0;	
						set_cnt=0;
					}else 
					{
						
						set_speed=1000;
			
					}
				}  
		default :break;
   } 
     /*速度环*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,set_speed);
     /*驱动拨弹电机*/
		 Allocate_Motor(&hcan1,pid_dial_spd.pos_out);
//		 Chassis.minipc_rx_state_flag=0;
//		 set_speed=0;
     osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	
	 
  }
 }
}


//void Mocha_Task(void const *argument)
//{
//	
//	osDelay(500);
//	portTickType xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();
//	
//	for(;;)
//	{
////		IMU_Get_Data();
//		
//		if(Mocha_Bopan) 
//	 {
//		
//   switch(ptr_heat_gun_t.sht_flg)
//    {
//			case 0://停止
//			{
////        TIM5_PWM_Init(0,0);
//			}break;
//			
//			case 1://单发模式
//      {
//			  TIM5_PWM_Init(120,120);
//      }break;
//			
//			case 2://固定角度模式
//      {
//				TIM5_PWM_Init(120,120);
//      }break;
//      case 3://连发模式
//      {
//				 TIM5_PWM_Init(120,120);
//      }break;
//			case 10://反转
//			{
//				
//			}break;
//			default :break;
//			
//		}
//	}
//	 osDelayUntil(&xLastWakeTime,Mocha_PERIOD);
//		
//	}
//}






