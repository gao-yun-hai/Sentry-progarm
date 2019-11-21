/* 包含头文件----------------------------------------------------------------*/
#include "data_pro_task.h"
#include "SystemState.h"
/* 内部宏定义----------------------------------------------------------------*/
#define press_times  20
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\
//extern osSemaphoreId Dubs_BinarySemHandle;
/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/

#define REMOTE_PERIOD 1 

/* 外部变量声明--------------------------------------------------------------*/

/* 调用的外部函数原型声明------------------------------------------------------
	uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
	uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
------------------------------------------------------------------------------
*/
/* 内部变量------------------------------------------------------------------*/
int16_t XY_speed_max = 6000;
int16_t XY_speed_min = -6000; 
int16_t W_speed_max = 3000;
int16_t W_speed_min = -3000; 
uint8_t press_counter;
uint8_t shot_anjian_counter=0;
uint8_t shot_frequency = 20;
int8_t chassis_gimble_Mode_flg;
//volatile float remain_power=0.0;   //底盘功率 _待续
//float power; 				 //底盘功率 _测试

//float chassis_Current; 
//float	 chassis_Volt; 
/* 内部函数原型声明-----------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	RemoteControlProcess()
	*	@param
	*	@supplement	与遥控器进行对接，对遥控器的数据进行处理，实现对底盘、云台、发射机构的控制
	*	@retval	
****************************************************************************************/
void RemoteControlProcess()  
{

	         if(Chassis.mode==1)
					 {	
							moto_3508_set.dstVmmps_X=((RC_Ctl.rc.ch0-0x400)*5);
							moto_3508_set.dstVmmps_Y=((RC_Ctl.rc.ch1-0x400)*5);
					 }else{
									moto_3508_set.dstVmmps_W=((RC_Ctl.rc.ch0-0x400)*5);
									moto_3508_set.dstVmmps_Y=((RC_Ctl.rc.ch1-0x400)*5);
							 }


					

									if(RC_Ctl.rc.s1==1)
									{
										Chassis.mode=1;
									}
                  else if(RC_Ctl.rc.s1==2)
                  {
										Chassis.mode=3;
                  }
                  else
                  {
                    Chassis.mode=1;
                  }
								
							
					
}

/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	对键鼠的数据进行处理
	*	@retval	
****************************************************************************************/
void MouseKeyControlProcess()
{
	
	
	if(RC_Ctl.key.v & 0x10 )//
					{

						XY_speed_max = 3000;//(NORMAL_SPEED_MAX)*3.5;
						XY_speed_min = -3000;//(NORMAL_SPEED_MIN)*3.5;
					}
			
					if(RC_Ctl.key.v & 0x01)                       moto_3508_set.dstVmmps_Y -= ACC_SPEED;//按下W键
					else if(RC_Ctl.key.v & 0x02)                  moto_3508_set.dstVmmps_Y += ACC_SPEED;//按下S键
					else{  
							 	if(moto_3508_set.dstVmmps_Y>-DEC_SPEED&&moto_3508_set.dstVmmps_Y<DEC_SPEED) 	 moto_3508_set.dstVmmps_Y = 0;
								if(moto_3508_set.dstVmmps_Y>0) 	                   moto_3508_set.dstVmmps_Y -= DEC_SPEED;
								if(moto_3508_set.dstVmmps_Y<0) 		                 moto_3508_set.dstVmmps_Y += DEC_SPEED;
					}


					if(RC_Ctl.key.v & 0x04)                        moto_3508_set.dstVmmps_X += ACC_SPEED; //按下D键
					else if(RC_Ctl.key.v & 0x08)    		           moto_3508_set.dstVmmps_X -= ACC_SPEED;//按下A键
					else{
									if(moto_3508_set.dstVmmps_X>-DEC_SPEED&&moto_3508_set.dstVmmps_X<DEC_SPEED) 		moto_3508_set.dstVmmps_X = 0;		
									if(moto_3508_set.dstVmmps_X>0) 	                   moto_3508_set.dstVmmps_X -= DEC_SPEED;
									if(moto_3508_set.dstVmmps_X<0) 		                 moto_3508_set.dstVmmps_X += DEC_SPEED;
					}

					
					
					//云台跟随底盘 c键 
					if(RC_Ctl.key.v & 0x2000)
					{
						moto_3508_set.flag = !moto_3508_set.flag;
					}
								
}



/* 任务主体部分 -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	遥控数据接收及处理任务
	*	@retval	
****************************************************************************************/
void Remote_Data_Task(void const * argument)
{
	uint32_t NotifyValue;
	
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	
	
	for(;;)
	{
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14); //GRE_main
			
			RefreshTaskOutLineTime(RemoteDataTask_ON);
				switch(RC_Ctl.rc.s2)
				{
					case 1: RemoteControlProcess();break; 
					case 2: hard_brak();break;
					case 3: MouseKeyControlProcess();break;
					default :break;
				}					
				
			VAL_LIMIT(moto_3508_set.dstVmmps_X, XY_speed_min, XY_speed_max);
			VAL_LIMIT(moto_3508_set.dstVmmps_Y, XY_speed_min, XY_speed_max);	
			VAL_LIMIT(moto_3508_set.dstVmmps_W, W_speed_min, W_speed_max);
				
       press_counter++;
		
				if(Remote.Mode)//遥控器掉线
				{
						Remote_Disable();
				}
		
			osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}

/***************************************************************************************
**
	*	@brief	JSYS_Task(void const * argument)
	*	@param
	*	@supplement	裁判系统数据处理任务
	*	@retval	
****************************************************************************************/
//extern DMA_HandleTypeDef hdma_usart6_rx;
//void Referee_Data_Task(void const * argument)
//{
//	    tFrame   *Frame;
//	
//	    uint32_t NotifyValue;
////		  uint8_t buff[USART6_RX_NUM];
//	    //USART6_RX_NUM=SizeofReferee;
//	for(;;)
//	{
//    
//			  NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
//    
//    if(NotifyValue==1)
//		{
//			  NotifyValue=0;
//			
//				HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1); //GRE_H
//        uint8_t *buff=USART6_RX_DATA;
//			for(uint8_t i=0;i<USART6_RX_NUM;i++)
//			{
//					if(buff[i]==0xA5)
//					{
//					   Frame = (tFrame *)&buff[i];
//						
//					    if( verify_crc16_check_sum((uint8_t *)Frame, Frame->FrameHeader.DataLength + sizeof(tFrameHeader) + sizeof(tCmdID) + sizeof(Frame->CRC16))
//		             && verify_crc8_check_sum((uint8_t *)Frame,sizeof(tFrameHeader)))
//								 {
//									 if(Frame->CmdID==game_robot_state)
//									 {
//											Robot.id = Frame->Data.game_robot_state.robot_id;//id号
//											Robot.level = Frame->Data.game_robot_state.robot_level;//等级
//                      Robot.remainHp = Frame->Data.game_robot_state.remain_HP;//剩余血量
//                      Robot.maxHp  = Frame->Data.game_robot_state.max_HP;//最大血量
//                      Robot.heat.shoot_17_cooling_rate = Frame->Data.game_robot_state.shooter_heat0_cooling_rate;//17每秒冷却值
//                     Robot.heat.shoot_17_cooling_limit = Frame->Data.game_robot_state.shooter_heat0_cooling_limit;//17；冷却上限
//                      Robot.heat.shoot_42_cooling_rate = Frame->Data.game_robot_state.shooter_heat1_cooling_rate;//
//                      Robot.heat.shoot_42_cooling_limit = Frame->Data.game_robot_state.shooter_heat1_cooling_limit;//
//											
//									 }
//									 if(Frame->CmdID == power_heat_data)
//									 {
//											Robot.heat.shoot_17_heat = Frame->Data.power_heat_data.shooter_heat0;//17枪口热量
//                      Robot.heat.shoot_42_heat = Frame->Data.power_heat_data.shooter_heat1;
//                     
//                      Robot.Chassis_Power.Chassis_Current = Frame->Data.power_heat_data.chassis_current;//电流
//                      Robot.Chassis_Power.chassis_Power = Frame->Data.power_heat_data.chassis_power;//功率
//                      Robot.Chassis_Power.Chassis_Power_buffer = Frame->Data.power_heat_data.chassis_power_buffer;//缓冲
//                      Robot.Chassis_Power.Chassis_Volt = Frame->Data.power_heat_data.chassis_volt;//电压
//									 }
//									 if(Frame->CmdID==game_robot_pos)
//									 {
//                     
//										 Robot.postion.x = Frame->Data.game_robot_pos.x;
//                     Robot.postion.y = Frame->Data.game_robot_pos.y;
//                     Robot.postion.z = Frame->Data.game_robot_pos.z;
//                     Robot.postion.yaw = Frame->Data.game_robot_pos.yaw;
//                    
//									 }
//									 if(Frame->CmdID==game_state)
//									 {
//										Robot.state=Frame->Data.game_state.stage_remain_time;
//									 }
//                   
//                   if(Frame->CmdID == buff_musk)
//                   {
//                     Robot.buff = Frame->Data.buff_musk.power_rune_buff;
//                     
//                   }
//                   if( Frame->CmdID == shoot_data)
//                   {
//                     if(Frame->Data.shoot_data.bullet_type == 1) //17
//                     {
//                     Robot.heat.shoot_17_freq = Frame->Data.shoot_data.bullet_freq;
//                     Robot.heat.shoot_17_speed = Frame->Data.shoot_data.bullet_speed;
//                     }
//                     if(Frame->Data.shoot_data.bullet_type == 2)//42
//                     {
//                     Robot.heat.shoot_42_freq = Frame->Data.shoot_data.bullet_freq;
//                     Robot.heat.shoot_42_speed = Frame->Data.shoot_data.bullet_speed;
//                     }
//                   }
//											 i=i+sizeof(Frame);
//								}
//					}
//				
//			}
//			
//		}
//		osDelay(10);
//	 }

// }


