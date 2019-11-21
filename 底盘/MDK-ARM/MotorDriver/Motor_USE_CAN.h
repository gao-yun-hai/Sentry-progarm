#ifndef __MOTOR_USE_CAN_H
#define __MOTOR_USE_CAN_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "can.h"
#include "communication.h "
#include "SystemState.h"

#define FILTER_BUF_LEN		5

typedef enum
{
	//add by langgo
	CAN_3508Moto_ALL_ID = 0x200,
	CAN_2006Moto_ID = 0x205,
	CAN_3508Moto1_ID = 0x201,
	CAN_3508Moto2_ID = 0x202,
		
}CAN_Message_ID;

typedef struct{
	int16_t	 			speed_rpm;
	int16_t  			real_current;
	int16_t  			given_current;
	uint8_t  			hall;
	uint16_t 			angle;				//abs angle range:[0,8191]
	uint16_t 			last_angle;	//abs angle range:[0,8191]
	uint16_t			offset_angle;
	int32_t				round_cnt;
	int32_t				total_angle;
	uint8_t				buf_idx;
	uint16_t			angle_buf[FILTER_BUF_LEN];
	uint16_t			fited_angle;	
	uint32_t			msg_cnt;
	int32_t      run_time;
	int32_t      cmd_time;
	int32_t      reverse_time;
	int32_t      REVE_time;
}moto_measure_t;

typedef struct {
	
	uint8_t mode;
	uint8_t flag;
	uint8_t	minipc_rx_state_flag;
	uint8_t minipc_rx_bopan_flag;
	uint8_t stop_flag;
  }Mode_Set;

extern moto_measure_t   moto_chassis_get[];
extern moto_measure_t   moto_dial_get;   //					_×¢ÊÍ
extern moto_measure_t   pit_get;
extern moto_measure_t   yaw_get;


void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
void Chassis_Motor( CAN_HandleTypeDef * hcan,int16_t iq1,int16_t iq2);
void Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value);
void get_moto_measure_3508(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_moto_measure_6623(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_moto_offset(moto_measure_t *ptr,CAN_HandleTypeDef * hcan);
void get_total_angle(moto_measure_t *p);
void Cloud_Platform_Motor_jiaozhun(CAN_HandleTypeDef * hcan);
void Cloud_Platform_Motor_Disable(CAN_HandleTypeDef * hcan);
void Chassis_Motor_Disable( CAN_HandleTypeDef * hcan);
void CAN_Send_YT( CAN_HandleTypeDef * hcan, int32_t time_three);
void CAN_GET_YK(RC_Ctl_t * RC , CAN_HandleTypeDef * hcan);
//void CAN_GET_YT(moto_measure_t * YT , CAN_HandleTypeDef * hcan);
void CAN_GET_YT(Mode_Set * Chassis , CAN_HandleTypeDef * hcan);
void CAN_GET_Error(SystemStateDef * Error , CAN_HandleTypeDef * hcan);
#endif
