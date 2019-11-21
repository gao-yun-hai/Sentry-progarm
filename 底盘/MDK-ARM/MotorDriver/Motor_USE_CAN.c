/*******************************************************************************
*                     ��Ȩ���� (C), 2017-,NCUROBOT
********************************************************************************
* �� �� ��   : Motor_USE_CAN.c
* �� �� ��   : ����
* ��    ��   : NCURM
* ��������   : 2018��7��
* ����޸�   :
* ��������   : �����ģ����ʹ��CAN���п��Ƶĵ��
* �����б�   :
*ʹ��CANͨѶ�ĵ������̨���   		 ���̵��	 	 	  �������
*				 	��Ӧ�ͺţ� c620						3508					 C2000
*�ӿں�����
*					Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch)
*					Chassis_Motor( CAN_HandleTypeDef * hcan,
*								  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Motor_USE_CAN.h"
#include "SystemState.h"
/* �ڲ��Զ����������� --------------------------------------------------------*/

/* �ڲ��궨�� ----------------------------------------------------------------*/

/* ���������Ϣ����-----------------------------------------------------------*/

/* �ڲ���������---------------------------------------------------------------*/

/* �ⲿ�������� --------------------------------------------------------------*/
/*******************Ħ���ֵ���͵��̵���Ĳ�������***************************/
moto_measure_t   moto_chassis_get[2] = {0};//4 �� 3508
moto_measure_t   moto_dial_get = {0};  //c2006
moto_measure_t   pit_get;
moto_measure_t   yaw_get;
/* �ⲿ����ԭ������ ----------------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/
//Ϊcan���ͷֱ𴴽����棬��ֹ���ڷ��͵�ʱ����ֻ��һ���ڴ���໥����
static CanTxMsgTypeDef  Cloud_Platform_Data;
static CanTxMsgTypeDef	 Chassis_Motor_Data;
static CanTxMsgTypeDef  Allocate_Motor_Data;
static CanTxMsgTypeDef  CANSend_YT;
/* ����ԭ������ ----------------------------------------------------------*/

/**
	**************************************************************
	** Descriptions: ��̨�����������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**					yaw:yaw�����ֵ
	**				pitch:pitch����ֵ
	** Output: NULL
	**************************************************************
**/
//void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch)
//{
//		Cloud_Platform_Data.StdId = 0x1FF;
//		Cloud_Platform_Data.IDE = CAN_ID_STD;
//		Cloud_Platform_Data.RTR = CAN_RTR_DATA;
//		Cloud_Platform_Data.DLC = 0X08;
//		
//		Cloud_Platform_Data.Data[0] = yaw>>8;
//		Cloud_Platform_Data.Data[1] = yaw;
//		Cloud_Platform_Data.Data[2] = pitch>>8;
//		Cloud_Platform_Data.Data[3] = pitch;
//		Cloud_Platform_Data.Data[4] = 0x00;
//		Cloud_Platform_Data.Data[5] = 0x00;
//		Cloud_Platform_Data.Data[6] = 0x00;
//		Cloud_Platform_Data.Data[7] = 0x00;

//  	hcan->pTxMsg = &Cloud_Platform_Data;
//		HAL_CAN_Transmit(hcan,0);
//}

/**
	**************************************************************
	** Descriptions: ��̨���У׼����
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**					
	**				
	** Output: NULL
	**************************************************************
**/
//void Cloud_Platform_Motor_jiaozhun(CAN_HandleTypeDef * hcan)
//{
//	
//		Cloud_Platform_Data.StdId = 0x3F0;
//		Cloud_Platform_Data.IDE = CAN_ID_STD;
//		Cloud_Platform_Data.RTR = CAN_RTR_DATA;
//		Cloud_Platform_Data.DLC = 0X08;
//		
//		Cloud_Platform_Data.Data[0] = 'c' ;
//		Cloud_Platform_Data.Data[1] = 0x00;
//		Cloud_Platform_Data.Data[2] = 0x00;
//		Cloud_Platform_Data.Data[3] = 0x00;
//		Cloud_Platform_Data.Data[4] = 0x00;
//		Cloud_Platform_Data.Data[5] = 0x00;
//		Cloud_Platform_Data.Data[6] = 0x00;
//		Cloud_Platform_Data.Data[7] = 0x00;

//  	hcan->pTxMsg = &Cloud_Platform_Data;
//		HAL_CAN_Transmit(hcan,10);
//}

/**
	**************************************************************
	** Descriptions: ��̨���ʧ�ܺ���
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**					
	**				
	** Output: NULL
	**************************************************************
**/
//void Cloud_Platform_Motor_Disable(CAN_HandleTypeDef * hcan)
//{
//	
//		Cloud_Platform_Data.StdId = 0x1FF;
//		Cloud_Platform_Data.IDE = CAN_ID_STD;
//		Cloud_Platform_Data.RTR = CAN_RTR_DATA;
//		Cloud_Platform_Data.DLC = 0X08;
//		
//		Cloud_Platform_Data.Data[0] = 0x00;
//		Cloud_Platform_Data.Data[1] = 0x00;
//		Cloud_Platform_Data.Data[2] = 0x00;
//		Cloud_Platform_Data.Data[3] = 0x00;
//		Cloud_Platform_Data.Data[4] = 0x00;
//		Cloud_Platform_Data.Data[5] = 0x00;
//		Cloud_Platform_Data.Data[6] = 0x00;
//		Cloud_Platform_Data.Data[7] = 0x00;

//  	hcan->pTxMsg = &Cloud_Platform_Data;
//		HAL_CAN_Transmit(hcan,10);
//}

/**
	**************************************************************
	** Descriptions: ���̵����������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**					iqn:��n�����̵���ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									   int16_t iq1,int16_t iq2)
{
			Chassis_Motor_Data.DLC = 0x08;
			Chassis_Motor_Data.IDE = CAN_ID_STD;
			Chassis_Motor_Data.RTR = CAN_RTR_DATA;
			Chassis_Motor_Data.StdId = 0x200;

			Chassis_Motor_Data.Data[0]=iq1>>8;
			Chassis_Motor_Data.Data[1]=iq1;
			Chassis_Motor_Data.Data[2]=iq2>>8;
			Chassis_Motor_Data.Data[3]=iq2;
			Chassis_Motor_Data.Data[4]=0;
			Chassis_Motor_Data.Data[5]=0;
			Chassis_Motor_Data.Data[6]=0;
			Chassis_Motor_Data.Data[7]=0;
	
			hcan->pTxMsg = &Chassis_Motor_Data;
			HAL_CAN_Transmit(hcan,0);
}

/**
	**************************************************************
	** Descriptions: ���̵��ʧ�ܺ���
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**					iqn:��n�����̵���ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor_Disable( CAN_HandleTypeDef * hcan)
{
			Chassis_Motor_Data.DLC = 0x08;
			Chassis_Motor_Data.IDE = CAN_ID_STD;
			Chassis_Motor_Data.RTR = CAN_RTR_DATA;
			Chassis_Motor_Data.StdId = 0x200;

			Chassis_Motor_Data.Data[0]=0x00;
			Chassis_Motor_Data.Data[1]=0x00;
			Chassis_Motor_Data.Data[2]=0x00;
			Chassis_Motor_Data.Data[3]=0x00;
			Chassis_Motor_Data.Data[4]=0x00;
			Chassis_Motor_Data.Data[5]=0x00;
			Chassis_Motor_Data.Data[6]=0x00;
			Chassis_Motor_Data.Data[7]=0x00;
	
			hcan->pTxMsg = &Chassis_Motor_Data;
			HAL_CAN_Transmit(hcan,5);
}	
/**
	**************************************************************
	** Descriptions: ���������������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN1
	**				value:��������ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Allocate_Motor(CAN_HandleTypeDef * hcan,int16_t value)
{

			Allocate_Motor_Data.DLC = 0x08;
			Allocate_Motor_Data.IDE = CAN_ID_STD;
			Allocate_Motor_Data.RTR = CAN_RTR_DATA;
			Allocate_Motor_Data.StdId = 0x1FF;

			Allocate_Motor_Data.Data[0]=value>>8;
			Allocate_Motor_Data.Data[1]=value;
			Allocate_Motor_Data.Data[2]=0;
			Allocate_Motor_Data.Data[3]=0;
			Allocate_Motor_Data.Data[4]=0;
			Allocate_Motor_Data.Data[5]=0;
			Allocate_Motor_Data.Data[6]=0;
			Allocate_Motor_Data.Data[7]=0;
	
			hcan->pTxMsg = &Allocate_Motor_Data;
			HAL_CAN_Transmit(hcan,0);
}
/**                                                           //����
	**************************************************************
	** Descriptions: ��ȡCANͨѶ��6623����ķ���ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
//void get_moto_measure_6623(moto_measure_t *ptr,CAN_HandleTypeDef * hcan)
//{
//	/*BUG!!! dont use this para code*/

//	ptr->last_angle = ptr->angle;
//	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
//	ptr->real_current  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
//	ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5]);
//	ptr->speed_rpm = ptr->real_current;
////	ptr->hall = hcan->pRxMsg->Data[6];
//	
//	if(ptr->angle - ptr->last_angle > 4096)
//		ptr->round_cnt --;
//	else if (ptr->angle - ptr->last_angle < -4096)
//		ptr->round_cnt ++;
//	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
//}
/**                                                           //����
	**************************************************************
	** Descriptions: ��ȡCANͨѶ��3508����ķ���ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
void get_moto_measure_3508(moto_measure_t *ptr,CAN_HandleTypeDef * hcan)
{
	/*BUG!!! dont use this para code*/

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->speed_rpm  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
	ptr->real_current = (int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5]);
	ptr->hall = hcan->pRxMsg->Data[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}
/**
	**************************************************************
	** Descriptions:��ȡ�������ֵ��ƫ��ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr,CAN_HandleTypeDef * hcan)
{
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 |hcan->pRxMsg->Data[1]) ;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
	**************************************************************
	** Descriptions: ��ȡ������ܽǶ�ֵ
	** Input: 	
	**			   *P:��Ҫ��ȡ�ܽǶ�ֵ�ĵ�ַ
	**				
	** Output: NULL
	**************************************************************
**/
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//?????
		res1 = p->angle + 8192 - p->last_angle;	//??,delta=+
		res2 = p->angle - p->last_angle;				//??	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//??	delta -
		res2 = p->angle - p->last_angle;				//??	delta +
	}
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}


/**
	**************************************************************
	** Descriptions: ����ͨ��
	** Input: 	ң��������
	**			   *P:�ṹ��
	**				
	** Output: NULL
	**************************************************************
**/
void CAN_GET_YK(RC_Ctl_t * RC , CAN_HandleTypeDef * hcan)
{
			RC->key.v = (hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	    RC->rc.ch0 = (hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]) ;
	    RC->rc.ch1 = (hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5]) ;
	    RC->rc.s1  =  hcan->pRxMsg->Data[6];
	    RC->rc.s2  =  hcan->pRxMsg->Data[7];
	
}	

//void CAN_GET_YT(moto_measure_t * YT , CAN_HandleTypeDef * hcan)
//{
//			YT->angle = (int16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
//	    YT->total_angle  =(int16_t) (hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]) ;
//	
//}	
void CAN_Send_YT( CAN_HandleTypeDef * hcan, int32_t time_three)
{    
			CANSend_YT.DLC = 0x08;
			CANSend_YT.IDE = CAN_ID_STD;
			CANSend_YT.RTR = CAN_RTR_DATA;
			CANSend_YT.StdId = 0x300;
	
			memset(CANSend_YT.Data, 0, sizeof(CANSend_YT.Data));
			CANSend_YT.Data[0]=time_three;
			CANSend_YT.Data[1]=0;
			CANSend_YT.Data[2]=0;
			CANSend_YT.Data[3]=0;
			CANSend_YT.Data[4]=0;
			CANSend_YT.Data[5]=0;
			CANSend_YT.Data[6]=0;
			CANSend_YT.Data[7]=0;
	
			hcan->pTxMsg = &CANSend_YT;
			HAL_CAN_Transmit(hcan,50);
}	

void CAN_GET_YT(Mode_Set * Chassis , CAN_HandleTypeDef * hcan)
{
		Chassis->mode =(hcan->pRxMsg->Data[0]) ;
	  Chassis->flag=(hcan->pRxMsg->Data[1]) ;
	  Chassis->minipc_rx_state_flag=(hcan->pRxMsg->Data[2]) ;
	  Chassis->minipc_rx_bopan_flag=(hcan->pRxMsg->Data[3]) ;
	  Chassis->stop_flag=(hcan->pRxMsg->Data[4]) ;
}	

void CAN_GET_Error(SystemStateDef * Error , CAN_HandleTypeDef * hcan)
{
			Error->OutLine_Flag = (hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	    Error->task_OutLine_Flag  = (hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]) ;
	
}	