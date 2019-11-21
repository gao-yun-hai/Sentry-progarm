#ifndef  minipc_H
#define  minipc_H

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "communication.h"
#include "usart.h"

typedef struct{

unsigned char 		frame_header; 		  //֡ͷ0xFF
 int16_t 					angle_yaw;     			//yaw angle
 int16_t 					angle_pit;     			//pitch angle 
unsigned char 		state_flag;     		//��ǰ״̬����0 δ��׼Ŀ�� ���� 1��ɨ�鵽��δ����Ŀ�꡿��2 ������ɨ�鵽������Ŀ�꡿��3 Զ����ɨ�鵽������Ŀ�꡿
	int16_t 					distance;     			//Ŀ����롾0 ������Ŀ�� ���� 1Զ����Ŀ�꡿
	int16_t           angle;
unsigned char 		frame_tail; 	  	  //֡β0xFE
	unsigned char 		colour;//0 blue;  1 red
	unsigned char 		bopan_flag;
}Minipc_Rx;

typedef struct{

//unsigned char 		frame_header; 		  //֡ͷ0xFF
//unsigned char 		cmd1;     					//cmd1
//unsigned char 		cmd2;     					//cmd2 
//unsigned char 		cmd3;     					//cmd1
//unsigned char 		cmd4;     					//cmd2 	
unsigned char send_buf[9];	
//unsigned char 		frame_tail; 	  	  //֡β0xFE
}Minipc_Tx;

extern Minipc_Rx minipc_rx;
extern Minipc_Tx minipc_tx;
extern uint16_t USART6_RX_NUM;
extern uint8_t USART6_RX_DATA[(SizeofMinipc)];	



void Get_MiniPC_Data(void);
void Send_MiniPC_Data(void);





#endif
