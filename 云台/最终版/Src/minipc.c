#include "minipc.h"



Minipc_Rx minipc_rx;
Minipc_Tx minipc_tx;

uint8_t USART6_RX_DATA[(SizeofMinipc)];		//MiniPC
uint16_t USART6_RX_NUM;


void Get_MiniPC_Data(void)
{
	uint8_t *buff = USART6_RX_DATA;
		
	minipc_rx.frame_header = buff[0];
	minipc_rx.frame_tail 	 = buff[8];
	if((minipc_rx.frame_header == 0xFF) && (minipc_rx.frame_tail == 0xFE))
	{
		minipc_rx.angle_yaw  = (int16_t)(buff[1]<<8|buff[2]);
		minipc_rx.angle_pit  = (int16_t)(buff[3]<<8|buff[4]);
		minipc_rx.state_flag = buff[5];
		minipc_rx.distance = buff[6]<<8|buff[7];
	}
}

//void Send_MiniPC_Data(void)
//{
////	minipc_tx.frame_header = 0xFF;
////	minipc_tx.cmd1 			   = (int16_t)(cmd1*174.5)>>8;
////	minipc_tx.cmd2 				 = (int16_t)cmd1*174.5;
////	minipc_tx.cmd3			   = (int16_t)(cmd2*174.5)>>8;
////	minipc_tx.cmd4 				 = (int16_t)cmd2*174.5;
////	minipc_tx.frame_tail   = 0xFE;
//	
//	
//	minipc_tx.send_buf[0] = 0xFF;
//	
//	minipc_tx.send_buf[1] = 0x00;
//	minipc_tx.send_buf[2] = 0x00;
//	minipc_tx.send_buf[3] = 0x00;
//	minipc_tx.send_buf[4] = 0x00;
//	minipc_tx.send_buf[5] = 0x01;
//	minipc_tx.send_buf[6] = 0x00;
//	minipc_tx.send_buf[7] = 0x00;
//	
//	minipc_tx.send_buf[8] = 0xFE;

//	
//	HAL_UART_Transmit(&huart2, minipc_tx.send_buf, sizeof(minipc_tx.send_buf), 10);

//}






