#ifndef  __communication_h
#define  __communication_h

#include "stm32f4xx_hal.h"

#define SizeofMinipc  9
#define SizeofRemote 18
extern uint8_t USART1_RX_DATA[(SizeofRemote)];//遥控
extern uint16_t USART1_RX_NUM;

typedef struct    //外接陀螺仪    可以改成套用电机参数的结构体moto_measure_t  _待续
{
	float err;
	float JY901_angle;
	float JY901_angle_last;
	float first_angle;
	float angle_round;
  float final_angle;
  float last_final_angle;
	float vx;
	float vy;	
	float vz;
	float vx_last;
	float vy_last;
	float vz_last;
	int8_t frame;
	uint8_t times;
}JY901_t;





extern JY901_t  	ptr_jy901_t_yaw;//外接陀螺仪数据
extern JY901_t  	ptr_jy901_t_pit;
extern JY901_t    ptr_jy901_t_angular_velocity;

///////////////遥控/////////////////////
typedef struct //遥控器及键鼠通道
{ 
			int16_t x; //!< Byte 6-7 
			int16_t y; //!< Byte 8-9 
			int16_t z; //!< Byte 10-11 
			uint8_t press_l; //!< Byte 12 
			uint8_t press_r; //!< Byte 13 
}Mouse; 

typedef 	struct 
{ 
	 uint16_t ch0; 
	 uint16_t ch1; 
	 uint16_t ch2; 
	 uint16_t ch3; 
	 uint8_t s1; 
	 uint8_t s2; 
}Rc; 

typedef  struct 
{ 
		uint16_t v; //!< Byte 14-15 
}Key; 
		
typedef struct 
{ 
  Rc rc; 
  Mouse mouse; 
  Key key; 
}RC_Ctl_t; 


//遥控
void Remote_Ctrl(void);
void Remote_Disable(void);











#endif
