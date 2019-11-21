#ifndef  gimbal_task_H
#define  gimbal_task_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "pid.h"
#include "communication.h "
#include "Motor_use_can.h"
#include "FreeRTOS.h"
#include "can.h"
#include "user_lib.h"

typedef struct{
		//int16_t expect;
		float expect;
		uint8_t	step;
		uint8_t mode;
		int16_t expect_pc;
} Pos_Set;


typedef struct {
	
	uint8_t mode;
	uint8_t flag;
  uint8_t state;
  }Mode_Set;

	
extern  Mode_Set Gimbal;
extern Pos_Set  yaw_set;
extern Pos_Set  pit_set;
extern Mode_Set Chassis;
void Gimbal_Task(void const * argument);
extern float llastpit_angle;
extern ramp_function_source_t fric1_ramp;
extern ramp_function_source_t fric2_ramp;




#endif
