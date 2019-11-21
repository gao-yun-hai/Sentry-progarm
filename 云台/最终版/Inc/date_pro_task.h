#ifndef  __data_pro_task_H
#define  __data_pro_task_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "pid.h"
#include "gimbal_task.h"
#include "communication.h"
#include "gun_task.h"
#include "can.h"

extern void RemoteControlProcess(); 
extern uint8_t h;
extern void MouseKeyControlProcess();
extern void Remote_Data_Task(void const * argument);
extern  void Minipc_Pid_Init();
extern void MiniPC_Data_task(void const * argument);














#endif
