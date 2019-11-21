#ifndef  gun_task_H
#define  gun_task_H

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "Motor_use_can.h"
#include "minipc.h"
#include "main.h"


typedef struct Heat_Gun_t
{
	int16_t  shted_bullet;
	int16_t limt_bullet;
	int16_t last_limt_bullet;
	uint16_t limt_heat;
	uint16_t rel_heat;
	uint16_t last_rel_heat;
	float    remain_power;
	uint8_t  limt_spd;
	uint8_t  roboLevel;
	uint8_t  sht_flg;
	uint8_t  stop_flg;
	uint8_t  heat_down_flg;
}Heat_Gun_t;

volatile typedef struct 
{
	volatile uint16_t rel_heat;
	volatile float remain_power;
}Power_Heat;



void Gun_Pid_Init();
void Gun_Task(void const * argument);
void Mocha_Task(void const * argument);















#endif
