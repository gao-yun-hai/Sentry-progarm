#ifndef  _pid_h
#define  _pid_h

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


enum {
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,

    POSITION_PID,
    DELTA_PID,
};
typedef struct __pid_t
{
    float p;
    float i;
    float d;

    float set[3];				//目标值,包含NOW， LAST， LLAST上上次
    float get[3];				//测量值
    float err[3];				//误差


    float pout;							//p输出
    float iout;							//i输出
    float dout;							//d输出
	  float pout_max;
	  float iout_max;
	  float dout_max;
	  float dout_last;
	  float dout_new;

    float pos_out;						//本次位置式输出
    float last_pos_out;				//上次输出
    float delta_u;						//本次增量值
    float delta_out;					//本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;

    float max_err;
    float deadband;				//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;				//输出限幅
    uint32_t IntegralLimit;		//积分限幅

    void (*f_param_init)(struct __pid_t *pid,  //PID参数初始化
                         uint32_t pid_mode,
                         uint32_t maxOutput,
                         uint32_t integralLimit,
                         float p,
                         float i,
                         float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid三个参数修改

} pid_t;


void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float 	kp,
    float 	ki,
    float 	kd);

float pid_calc(pid_t* pid, float fdb, float ref);
float pid_sp_calc(pid_t* pid, float get, float set, float gyro);


extern pid_t pid_rub_spd[2];      //抹茶速度环		
extern pid_t pid_yaw;
extern pid_t pid_yaw_jy901;
extern pid_t pid_yaw_jy901_spd;
extern pid_t pid_pit_jy901;
extern pid_t pid_pit_jy901_spd;
extern pid_t pid_pit;
extern pid_t pid_yaw_spd;
extern pid_t pid_pit_spd;
extern pid_t pid_dial_pos;  //拨盘电机位置环
extern pid_t pid_dial_spd;	//速度环
extern pid_t pid_3508_pos;
extern pid_t pid_3508_spd[4];
extern pid_t pid_3508_current[4];
extern pid_t pid_minipc_yaw;
extern pid_t pid_minipc_pit;
extern pid_t pid_chassis_follow ;//底盘跟随位置环
extern pid_t pid_chassis_follow_spd ;//底盘跟随速度环















#endif

