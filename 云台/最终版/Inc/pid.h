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

    float set[3];				//Ŀ��ֵ,����NOW�� LAST�� LLAST���ϴ�
    float get[3];				//����ֵ
    float err[3];				//���


    float pout;							//p���
    float iout;							//i���
    float dout;							//d���
	  float pout_max;
	  float iout_max;
	  float dout_max;
	  float dout_last;
	  float dout_new;

    float pos_out;						//����λ��ʽ���
    float last_pos_out;				//�ϴ����
    float delta_u;						//��������ֵ
    float delta_out;					//��������ʽ��� = last_delta_out + delta_u
    float last_delta_out;

    float max_err;
    float deadband;				//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;				//����޷�
    uint32_t IntegralLimit;		//�����޷�

    void (*f_param_init)(struct __pid_t *pid,  //PID������ʼ��
                         uint32_t pid_mode,
                         uint32_t maxOutput,
                         uint32_t integralLimit,
                         float p,
                         float i,
                         float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid���������޸�

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


extern pid_t pid_rub_spd[2];      //Ĩ���ٶȻ�		
extern pid_t pid_yaw;
extern pid_t pid_yaw_jy901;
extern pid_t pid_yaw_jy901_spd;
extern pid_t pid_pit_jy901;
extern pid_t pid_pit_jy901_spd;
extern pid_t pid_pit;
extern pid_t pid_yaw_spd;
extern pid_t pid_pit_spd;
extern pid_t pid_dial_pos;  //���̵��λ�û�
extern pid_t pid_dial_spd;	//�ٶȻ�
extern pid_t pid_3508_pos;
extern pid_t pid_3508_spd[4];
extern pid_t pid_3508_current[4];
extern pid_t pid_minipc_yaw;
extern pid_t pid_minipc_pit;
extern pid_t pid_chassis_follow ;//���̸���λ�û�
extern pid_t pid_chassis_follow_spd ;//���̸����ٶȻ�















#endif

