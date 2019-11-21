#include "pid.h"
#include "main.h"

#define ABS(x)		((x>0)? (x): (-x)) 

/**
	**************************************************************
	** Descriptions: 限幅函数
	** Input: 	
	**			   a :要限制的变量的指针
	**				ABS_MAX :幅值
	** Output: NULL
	**************************************************************
**/

void ABS_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}


/**
	**************************************************************
	** Descriptions: pid参数初始化
	** Input: 	
	**			   pid_t *    pid,    					pid的指针
	**    	   uint32_t 	mode, 						pid模式
	**  	 	 	 uint32_t	  maxout,						限幅值
	**    		 uint32_t 	intergral_limit,	积分环限幅
	**   		 	 float 	    kp, 
	**   			 float   		ki, 
	**   			 float 			kd
	** Output: NULL
	**************************************************************
**/
static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd)
{   
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;    
}


/**
	**************************************************************
	** Descriptions: 限幅中途更改参数设定函数(调试)
	** Input: 	
  **				*pid：要求更改的pid的指针	
	**			   kp :
	**				 ki :
  ** 				 kd ：
	** Output: NULL
	**************************************************************
**/
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}
//_________________________滤波函数，应该写于热量限制里——————
/*
** Descriptions: 一阶低通滤波
** Input: 
** Output: 滤波结果
*/
float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}


//—————— ——--  _____-----_-_-_-_________

/**
**************************************************************
	** Descriptions: pid计算函数
	** Input: 	
  **				 *pid :		要进行计算的pid的指针
	**					get :   实际值
	**					set :   目标值
	** Output: NULL
	**************************************************************
**/
float pid_calc(pid_t* pid, float get, float set){
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
		return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
		return 0;
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout_new = pid->d * (pid->err[NOW] - pid->err[LAST] );
			  
		    pid->dout = LPF_1st(pid->dout_last,pid->dout_new,0.6);
			
        ABS_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        ABS_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
			  pid->dout_last = pid->dout;
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        ABS_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        ABS_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//	
}

/**
    *@bref. special calculate position PID @attention @use @gyro data!!
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_sp_calc(pid_t* pid, float get, float set, float gyro){
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
				if(fabs(pid->i) >= 0.001f)
					pid->iout += pid->i * pid->err[NOW];
				else
					pid->iout = 0;
        pid->dout = -pid->d * gyro/100.0f;	
        ABS_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        ABS_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
//        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
//        pid->iout = pid->i * pid->err[NOW];
//        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
//        
//        ABS_limit(&(pid->iout), pid->IntegralLimit);
//        pid->delta_u = pid->pout + pid->iout + pid->dout;
//        pid->delta_out = pid->last_delta_out + pid->delta_u;
//        ABS_limit(&(pid->delta_out), pid->MaxOutput);
//        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//	
}
/**
	**************************************************************
	** Descriptions: pid参数初始化
	** Input: 	
	**			   pid_t *    pid,    					pid的指针
	**    	   uint32_t 	mode, 						pid模式
	**  	 	 	 uint32_t	  maxout,						限幅值
	**    		 uint32_t 	intergral_limit,	积分环限幅
	**   		 	 float 	    kp, 
	**   			 float   		ki, 
	**   			 float 			kd
	** Output: NULL
	**************************************************************
**/void PID_struct_init(
											pid_t* pid,
											uint32_t mode,
											uint32_t maxout,
											uint32_t intergral_limit,   
											float 	kp, 
											float 	ki, 
											float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);	
}

