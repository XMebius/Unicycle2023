#include "pid.h"
#include "zf_common_headfile.h"

extern float Pitch,Roll,Yaw;
extern float gyro_yaw,gyro_pitch,gyro_roll;
extern float Move_distance;
/*!
  * @brief    限幅函数
  * @param    amt   ： 参数
  * @param    low   ： 最低值
  * @param    high  ： 最高值
  */
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

short constrain_short(short amt, short low, short high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

pid_param_t Pitch_acc_pid;      //C电机角速度环
pid_param_t Pitch_angle_pid;    //C电机角度环
pid_param_t Pitch_vel_pid;      //C电机速度环
pid_param_t Roll_acc_pid;      //AB电机角速度环
pid_param_t Roll_angle_pid;    //AB电机角度环

extern short Encoder_C;

void P_Acc_Pid_Init(pid_param_t * pid);
void P_Angle_Pid_Init(pid_param_t * pid);
void P_Vel_Pid_Init(pid_param_t * pid);
void R_Acc_Pid_Init(pid_param_t * pid);
void R_Angle_Pid_Init(pid_param_t * pid);

void My_Pid_P_Init(void)
{
    P_Acc_Pid_Init(&Pitch_acc_pid);
    P_Angle_Pid_Init(&Pitch_angle_pid);
    P_Vel_Pid_Init(&Pitch_vel_pid);
}
void My_Pid_R_Init(void)
{
    R_Acc_Pid_Init(&Roll_acc_pid);
    R_Angle_Pid_Init(&Roll_angle_pid);
}

float P_Cascade_Pid_Ctrl(float zhongzhi)
{
    PidLocCtrl(&Pitch_angle_pid,Pitch-zhongzhi,gyro_pitch);//角度环
    return Pitch_angle_pid.out;
}

float R_Cascade_Pid_Ctrl(float zhongzhi)
{
    PidLocCtrl(&Roll_angle_pid,Roll-zhongzhi, gyro_roll);//角度环
    return Roll_angle_pid.out;
}

void R_Angle_Pid_Init(pid_param_t * pid)
{
    pid->kp        = -15400;//90
    pid->ki        = 0.0;
    pid->kd        = -415/*-4150*/;
    pid->imax      = 120;

    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

/*串级PID参数*/
void P_Angle_Pid_Init(pid_param_t * pid)
{
    pid->kp        = 2350;//>3500
    pid->ki        = 0.0;
    pid->kd        = 0;
    pid->imax      = 30;

    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

/*串级PID参数*/
void P_Acc_Pid_Init(pid_param_t * pid)
{
    pid->kp        = 1.9; //10
    pid->ki        = 0;
    pid->kd        = 0;
    pid->imax      = 100;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}


/*串级PID参数*/
void P_Vel_Pid_Init(pid_param_t * pid)
{
    pid->kp        = 0.047;
    pid->ki        = 0.0;
    pid->kd        = 0.005;
    pid->imax      = 0.0;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

/*串级PID参数*/
void R_Acc_Pid_Init(pid_param_t * pid)
{
    pid->kp        = 45.0; //10
    pid->ki        = 0;
    pid->kd        = 0;
    pid->imax      = 100;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

/*串级PID参数*/

/*!
  * @brief    pid参数初始化函数
  */
void PidInit(pid_param_t * pid)
{
    pid->kp        = 0;
    pid->ki        = 0;
    pid->kd        = 0;
    pid->imax      = 0;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}


/*!
  * @brief    pid位置式控制器输出
  * @param    pid     pid参数
  * @param    error   pid输入误差
  */
float PidLocCtrl(pid_param_t* pid,float error,float dot_error)
{
    /* 累积误差 */
    pid->integrator += error;

    /* 误差限幅 */
    constrain_float(pid->integrator, -pid->imax, pid->imax);


    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * dot_error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}
