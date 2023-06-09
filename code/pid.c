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
    static int16_t Pid_t;
    Pid_t = Pid_t+15;
    if(Pid_t % 150==0){
        Pid_t = 0;
        Encoder_C = encoder_get_count(ENCODER_3_QUADDEC);   //行进电机编码器
        /*150ms运行一次*/  
        PidLocCtrl(&Pitch_vel_pid, (Encoder_C*5/32)-Move_distance);   //速度环,期望速度为Move_distance
    }
    if(Pid_t % 30 == 0)
        /*30ms运行一次*/
        PidLocCtrl(&Pitch_angle_pid,Pitch_vel_pid.out - Pitch + zhongzhi);//角度环

    /*15ms运行一次*/
    // PidLocCtrl(&Pitch_acc_pid,-gyro_pitch + Pitch_angle_pid.out);  //角速度环

    // return Pitch_acc_pid.out;
    return Pitch_angle_pid.out;
}

float R_Cascade_Pid_Ctrl(float zhongzhi)
{
    static int16_t Pid_t = 0;
    Pid_t = Pid_t+15;
    // 1/2
    // if(Pid_t % 30 == 0){
    //     Pid_t = 0;
    //     PidLocCtrl(&Roll_angle_pid,Roll - zhongzhi);//角度环
    // }

    PidLocCtrl(&Roll_angle_pid,Roll - zhongzhi);//角度环
    
    // PidLocCtrl(&Roll_acc_pid,+gyro_yaw + Roll_angle_pid.out);  //角速度环

    // return Roll_acc_pid.out;
    return Roll_angle_pid.out;
}

void R_Angle_Pid_Init(pid_param_t * pid)
{
    pid->kp        = -15400;//90
    pid->ki        = 0.0;
    pid->kd        = -6150/*-4150*/;
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
    pid->kp        = -170;//90
    pid->ki        = 0.0;
    pid->kd        = -60;
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
float PidLocCtrl(pid_param_t * pid, float error)
{
    /* 累积误差 */
    pid->integrator += error;

    /* 误差限幅 */
    constrain_float(pid->integrator, -pid->imax, pid->imax);


    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);
//    pid->out_d = pid->kd * gyro_roll;
    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}


/*!
  * @brief    pid增量式控制器输出
  * @param    pid     pid参数
  * @param    error   pid输入误差
  * @return   PID输出结果   注意输出结果已经包涵了上次结果
  */
float PidIncCtrl(pid_param_t * pid, float error)
{

    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * (error - pid->last_error);
    pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}

